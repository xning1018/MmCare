/* ============================================================================
 * API/ai_fall_engine.c
 * 瑞萨 RA6M5 (200MHz Cortex-M33 + 硬件 FPU + 512KB SRAM) 端侧双雷达 AI 跌倒检测核心引擎
 *
 * 核心算法架构（异构双雷达独立多参数 AI 判决 + 并行或门触发）：
 *   【主通道 A：LD2450 24GHz 轨迹与微多普勒液态神经网络引擎】
 *     1. 1024 粒子蒙特卡洛空间滤波器 (1024-Particle Monte Carlo Filter)
 *     2. 16点汉宁短窗 + 64点零填充滑动微多普勒 FFT (64-pt Zero-Padded STFT)
 *     3. 物理内嵌闭式连续时间液态神经网络 (PINN-CfC + Temporal Self-Attention)
 *     4. Flash INT8 量化存储 + 上电 SRAM FP32 反量化张量池 + ±15% 锚定在线 RLS 自蒸馏
 *
 *   【主通道 B：LD6002C 60GHz 三维点云多参数姿态与高度坍塌判别引擎】
 *     - 无需连接硬件 SCL0/IO 置位引脚，直接深度解析 UART 三维点云多维物理参数：
 *       (离地高度 H、垂直坠落速率 -dH/dt、三维移动速度 V、三维冲击加速度 a_3D、
 *        三维点云高宽跨度比 ΔHoriz/ΔVert、低位静止保持时长，并兼容 0x0E02 帧)
 *
 *   【并行触发规则（满足任一立即报警）】：
 *     - LD2450 识别出跌倒（无论 LD6002C 是否触发） -> 立即报警 (combined_fall_code = 111)
 *     - LD6002C 多参数识别出跌倒（无论 LD2450 是否触发） -> 立即报警 (combined_fall_code = 1)
 *     - LD2450 与 LD6002C 两者同时触发 -> 立即报警 (combined_fall_code = 11)
 * ============================================================================ */

#include "ai_fall_engine.h"
#include "ai_weights.h"
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ----------------------------------------------------------------------------
 * 宏定义与超参数配置
 * ---------------------------------------------------------------------------- */
#define AI_NUM_PARTICLES        1024u   /* 蒙特卡洛粒子数 (充分利用 RA6M5 512KB 大容量 SRAM) */
#define AI_GRID_SIZE            32u     /* 32x32 房间空间自标定网格 (覆盖 X:±3.2m, Y:0~6.4m) */
#define AI_TRAJ_WIN_LEN         16u     /* LD2450 滑动轨迹时序窗口长度 (与 Python 训练严格对齐为 16 点) */
#define AI_PC60_WIN_LEN         16u     /* LD6002C 三维点云多参数滑动窗口长度 (与 Python 训练严格对齐为 16 点) */
#define AI_STFT_WIN_LEN         16u     /* STFT 汉宁短窗长度 */
#define AI_STFT_FFT_LEN         64u     /* 零填充基2 FFT 点数 */
#define AI_FALL_HOLD_MS         8000u   /* 端侧 AI 检出跌倒后的最小报警保持时间 (ms) */
#define AI_ENABLE_ONLINE_RLS    0       /* 默认关闭未受人工审核的在线伪标签自训练，防止误报自我强化 */

/* ----------------------------------------------------------------------------
 * 1. 大容量 SRAM 状态张量与 FP32 反量化权重池 (静态分配于 ZI-data)
 * ---------------------------------------------------------------------------- */

typedef struct {
    float x_m;
    float y_m;
    float vx_ms;
    float vy_ms;
    float weight;
} AI_Particle_t;

typedef struct {
    float static_clutter_score;
    float dynamic_walk_score;
} AI_GridCell_t;

typedef struct {
    float    t_sec;
    float    r_m;
    float    v_ms;
    int16_t  x_mm;
    int16_t  y_mm;
} AI_TrajSample_t;

/* LD6002C 三维点云多参数时序采样点（物理量字段严格独立，不混用） */
typedef struct {
    float    t_sec;
    float    r_3d_m;        /* 三维空间径向距离 sqrt(x^2+y^2+z^2) (m) */
    float    y_radar_m;     /* 质心距雷达垂直距离 raw_y (m) */
    float    v_3d_ms;       /* 三维点云移动速度 V (m/s) */
    float    v_drop_ms;     /* 垂直向下跌落速度 -dH/dt (m/s) */
    float    acc_3d_ms2;    /* 三维加速度 (m/s^2) */
    float    aspect_ratio;  /* 点云水平/垂直跨度比 (卧姿特征) */
} AI_PC60Sample_t;

static AI_Particle_t   s_particles[AI_NUM_PARTICLES];
static AI_GridCell_t   s_room_grid[AI_GRID_SIZE][AI_GRID_SIZE];
static AI_TrajSample_t s_traj_buf[AI_TRAJ_WIN_LEN];
static uint32_t        s_traj_count = 0;

/* LD6002C 三维点云多参数滑动窗口与状态机 */
static AI_PC60Sample_t s_pc60_buf[AI_PC60_WIN_LEN];
static uint32_t        s_pc60_count = 0;
static uint32_t        s_last_pc60_sample_tick = 0;
static uint32_t        s_last_cfc_tick_b = 0;
static bool            s_ld6002c_fall_latched = false;
static uint32_t        s_ld6002c_fall_latch_tick = 0;

/* 上电从 Flash INT8 权重库反量化展开到高速 SRAM 的 FP32 核心权重张量池 */
static float s_w_tau_sram[AI_FEAT_DIM][AI_HIDDEN_DIM];
static float s_b_tau_sram[AI_HIDDEN_DIM];
static float s_w_g_sram[AI_FEAT_DIM][AI_HIDDEN_DIM];
static float s_b_g_sram[AI_HIDDEN_DIM];
static float s_w_pinn_sram[AI_FEAT_DIM][AI_HIDDEN_DIM];

/* PINN-CfC 液态隐状态 (通道 A 与通道 B 各自维护独立的连续时间液态微分记忆张量与真实更新时基) */
static float s_liquid_h[AI_HIDDEN_DIM];
static float s_liquid_h_60[AI_HIDDEN_DIM];
static uint32_t s_last_cfc_tick_a = 0;

/* SRAM 输出层权重与原型矩阵 (由 Flash INT8 反量化加载) */
static float s_w_out_online[AI_HIDDEN_DIM][AI_NUM_CLASSES];
static float s_b_out_online[AI_NUM_CLASSES];
static float s_prototypes_online[AI_NUM_CLASSES][AI_HIDDEN_DIM];
#if AI_ENABLE_ONLINE_RLS
static float s_rls_diag_p[AI_HIDDEN_DIM];
#endif

static AI_FallState_t s_ai_state;
static uint32_t       s_rng_state = 0x2026A6E5u;
static uint32_t       s_last_step_tick = 0;
static uint32_t       s_ai_fall_latch_tick = 0;
static bool           s_ai_fall_latched = false;
static bool           s_pf_initialized = false;

/* ----------------------------------------------------------------------------
 * Cortex-M33 硬件单精度 FPU 内联算子
 * ---------------------------------------------------------------------------- */

static inline float ai_fabsf(float x) {
    return (x < 0.0f) ? -x : x;
}

static inline float ai_clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float ai_sqrtf(float x) {
    if (x <= 0.0f) return 0.0f;
    float res;
    __asm volatile ("vsqrt.f32 %0, %1" : "=t"(res) : "t"(x));
    return res;
}

static inline float ai_expf(float x) {
    float xc = ai_clampf(x, -16.0f, 16.0f);
    float y = 1.0f + xc * (1.0f / 64.0f);
    if (y <= 0.0f) return 0.0f;
    y *= y;
    y *= y;
    y *= y;
    y *= y;
    y *= y;
    y *= y;
    return y;
}

static inline float ai_sigmoidf(float x) {
    return 1.0f / (1.0f + ai_expf(-x));
}

static inline float ai_tanhf(float x) {
    float e2x = ai_expf(2.0f * ai_clampf(x, -8.0f, 8.0f));
    return (e2x - 1.0f) / (e2x + 1.0f);
}

static inline float ai_sinf(float rad) {
    while (rad > M_PI)  rad -= 2.0f * M_PI;
    while (rad < -M_PI) rad += 2.0f * M_PI;
    const float B = 4.0f / M_PI;
    const float C = -4.0f / (M_PI * M_PI);
    float y = B * rad + C * rad * ai_fabsf(rad);
    return 0.225f * (y * ai_fabsf(y) - y) + y;
}

static inline float ai_cosf(float rad) {
    return ai_sinf(rad + 0.5f * M_PI);
}

static inline float ai_rand_uniform(void) {
    uint32_t x = s_rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_rng_state = x;
    return (float)(x & 0x00FFFFFFu) / 16777216.0f;
}

static inline float ai_rand_normal(void) {
    float sum = 0.0f;
    for (int i = 0; i < 6; i++) {
        sum += ai_rand_uniform();
    }
    return (sum - 3.0f) * 1.41421356f;
}

static inline void ai_map_to_grid(float x_m, float y_m, int *gx, int *gy) {
    int ix = (int)((x_m + 3.2f) * (32.0f / 6.4f));
    int iy = (int)(y_m * (32.0f / 6.4f));
    if (ix < 0) ix = 0;
    if (ix >= (int)AI_GRID_SIZE) ix = (int)AI_GRID_SIZE - 1;
    if (iy < 0) iy = 0;
    if (iy >= (int)AI_GRID_SIZE) iy = (int)AI_GRID_SIZE - 1;
    *gx = ix;
    *gy = iy;
}

/* ----------------------------------------------------------------------------
 * 2. 1024 粒子蒙特卡洛空间滤波器 (LD2450 通道 A)
 * ---------------------------------------------------------------------------- */
static bool ai_particle_filter_update(const LD2450_Data_t *p_ld2450,
                                      float dt_s,
                                      float *out_x_m,
                                      float *out_y_m,
                                      float *out_v_ms) {
    if (p_ld2450 == NULL) {
        return false;
    }

    int valid_cnt = 0;
    float obs_x[3], obs_y[3], obs_v[3];
    for (int i = 0; i < 3; i++) {
        if (p_ld2450->target[i].is_valid) {
            float xm = (float)p_ld2450->target[i].x / 1000.0f;
            float ym = (float)p_ld2450->target[i].y / 1000.0f;
            float vm = (float)p_ld2450->target[i].speed / 100.0f;
            float rm = ai_sqrtf(xm * xm + ym * ym);
            if (rm > 0.15f && rm < 6.5f) {
                obs_x[valid_cnt] = xm;
                obs_y[valid_cnt] = ym;
                obs_v[valid_cnt] = vm;
                valid_cnt++;
            }
        }
    }

    if (valid_cnt == 0) {
        return false;
    }

    /* 冷启动首帧锚定：开机/复位后第一次收到有效目标时，直接将 1024 粒子锚定在真实目标周围，
     * 彻底消除粒子从默认中心 (0, 2.75m) 滑向真实位置时产生的虚假高速收敛冲击！ */
    if (!s_pf_initialized) {
        float inv_n = 1.0f / (float)AI_NUM_PARTICLES;
        for (uint32_t i = 0; i < AI_NUM_PARTICLES; i++) {
            int k = (int)(i % (uint32_t)valid_cnt);
            s_particles[i].x_m = obs_x[k] + ai_rand_normal() * 0.05f;
            s_particles[i].y_m = obs_y[k] + ai_rand_normal() * 0.05f;
            s_particles[i].vx_ms = 0.0f;
            s_particles[i].vy_ms = 0.0f;
            s_particles[i].weight = inv_n;
        }
        s_pf_initialized = true;
        *out_x_m = obs_x[0];
        *out_y_m = obs_y[0];
        *out_v_ms = obs_v[0];
        return true;
    }

    for (int k = 0; k < valid_cnt; k++) {
        int gx, gy;
        ai_map_to_grid(obs_x[k], obs_y[k], &gx, &gy);
        if (ai_fabsf(obs_v[k]) < 0.03f) {
            s_room_grid[gy][gx].static_clutter_score =
                ai_clampf(s_room_grid[gy][gx].static_clutter_score * 0.995f + 0.01f, 0.0f, 1.0f);
        } else if (ai_fabsf(obs_v[k]) > 0.25f && ai_fabsf(obs_v[k]) < 1.6f) {
            s_room_grid[gy][gx].dynamic_walk_score =
                ai_clampf(s_room_grid[gy][gx].dynamic_walk_score * 0.995f + 0.02f, 0.0f, 1.0f);
            s_room_grid[gy][gx].static_clutter_score *= 0.95f;
        }
    }

    float weight_sum = 0.0f;
    for (uint32_t i = 0; i < AI_NUM_PARTICLES; i++) {
        AI_Particle_t *p = &s_particles[i];
        p->x_m += p->vx_ms * dt_s + ai_rand_normal() * 0.06f;
        p->y_m += p->vy_ms * dt_s + ai_rand_normal() * 0.06f;

        float best_likelihood = 1e-5f;
        float matched_v = obs_v[0];
        for (int k = 0; k < valid_cnt; k++) {
            float dx = p->x_m - obs_x[k];
            float dy = p->y_m - obs_y[k];
            float dist_sq = dx * dx + dy * dy;
            int gx, gy;
            ai_map_to_grid(obs_x[k], obs_y[k], &gx, &gy);

            float clutter_penalty = 1.0f - 0.65f * s_room_grid[gy][gx].static_clutter_score;
            float lh = ai_expf(-dist_sq / 0.28f) * clutter_penalty;
            if (lh > best_likelihood) {
                best_likelihood = lh;
                matched_v = obs_v[k];
            }
        }

        float r_norm = ai_sqrtf(p->x_m * p->x_m + p->y_m * p->y_m) + 1e-3f;
        p->vx_ms = 0.6f * p->vx_ms + 0.4f * matched_v * (p->x_m / r_norm);
        p->vy_ms = 0.6f * p->vy_ms + 0.4f * matched_v * (p->y_m / r_norm);

        p->weight *= best_likelihood;
        weight_sum += p->weight;
    }

    float est_x = 0.0f, est_y = 0.0f, neff_inv = 0.0f;
    if (weight_sum > 1e-9f) {
        float inv_w = 1.0f / weight_sum;
        for (uint32_t i = 0; i < AI_NUM_PARTICLES; i++) {
            s_particles[i].weight *= inv_w;
            float w = s_particles[i].weight;
            est_x += w * s_particles[i].x_m;
            est_y += w * s_particles[i].y_m;
            neff_inv += w * w;
        }
    } else {
        est_x = obs_x[0];
        est_y = obs_y[0];
        neff_inv = 1.0f;
    }

    if (neff_inv > (1.0f / 256.0f)) {
        float inv_n = 1.0f / (float)AI_NUM_PARTICLES;
        for (uint32_t i = 0; i < AI_NUM_PARTICLES; i++) {
            int k = (int)(i % (uint32_t)valid_cnt);
            s_particles[i].x_m = 0.7f * est_x + 0.3f * obs_x[k] + ai_rand_normal() * 0.08f;
            s_particles[i].y_m = 0.7f * est_y + 0.3f * obs_y[k] + ai_rand_normal() * 0.08f;
            s_particles[i].weight = inv_n;
        }
    }

    float best_v = obs_v[0];
    float min_d2 = 1e9f;
    for (int k = 0; k < valid_cnt; k++) {
        float d2 = (obs_x[k] - est_x) * (obs_x[k] - est_x) + (obs_y[k] - est_y) * (obs_y[k] - est_y);
        if (d2 < min_d2) {
            min_d2 = d2;
            best_v = obs_v[k];
        }
    }

    *out_x_m = est_x;
    *out_y_m = est_y;
    *out_v_ms = best_v;
    return true;
}

/* ----------------------------------------------------------------------------
 * 3. 16点汉宁短窗 + 64点零填充基2 FFT 微多普勒频谱提取
 * ---------------------------------------------------------------------------- */
static void ai_compute_stft_bands_64pt(const float *v_seq, uint32_t len, float out_bands[4]) {
    float re[AI_STFT_FFT_LEN];
    float im[AI_STFT_FFT_LEN];
    memset(re, 0, sizeof(re));
    memset(im, 0, sizeof(im));

    uint32_t n_win = (len < AI_STFT_WIN_LEN) ? len : AI_STFT_WIN_LEN;
    if (n_win > 0) {
        const float *seg = &v_seq[len - n_win];
        float denom = (n_win > 1u) ? (float)(n_win - 1u) : 1.0f;
        for (uint32_t i = 0; i < n_win; i++) {
            float hann = 0.5f - 0.5f * ai_cosf(2.0f * (float)M_PI * (float)i / denom);
            re[i] = seg[i] * hann;
        }
    }

    uint32_t j = 0;
    for (uint32_t i = 0; i < AI_STFT_FFT_LEN; i++) {
        if (i < j) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
        uint32_t m = AI_STFT_FFT_LEN >> 1;
        while (m >= 1u && j >= m) {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    for (uint32_t s = 1; s <= 6; s++) {
        uint32_t m = 1u << s;
        uint32_t half_m = m >> 1;
        float theta = -2.0f * (float)M_PI / (float)m;
        float w_m_re = ai_cosf(theta);
        float w_m_im = ai_sinf(theta);

        for (uint32_t k = 0; k < AI_STFT_FFT_LEN; k += m) {
            float w_re = 1.0f;
            float w_im = 0.0f;
            for (uint32_t idx = 0; idx < half_m; idx++) {
                uint32_t u = k + idx;
                uint32_t t = u + half_m;
                float tr = w_re * re[t] - w_im * im[t];
                float ti = w_re * im[t] + w_im * re[t];
                re[t] = re[u] - tr;
                im[t] = im[u] - ti;
                re[u] += tr;
                im[u] += ti;

                float next_w_re = w_re * w_m_re - w_im * w_m_im;
                w_im = w_re * w_m_im + w_im * w_m_re;
                w_re = next_w_re;
            }
        }
    }

    float power[32];
    float total_p = 0.0f;
    for (uint32_t k = 0; k < 32u; k++) {
        power[k] = re[k] * re[k] + im[k] * im[k] + 1e-6f;
        total_p += power[k];
    }

    float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f, b3 = 0.0f;
    for (uint32_t k = 0;  k < 4u;  k++) b0 += power[k];
    for (uint32_t k = 4;  k < 12u; k++) b1 += power[k];
    for (uint32_t k = 12; k < 21u; k++) b2 += power[k];
    for (uint32_t k = 21; k < 32u; k++) b3 += power[k];

    out_bands[0] = b0 / total_p;
    out_bands[1] = b1 / total_p;
    out_bands[2] = b2 / total_p;
    out_bands[3] = b3 / total_p;
}

/* ----------------------------------------------------------------------------
 * 4. 提取 LD2450 16 维物理内嵌时频流形特征向量
 * ---------------------------------------------------------------------------- */
static void ai_extract_16d_features(float out_feat[AI_FEAT_DIM], float *out_phys_residual) {
    memset(out_feat, 0, sizeof(float) * AI_FEAT_DIM);
    *out_phys_residual = 0.0f;

    uint32_t n = s_traj_count;
    if (n < 4u) {
        return;
    }

    float dt_arr[AI_TRAJ_WIN_LEN];
    float v_fused[AI_TRAJ_WIN_LEN];
    float a_arr[AI_TRAJ_WIN_LEN];
    float jerk_arr[AI_TRAJ_WIN_LEN];

    uint32_t vf_len = n - 1u;
    float sum_r = s_traj_buf[0].r_m;
    float min_r = s_traj_buf[0].r_m;
    float max_r = s_traj_buf[0].r_m;

    for (uint32_t i = 0; i < vf_len; i++) {
        float r0 = s_traj_buf[i].r_m;
        float r1 = s_traj_buf[i + 1u].r_m;
        sum_r += r1;
        if (r1 < min_r) min_r = r1;
        if (r1 > max_r) max_r = r1;

        float dt = ai_clampf(s_traj_buf[i + 1u].t_sec - s_traj_buf[i].t_sec, 0.05f, 1.0f);
        dt_arr[i] = dt;
        float vr_kin = (r1 - r0) / dt;
        v_fused[i] = 0.6f * s_traj_buf[i + 1u].v_ms + 0.4f * vr_kin;
    }

    uint32_t a_len = vf_len - 1u;
    float max_acc_abs = 0.0f;
    for (uint32_t i = 0; i < a_len; i++) {
        a_arr[i] = (v_fused[i + 1u] - v_fused[i]) / dt_arr[i + 1u];
        float aa = ai_fabsf(a_arr[i]);
        if (aa > max_acc_abs) max_acc_abs = aa;
    }

    float max_jerk_abs = 0.0f;
    if (a_len > 1u) {
        uint32_t j_len = a_len - 1u;
        for (uint32_t i = 0; i < j_len; i++) {
            jerk_arr[i] = (a_arr[i + 1u] - a_arr[i]) / dt_arr[i + 2u];
            float ja = ai_fabsf(jerk_arr[i]);
            if (ja > max_jerk_abs) max_jerk_abs = ja;
        }
    }

    uint32_t mid = (vf_len / 2u > 0u) ? (vf_len / 2u) : 1u;
    float sum_v = 0.0f, max_v_abs = 0.0f;
    float sum_v_early = 0.0f, sum_v_late = 0.0f;
    uint32_t zc_cnt = 0;

    for (uint32_t i = 0; i < vf_len; i++) {
        float va = ai_fabsf(v_fused[i]);
        sum_v += va;
        if (va > max_v_abs) max_v_abs = va;
        if (i < mid) {
            sum_v_early += va;
        } else {
            sum_v_late += va;
        }
        if (i > 0u) {
            if ((v_fused[i] >= 0.0f && v_fused[i - 1u] < 0.0f) ||
                (v_fused[i] < 0.0f && v_fused[i - 1u] >= 0.0f)) {
                zc_cnt++;
            }
        }
    }

    float mean_r = ai_clampf((sum_r / (float)n) / 6.0f, 0.0f, 1.5f);
    float span_r = ai_clampf((max_r - min_r) / 2.5f, 0.0f, 2.0f);
    float mean_v = ai_clampf((sum_v / (float)vf_len) / 2.0f, 0.0f, 2.0f);
    float max_v  = ai_clampf(max_v_abs / 2.5f, 0.0f, 2.5f);

    float mean_v_early = ai_clampf((sum_v_early / (float)mid) / 2.0f, 0.0f, 2.0f);
    uint32_t late_cnt = (vf_len > mid) ? (vf_len - mid) : 1u;
    float mean_v_late  = ai_clampf((sum_v_late / (float)late_cnt) / 2.0f, 0.0f, 2.0f);

    float max_acc  = ai_clampf(max_acc_abs / 6.0f, 0.0f, 2.5f);
    float max_jerk = ai_clampf(max_jerk_abs / 25.0f, 0.0f, 3.0f);

    float zero_crossings = (float)zc_cnt / (float)vf_len;
    float collapse_ratio = ai_clampf((mean_v_early - mean_v_late) / (mean_v_early + 0.15f), -1.0f, 1.5f);

    float r_late_mean = 0.0f;
    uint32_t r_late_cnt = n - mid;
    for (uint32_t i = mid; i < n; i++) {
        r_late_mean += s_traj_buf[i].r_m;
    }
    r_late_mean /= (float)r_late_cnt;

    float r_late_var = 0.0f;
    for (uint32_t i = mid; i < n; i++) {
        float diff = s_traj_buf[i].r_m - r_late_mean;
        r_late_var += diff * diff;
    }
    float r_late_std = ai_sqrtf(r_late_var / (float)r_late_cnt);
    float post_stillness = ai_expf(-r_late_std * 8.0f);
    float pinn_impact = ai_clampf(max_v * max_jerk * post_stillness, 0.0f, 3.0f);

    float bands[4];
    ai_compute_stft_bands_64pt(v_fused, vf_len, bands);

    out_feat[0]  = mean_r;
    out_feat[1]  = span_r;
    out_feat[2]  = mean_v;
    out_feat[3]  = max_v;
    out_feat[4]  = mean_v_early;
    out_feat[5]  = mean_v_late;
    out_feat[6]  = max_acc;
    out_feat[7]  = max_jerk;
    out_feat[8]  = zero_crossings;
    out_feat[9]  = collapse_ratio;
    out_feat[10] = post_stillness;
    out_feat[11] = pinn_impact;
    out_feat[12] = bands[0];
    out_feat[13] = bands[1];
    out_feat[14] = bands[2];
    out_feat[15] = bands[3];

    float expected_span = mean_v * 2.0f * ((float)n * 0.1f);
    *out_phys_residual = ai_fabsf((max_r - min_r) - expected_span);
}

/* ----------------------------------------------------------------------------
 * 5. PINN-CfC 物理内嵌液态神经网络前向推理 (双通道共享 SRAM FP32 权重池，独立液态微分记忆)
 * ---------------------------------------------------------------------------- */
static void ai_pinn_cfc_forward(const float feat[AI_FEAT_DIM],
                                float dt_s,
                                float liquid_state[AI_HIDDEN_DIM],
                                float out_probs[AI_NUM_CLASSES],
                                float out_h[AI_HIDDEN_DIM]) {
    float h_instant[AI_HIDDEN_DIM];
    float dt_clamped = ai_clampf(dt_s, 0.05f, 1.0f);

    for (uint32_t j = 0; j < AI_HIDDEN_DIM; j++) {
        float z_tau  = s_b_tau_sram[j];
        float z_g    = s_b_g_sram[j];
        float z_pinn = 0.0f;

        for (uint32_t i = 0; i < AI_FEAT_DIM; i++) {
            float xi = feat[i];
            z_tau  += xi * s_w_tau_sram[i][j];
            z_g    += xi * s_w_g_sram[i][j];
            z_pinn += xi * s_w_pinn_sram[i][j];
        }

        float tau_gate = ai_sigmoidf(z_tau);
        float g_act    = ai_tanhf(z_g);
        float pinn_act = (z_pinn > 0.0f) ? z_pinn : 0.0f;

        float h_target = tau_gate * g_act + (1.0f - tau_gate) * pinn_act;

        float tau_time_const = 0.08f + 0.25f * tau_gate;
        float decay = ai_expf(-dt_clamped / tau_time_const);
        liquid_state[j] = decay * liquid_state[j] + (1.0f - decay) * h_target;
        h_instant[j]    = 0.80f * h_target + 0.20f * liquid_state[j];
        out_h[j]        = h_instant[j];
    }

    float logits[AI_NUM_CLASSES];
    float max_logit = -1e9f;

    for (uint32_t c = 0; c < AI_NUM_CLASSES; c++) {
        float lg = s_b_out_online[c];
        float proto_dist_sq = 0.0f;
        for (uint32_t j = 0; j < AI_HIDDEN_DIM; j++) {
            lg += out_h[j] * s_w_out_online[j][c];
            float diff = out_h[j] - s_prototypes_online[c][j];
            proto_dist_sq += diff * diff;
        }
        lg -= 0.05f * proto_dist_sq;
        logits[c] = lg;
        if (lg > max_logit) {
            max_logit = lg;
        }
    }

    float sum_exp = 0.0f;
    for (uint32_t c = 0; c < AI_NUM_CLASSES; c++) {
        out_probs[c] = ai_expf(ai_clampf(logits[c] - max_logit, -16.0f, 0.0f));
        sum_exp += out_probs[c];
    }
    for (uint32_t c = 0; c < AI_NUM_CLASSES; c++) {
        out_probs[c] /= (sum_exp + 1e-7f);
    }
}

#if AI_ENABLE_ONLINE_RLS
/* ----------------------------------------------------------------------------
 * 6. 片上在线递推最小二乘 (RLS) (带 ±15% 弹性权重安全锚定，默认关闭以防伪标签自强化)
 * ---------------------------------------------------------------------------- */
static void ai_online_rls_distill(const float h_vec[AI_HIDDEN_DIM],
                                  uint32_t target_class,
                                  float lr_scale) {
    if (target_class >= AI_NUM_CLASSES) {
        return;
    }

    for (uint32_t j = 0; j < AI_HIDDEN_DIM; j++) {
        float hj = h_vec[j];
        float gain = (s_rls_diag_p[j] * hj) / (0.99f + hj * s_rls_diag_p[j] * hj);
        s_rls_diag_p[j] = ai_clampf((s_rls_diag_p[j] - gain * hj * s_rls_diag_p[j]) / 0.99f, 0.01f, 2.0f);

        for (uint32_t c = 0; c < AI_NUM_CLASSES; c++) {
            float y_target = (c == target_class) ? 1.0f : 0.0f;
            float err = y_target - s_ai_state.class_probs[c];
            float updated = s_w_out_online[j][c] + lr_scale * gain * err;

            float base_w = (float)g_ai_w_out_q8[j][c] * AI_SCALE_W_OUT;
            float max_delta = 0.15f * (ai_fabsf(base_w) + 0.25f);
            s_w_out_online[j][c] = ai_clampf(updated, base_w - max_delta, base_w + max_delta);
        }

        float base_p = (float)g_ai_prototypes_q8[target_class][j] * AI_SCALE_PROTOS;
        float p_delta = 0.15f * (ai_fabsf(base_p) + 0.30f);
        float new_p = 0.98f * s_prototypes_online[target_class][j] + 0.02f * hj;
        s_prototypes_online[target_class][j] = ai_clampf(new_p, base_p - p_delta, base_p + p_delta);
    }

    s_ai_state.online_adapt_steps++;
}
#endif

/* ----------------------------------------------------------------------------
 * 7. 主通道 B：LD6002C 60GHz 三维点云自标定 16 维流形提取 + PINN-CfC 液态神经网络引擎
 *    (自动在线学习当前挂高下的常态三维基准，提取与通道 A 同构的 16 维时频+64点STFT+三维形变流形，
 *     直接由 PINN-CfC 液态神经网络进行 4 分类连续概率推理)
 * ---------------------------------------------------------------------------- */
static float s_pc60_base_y_m = 0.0f;       /* 自适应标定的常态站立/行走垂直距离基线 */
static float s_pc60_base_aspect = 1.0f;    /* 自适应标定的常态站立点云宽高比基线 */

static bool ai_ld6002c_multiparam_step(const radar_data_t *p_ld6002c, uint32_t current_tick_ms) {
    if (p_ld6002c == NULL) {
        return s_ld6002c_fall_latched;
    }

    /* 1. 若 0x0E02 协议帧原生报跌倒，且已过开机前 5 秒冷启动期，直接置位 */
    if (current_tick_ms >= 5000u && p_ld6002c->is_fall == 1u && p_ld6002c->is_human != 0u) {
        s_ld6002c_fall_latched = true;
        s_ld6002c_fall_latch_tick = current_tick_ms;
        s_ai_state.ld6002c_fall_score = 1.0f;
    }

    /* 2. 每 120ms 对 LD6002C 的三维点云多参数进行滑动窗口采样并送入 PINN-CfC 液态网络 */
    bool pc_fresh = p_ld6002c->height_updated &&
                    (p_ld6002c->is_human != 0u || (current_tick_ms - p_ld6002c->last_pc_tick < 1500u));

    if (pc_fresh && (current_tick_ms - s_last_pc60_sample_tick >= 120u)) {
        s_last_pc60_sample_tick = current_tick_ms;

        /* 使用原始三维点云坐标与速度，完全不依赖任何假定的 2.2m 安装高度 */
        float raw_y = p_ld6002c->height_radar_m;
        float raw_x = p_ld6002c->pc_avg_x_m;
        float raw_z = p_ld6002c->pc_avg_z_m;
        float r_3d  = ai_sqrtf(raw_x * raw_x + raw_y * raw_y + raw_z * raw_z);
        if (r_3d < 0.10f) r_3d = raw_y;

        float cur_v  = p_ld6002c->speed_m_s;
        float v_drop = p_ld6002c->pc_v_drop_ms;
        float acc_3d = p_ld6002c->pc_acc_3d_ms2;
        float aspect = p_ld6002c->pc_span_horiz_m / (p_ld6002c->pc_span_vert_m + 0.12f);

        /* 在线自适应标定当前挂高下的常态人体基准 (当人体处于平稳散步速度 0.20~1.10 m/s 时慢速跟踪) */
        if (s_pc60_base_y_m < 0.10f && raw_y > 0.20f) {
            s_pc60_base_y_m = raw_y;
            s_pc60_base_aspect = (aspect > 0.2f) ? aspect : 1.0f;
        } else if (cur_v >= 0.20f && cur_v <= 1.10f && ai_fabsf(v_drop) < 0.22f) {
            s_pc60_base_y_m = 0.98f * s_pc60_base_y_m + 0.02f * raw_y;
            if (aspect > 0.1f) {
                s_pc60_base_aspect = 0.98f * s_pc60_base_aspect + 0.02f * aspect;
            }
        }

        AI_PC60Sample_t smp;
        smp.t_sec        = (float)current_tick_ms / 1000.0f;
        smp.r_3d_m       = r_3d;         /* 真实三维径向距离 r_3d (m) */
        smp.y_radar_m    = raw_y;        /* 质心距雷达垂直距离 raw_y (m) */
        smp.v_3d_ms      = cur_v;        /* 三维点云移动速度 (m/s) */
        smp.v_drop_ms    = v_drop;       /* 垂直下坠速度分量 (m/s) */
        smp.acc_3d_ms2   = acc_3d;       /* 三维加速度 (m/s^2) */
        smp.aspect_ratio = aspect;       /* 点云水平/垂直跨度比 */

        if (s_pc60_count < AI_PC60_WIN_LEN) {
            s_pc60_buf[s_pc60_count++] = smp;
        } else {
            memmove(&s_pc60_buf[0], &s_pc60_buf[1], sizeof(AI_PC60Sample_t) * (AI_PC60_WIN_LEN - 1u));
            s_pc60_buf[AI_PC60_WIN_LEN - 1u] = smp;
        }

        /* 当窗口积累至少 10 个点时，提取 16 维归一化时频流形特征并送入 PINN-CfC 液态神经网络 */
        if (s_pc60_count >= 10u) {
            float dt_cfc_b = (s_last_cfc_tick_b > 0u && current_tick_ms >= s_last_cfc_tick_b)
                                 ? ((float)(current_tick_ms - s_last_cfc_tick_b) / 1000.0f)
                                 : 0.12f;
            s_last_cfc_tick_b = current_tick_ms;

            uint32_t n = s_pc60_count;
            uint32_t vf_len = n - 1u;
            float dt_arr[AI_PC60_WIN_LEN];
            float v_seq[AI_PC60_WIN_LEN];
            float a_arr[AI_PC60_WIN_LEN];

            float sum_r = s_pc60_buf[0].r_3d_m;
            float min_r = s_pc60_buf[0].r_3d_m;
            float max_r = s_pc60_buf[0].r_3d_m;
            float min_y = s_pc60_buf[0].y_radar_m;
            float max_y = s_pc60_buf[0].y_radar_m;

            for (uint32_t i = 0; i < vf_len; i++) {
                float r0 = s_pc60_buf[i].r_3d_m;
                float r1 = s_pc60_buf[i + 1u].r_3d_m;
                float y1 = s_pc60_buf[i + 1u].y_radar_m;
                sum_r += r1;
                if (r1 < min_r) min_r = r1;
                if (r1 > max_r) max_r = r1;
                if (y1 < min_y) min_y = y1;
                if (y1 > max_y) max_y = y1;

                float dt = ai_clampf(s_pc60_buf[i + 1u].t_sec - s_pc60_buf[i].t_sec, 0.05f, 1.0f);
                dt_arr[i] = dt;
                float vr_kin = (r1 - r0) / dt;
                /* 融合三维点云速度、垂直下坠速度与径向导数速度 */
                float v_comb = 0.5f * s_pc60_buf[i + 1u].v_3d_ms +
                               0.3f * ai_fabsf(s_pc60_buf[i + 1u].v_drop_ms) +
                               0.2f * ai_fabsf(vr_kin);
                v_seq[i] = (s_pc60_buf[i + 1u].v_drop_ms >= 0.0f) ? v_comb : -v_comb;
            }

            uint32_t a_len = vf_len - 1u;
            float max_acc_abs = ai_fabsf(s_pc60_buf[n - 1u].acc_3d_ms2);
            for (uint32_t i = 0; i < a_len; i++) {
                a_arr[i] = (v_seq[i + 1u] - v_seq[i]) / dt_arr[i + 1u];
                float aa = ai_fabsf(a_arr[i]);
                if (aa > max_acc_abs) max_acc_abs = aa;
            }

            float max_jerk_abs = 0.0f;
            if (a_len > 1u) {
                for (uint32_t i = 0; i < a_len - 1u; i++) {
                    float ja = ai_fabsf((a_arr[i + 1u] - a_arr[i]) / dt_arr[i + 2u]);
                    if (ja > max_jerk_abs) max_jerk_abs = ja;
                }
            }

            uint32_t mid = (vf_len / 2u > 0u) ? (vf_len / 2u) : 1u;
            float sum_v = 0.0f, max_v_abs = 0.0f;
            float sum_v_early = 0.0f, sum_v_late = 0.0f;
            uint32_t zc_cnt = 0;

            for (uint32_t i = 0; i < vf_len; i++) {
                float va = ai_fabsf(v_seq[i]);
                sum_v += va;
                if (va > max_v_abs) max_v_abs = va;
                if (i < mid) sum_v_early += va;
                else         sum_v_late  += va;
                if (i > 0u && ((v_seq[i] >= 0.0f && v_seq[i - 1u] < 0.0f) ||
                               (v_seq[i] < 0.0f && v_seq[i - 1u] >= 0.0f))) {
                    zc_cnt++;
                }
            }

            /* 计算相对于自标定基线的无量纲垂直偏移率与包络形变率 */
            float rel_y_shift = (max_y - min_y) / (s_pc60_base_y_m + 0.35f);
            float rel_aspect_gain = aspect / (s_pc60_base_aspect + 0.25f);
            s_ai_state.ld6002c_height_drop_m = max_y - min_y;
            s_ai_state.ld6002c_peak_speed_ms = max_v_abs;

            float mean_r = ai_clampf((sum_r / (float)n) / 6.0f, 0.0f, 1.5f);
            float span_r = ai_clampf(((max_r - min_r) + rel_y_shift * 0.8f) / 2.5f, 0.0f, 2.0f);
            float mean_v = ai_clampf((sum_v / (float)vf_len) / 2.0f, 0.0f, 2.0f);
            float max_v  = ai_clampf(max_v_abs / 2.5f, 0.0f, 2.5f);

            float mean_v_early = ai_clampf((sum_v_early / (float)mid) / 2.0f, 0.0f, 2.0f);
            uint32_t late_cnt  = (vf_len > mid) ? (vf_len - mid) : 1u;
            float mean_v_late  = ai_clampf((sum_v_late / (float)late_cnt) / 2.0f, 0.0f, 2.0f);

            float max_acc  = ai_clampf(max_acc_abs / 6.0f, 0.0f, 2.5f);
            float max_jerk = ai_clampf(max_jerk_abs / 25.0f, 0.0f, 3.0f);
            float zero_crossings = (float)zc_cnt / (float)vf_len;
            float collapse_ratio = ai_clampf((mean_v_early - mean_v_late) / (mean_v_early + 0.15f), -1.0f, 1.5f);

            /* 后半窗三维位置静止度 */
            float r_late_mean = 0.0f;
            for (uint32_t i = mid; i < n; i++) r_late_mean += s_pc60_buf[i].r_3d_m;
            r_late_mean /= (float)(n - mid);
            float r_late_var = 0.0f;
            for (uint32_t i = mid; i < n; i++) {
                float d = s_pc60_buf[i].r_3d_m - r_late_mean;
                r_late_var += d * d;
            }
            float post_stillness = ai_expf(-ai_sqrtf(r_late_var / (float)(n - mid)) * 8.0f);
            float pinn_impact = ai_clampf(max_v * max_jerk * post_stillness * ai_clampf(rel_aspect_gain, 0.8f, 1.6f), 0.0f, 3.0f);

            float bands[4];
            ai_compute_stft_bands_64pt(v_seq, vf_len, bands);

            float feat_60[AI_FEAT_DIM];
            feat_60[0]  = mean_r;
            feat_60[1]  = span_r;
            feat_60[2]  = mean_v;
            feat_60[3]  = max_v;
            feat_60[4]  = mean_v_early;
            feat_60[5]  = mean_v_late;
            feat_60[6]  = max_acc;
            feat_60[7]  = max_jerk;
            feat_60[8]  = zero_crossings;
            feat_60[9]  = collapse_ratio;
            feat_60[10] = post_stillness;
            feat_60[11] = pinn_impact;
            feat_60[12] = bands[0];
            feat_60[13] = bands[1];
            feat_60[14] = bands[2];
            feat_60[15] = bands[3];

            /* 直接调用真机训练的 PINN-CfC 液态神经网络对 LD6002C 的 16 维流形进行 4 分类概率推理！ */
            float probs_60[AI_NUM_CLASSES];
            float h_vec_60[AI_HIDDEN_DIM];
            ai_pinn_cfc_forward(feat_60, dt_cfc_b, s_liquid_h_60, probs_60, h_vec_60);

            s_ai_state.ld6002c_fall_score = probs_60[AI_POSTURE_FALL_ALARM];

            /* 由 PINN-CfC 神经网络的 Class 3 (跌倒) 概率 + 相对物理守恒触发（仅在开机 5 秒热机稳定后允许触发） */
            if (current_tick_ms >= 5000u &&
                probs_60[AI_POSTURE_FALL_ALARM] >= 0.56f &&
                max_v >= 0.38f && post_stillness >= 0.42f && collapse_ratio >= 0.25f) {
                s_ld6002c_fall_latched = true;
                s_ld6002c_fall_latch_tick = current_tick_ms;
            }

            /* 若 PINN-CfC 识别到人体已恢复正常慢走/快走 (Class 0/1)，自动解除锁存 */
            if (s_ld6002c_fall_latched && (current_tick_ms - s_ld6002c_fall_latch_tick > 3000u)) {
                if ((probs_60[AI_POSTURE_SLOW_WALK] > 0.55f || probs_60[AI_POSTURE_FAST_TURN] > 0.55f) &&
                    cur_v > 0.35f) {
                    s_ld6002c_fall_latched = false;
                }
            }
        }
    }

    if (current_tick_ms < 5000u) {
        s_ld6002c_fall_latched = false;
    } else if (s_ld6002c_fall_latched && (current_tick_ms - s_ld6002c_fall_latch_tick > AI_FALL_HOLD_MS)) {
        if (p_ld6002c->is_human == 0u || p_ld6002c->speed_m_s > 0.30f) {
            s_ld6002c_fall_latched = false;
        }
    }

    return s_ld6002c_fall_latched;
}

/* ----------------------------------------------------------------------------
 * 公开 API 实现
 * ---------------------------------------------------------------------------- */

void AI_FallEngine_Init(void) {
    memset(&s_ai_state, 0, sizeof(s_ai_state));
    memset(s_room_grid, 0, sizeof(s_room_grid));
    memset(s_traj_buf, 0, sizeof(s_traj_buf));
    memset(s_pc60_buf, 0, sizeof(s_pc60_buf));
    memset(s_liquid_h, 0, sizeof(s_liquid_h));
    memset(s_liquid_h_60, 0, sizeof(s_liquid_h_60));

    s_traj_count = 0;
    s_pc60_count = 0;
    s_last_pc60_sample_tick = 0;
    s_last_cfc_tick_a = 0;
    s_last_cfc_tick_b = 0;
    s_ld6002c_fall_latched = false;
    s_ld6002c_fall_latch_tick = 0;
    s_pc60_base_y_m = 0.0f;
    s_pc60_base_aspect = 1.0f;

    s_last_step_tick = 0;
    s_ai_fall_latch_tick = 0;
    s_ai_fall_latched = false;
    s_pf_initialized = false;

    /* 1. 初始化 1024 空间粒子均匀分布在雷达视野中心区域 */
    float inv_n = 1.0f / (float)AI_NUM_PARTICLES;
    for (uint32_t i = 0; i < AI_NUM_PARTICLES; i++) {
        s_particles[i].x_m = (ai_rand_uniform() - 0.5f) * 3.0f;
        s_particles[i].y_m = 1.0f + ai_rand_uniform() * 3.5f;
        s_particles[i].vx_ms = 0.0f;
        s_particles[i].vy_ms = 0.0f;
        s_particles[i].weight = inv_n;
    }

    /* 2. 将 Flash 中的 INT8 对称量化权重反量化展开至 RA6M5 高速 SRAM FP32 张量池 */
    for (uint32_t i = 0; i < AI_FEAT_DIM; i++) {
        for (uint32_t j = 0; j < AI_HIDDEN_DIM; j++) {
            s_w_tau_sram[i][j]  = (float)g_ai_w_tau_q8[i][j]  * AI_SCALE_W_TAU;
            s_w_g_sram[i][j]    = (float)g_ai_w_g_q8[i][j]    * AI_SCALE_W_G;
            s_w_pinn_sram[i][j] = (float)g_ai_w_pinn_q8[i][j] * AI_SCALE_W_PINN;
        }
    }

    for (uint32_t j = 0; j < AI_HIDDEN_DIM; j++) {
        s_b_tau_sram[j] = (float)g_ai_b_tau_q8[j] * AI_SCALE_B_TAU;
        s_b_g_sram[j]   = (float)g_ai_b_g_q8[j]   * AI_SCALE_B_G;
#if AI_ENABLE_ONLINE_RLS
        s_rls_diag_p[j] = 0.25f;
#endif
        for (uint32_t c = 0; c < AI_NUM_CLASSES; c++) {
            s_w_out_online[j][c] = (float)g_ai_w_out_q8[j][c] * AI_SCALE_W_OUT;
            s_prototypes_online[c][j] = (float)g_ai_prototypes_q8[c][j] * AI_SCALE_PROTOS;
        }
    }

    for (uint32_t c = 0; c < AI_NUM_CLASSES; c++) {
        s_b_out_online[c] = (float)g_ai_b_out_q8[c] * AI_SCALE_B_OUT;
    }

    s_ai_state.class_probs[0] = 1.0f;
    s_ai_state.posture_class = AI_POSTURE_SLOW_WALK;
}

bool AI_FallEngine_Step(const LD2450_Data_t *p_ld2450,
                        const radar_data_t *p_ld6002c,
                        uint32_t current_tick_ms) {
    float dt_s = 0.10f;
    if (s_last_step_tick > 0u && current_tick_ms >= s_last_step_tick) {
        dt_s = (float)(current_tick_ms - s_last_step_tick) / 1000.0f;
    }
    if (dt_s < 0.02f) dt_s = 0.02f;
    if (dt_s > 0.50f) dt_s = 0.50f;

    /* 1. 独立运行【通道 B：LD6002C 三维点云多参数跌倒分析引擎】 */
    bool ld6002c_fall_now = ai_ld6002c_multiparam_step(p_ld6002c, current_tick_ms);

    /* 2. 独立运行【通道 A：LD2450 1024 粒子滤波与 PINN-CfC 液态网络引擎】 */
    if (p_ld2450 != NULL) {
        s_last_step_tick = current_tick_ms;

        float fx_m = 0.0f, fy_m = 0.0f, fv_ms = 0.0f;
        if (ai_particle_filter_update(p_ld2450, dt_s, &fx_m, &fy_m, &fv_ms)) {
            float fr_m = ai_sqrtf(fx_m * fx_m + fy_m * fy_m);
            s_ai_state.filtered_x_mm  = (int16_t)(fx_m * 1000.0f);
            s_ai_state.filtered_y_mm  = (int16_t)(fy_m * 1000.0f);
            s_ai_state.filtered_v_cms = (int16_t)(fv_ms * 100.0f);
            s_ai_state.filtered_r_m   = fr_m;
            s_ai_state.filtered_v_ms  = fv_ms;

            AI_TrajSample_t smp;
            smp.t_sec = (float)current_tick_ms / 1000.0f;
            smp.r_m   = fr_m;
            smp.v_ms  = fv_ms;
            smp.x_mm  = s_ai_state.filtered_x_mm;
            smp.y_mm  = s_ai_state.filtered_y_mm;

            if (s_traj_count < AI_TRAJ_WIN_LEN) {
                s_traj_buf[s_traj_count++] = smp;
            } else {
                memmove(&s_traj_buf[0], &s_traj_buf[1], sizeof(AI_TrajSample_t) * (AI_TRAJ_WIN_LEN - 1u));
                s_traj_buf[AI_TRAJ_WIN_LEN - 1u] = smp;
            }

            /* 必须等 16 点轨迹滑窗完全填满后再进行特征提取与推理，杜绝开机未满窗时的残缺瞬态误判 */
            if (s_traj_count >= AI_TRAJ_WIN_LEN) {
                float dt_cfc_a = (s_last_cfc_tick_a > 0u && current_tick_ms >= s_last_cfc_tick_a)
                                     ? ((float)(current_tick_ms - s_last_cfc_tick_a) / 1000.0f)
                                     : dt_s;
                s_last_cfc_tick_a = current_tick_ms;

                float feat[AI_FEAT_DIM];
                float phys_res = 0.0f;
                ai_extract_16d_features(feat, &phys_res);
                s_ai_state.physics_residual = phys_res;

                float probs[AI_NUM_CLASSES];
                float h_vec[AI_HIDDEN_DIM];
                ai_pinn_cfc_forward(feat, dt_cfc_a, s_liquid_h, probs, h_vec);

                memcpy(s_ai_state.class_probs, probs, sizeof(probs));
                s_ai_state.fall_confidence = probs[AI_POSTURE_FALL_ALARM];

                uint32_t best_cls = 0;
                float best_p = probs[0];
                for (uint32_t c = 1; c < AI_NUM_CLASSES; c++) {
                    if (probs[c] > best_p) {
                        best_p = probs[c];
                        best_cls = c;
                    }
                }
                s_ai_state.posture_class = (AI_PostureClass_t)best_cls;

                bool kinetic_valid = (feat[3] >= 0.42f) && (feat[10] >= 0.40f) && (phys_res < 2.2f);
                bool pinn_direct_fall = (feat[3] >= 0.55f) && (feat[6] >= 0.35f) &&
                                        (feat[9] >= 0.32f) && (feat[10] >= 0.52f);

                /* 仅在开机/复位满 5 秒后允许锁存跌倒报警 */
                if (current_tick_ms >= 5000u &&
                    ((probs[AI_POSTURE_FALL_ALARM] >= 0.56f && kinetic_valid) || pinn_direct_fall)) {
                    s_ai_fall_latched = true;
                    s_ai_fall_latch_tick = current_tick_ms;
                }

                if (s_ai_fall_latched && (current_tick_ms - s_ai_fall_latch_tick > 3000u)) {
                    if (ai_fabsf(fv_ms) > 0.48f &&
                        (best_cls == AI_POSTURE_SLOW_WALK || best_cls == AI_POSTURE_FAST_TURN)) {
                        s_ai_fall_latched = false;
                    }
                }

#if AI_ENABLE_ONLINE_RLS
                if (ld6002c_fall_now && feat[3] >= 0.35f) {
                    ai_online_rls_distill(h_vec, AI_POSTURE_FALL_ALARM, 0.015f);
                }
#endif
            }
        }
    }

    if (current_tick_ms < 5000u) {
        s_ai_fall_latched = false;
        ld6002c_fall_now = false;
    } else if (s_ai_fall_latched && (current_tick_ms - s_ai_fall_latch_tick > AI_FALL_HOLD_MS)) {
        if (ai_fabsf(s_ai_state.filtered_v_ms) > 0.25f) {
            s_ai_fall_latched = false;
        }
    }

    /* 3. 双雷达独立并行触发合成（满足任意一路，或两路同时触发，均立即报警！） */
    s_ai_state.ld2450_ai_fall_triggered = s_ai_fall_latched;
    s_ai_state.ld6002c_fall_triggered   = ld6002c_fall_now;

    if (s_ai_state.ld2450_ai_fall_triggered && s_ai_state.ld6002c_fall_triggered) {
        s_ai_state.combined_fall_code = 11;  /* 双雷达同时触发报警 */
    } else if (s_ai_state.ld2450_ai_fall_triggered) {
        s_ai_state.combined_fall_code = 111; /* LD2450 端侧 AI 独立触发报警 */
    } else if (s_ai_state.ld6002c_fall_triggered) {
        s_ai_state.combined_fall_code = 1;   /* LD6002C 多参数引擎独立触发报警 */
    } else {
        s_ai_state.combined_fall_code = 0;   /* 正常状态 */
    }

    return (s_ai_state.combined_fall_code != 0);
}

const AI_FallState_t *AI_FallEngine_GetState(void) {
    return &s_ai_state;
}

void AI_FallEngine_ResetAlarm(void) {
    s_ai_fall_latched = false;
    s_ld6002c_fall_latched = false;
    s_ai_state.ld2450_ai_fall_triggered = false;
    s_ai_state.ld6002c_fall_triggered = false;
    s_ai_state.combined_fall_code = 0;
}
