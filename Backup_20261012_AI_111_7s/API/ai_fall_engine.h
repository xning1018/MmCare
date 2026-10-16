#ifndef AI_FALL_ENGINE_H_
#define AI_FALL_ENGINE_H_

#include "hal_data.h"
#include "ld2450.h"
#include "radar_uart.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 4 类人体姿态流形定义（与上位机引导式训练工具严格对齐） */
typedef enum {
    AI_POSTURE_SLOW_WALK  = 0, /* Class 0: 正常慢走 */
    AI_POSTURE_FAST_TURN  = 1, /* Class 1: 快步走 / 急转身 / 挥手 */
    AI_POSTURE_BEND_SQUAT = 2, /* Class 2: 原地弯腰 / 慢速下蹲 */
    AI_POSTURE_FALL_ALARM = 3  /* Class 3: 失衡急剧跌倒并触地静止 */
} AI_PostureClass_t;

/* 端侧 AI 引擎实时状态输出结构体 */
typedef struct {
    /* 1024 粒子蒙特卡洛滤波后的最优人体质心状态 (LD2450) */
    int16_t  filtered_x_mm;
    int16_t  filtered_y_mm;
    int16_t  filtered_v_cms;
    float    filtered_r_m;
    float    filtered_v_ms;

    /* LD2450 PINN-CfC 液态神经网络 4 分类概率与姿态判别结果 */
    AI_PostureClass_t posture_class;
    float    class_probs[4];
    float    fall_confidence;
    float    physics_residual;

    /* LD6002C 三维点云多参数分析输出 */
    float    ld6002c_height_drop_m;    /* 窗口内 3D 质心高度下降幅度 ΔH (m) */
    float    ld6002c_peak_speed_ms;    /* 窗口内 3D 点云峰值冲击速度 (m/s) */
    float    ld6002c_fall_score;       /* LD6002C 多参数综合跌倒置信度 [0.0 ~ 1.0] */

    /* 双雷达独立并行触发状态（满足至少一项即触发跌倒报警） */
    bool     ld2450_ai_fall_triggered; /* 通道 A：LD2450 端侧 PINN-CfC AI 检出跌倒 */
    bool     ld6002c_fall_triggered;   /* 通道 B：LD6002C 三维点云多参数引擎检出跌倒 */
    int      combined_fall_code;       /* 0=正常, 111=LD2450触发, 1=LD6002C多参数触发, 11=双路同时触发 */

    /* 片上在线自进化统计 */
    uint32_t online_adapt_steps;
} AI_FallState_t;

/**
 * @brief 初始化瑞萨 RA6M5 端侧双雷达 AI 跌倒检测引擎
 */
void AI_FallEngine_Init(void);

/**
 * @brief 执行一步双雷达端侧 AI 推理（LD2450 轨迹液态网络 + LD6002C 三维点云多参数分析）
 * @param p_ld2450        LD2450 三目标轨迹数据包指针 (可为 NULL 表示本周期无新帧)
 * @param p_ld6002c       LD6002C 雷达完整多参数数据结构体指针 (含高度H、三维速度V、点云跨度、坠落速率等)
 * @param current_tick_ms 系统绝对毫秒时基 g_system_tick
 * @return true 表示处于跌倒报警状态 (LD2450 或 LD6002C 任一通道触发，或两者同时触发)，false 表示正常
 */
bool AI_FallEngine_Step(const LD2450_Data_t *p_ld2450,
                        const radar_data_t *p_ld6002c,
                        uint32_t current_tick_ms);

/**
 * @brief 获取端侧 AI 引擎只读状态指针
 */
const AI_FallState_t *AI_FallEngine_GetState(void);

/**
 * @brief 手动复位双通道跌倒锁存状态
 */
void AI_FallEngine_ResetAlarm(void);

#ifdef __cplusplus
}
#endif

#endif /* AI_FALL_ENGINE_H_ */
