#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MMCare 瑞萨 RA6M5 端侧 AI 跌倒检测系统
分步倒计时引导式多姿态采集与 PINN-CfC (物理内嵌闭式液态神经网络) 可视化训练工具

支持两种模式：
1. 可视化大屏 GUI 界面模式 (默认直接运行或双击 .bat 启动):
   - 自带 72号超大字号倒计时（站在 4 米外雷达垫上也能清晰看见）
   - 自带 3秒准备倒计时 + 蜂鸣声音提示（躺在垫子上闭眼听声音即可知道开始和结束）
   - 实时显示当前雷达距离、速度与累计样本数
   - 一键训练并自动导出 C 权重头文件 API/ai_weights.h
2. 命令行初始化模式 (--init-baseline):
   - 生成纯净物理动力学基线权重 API/ai_weights.h
"""

import os
import sys
import time
import math
import json
import argparse
import threading
import urllib.request
import numpy as np

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

try:
    import winsound
except ImportError:
    winsound = None

SERVER_BASE = "http://182.254.135.92:8080"
API_KEY = "mmcare-demo-key-2026"
DATASET_PATH = os.path.join("datasets", "mmcare_real_dataset.npz")
WEIGHTS_HEADER_PATH = os.path.join("API", "ai_weights.h")

INPUT_DIM = 16
HIDDEN_DIM = 32
NUM_CLASSES = 3  # 0: 正常(含慢走/站立/轻微转身/洗浴微动), 1: 弯腰/下蹲/坐下, 2: 跌倒报警


def play_beep(freq=880, duration_ms=200):
    if winsound:
        try:
            winsound.Beep(freq, duration_ms)
        except Exception:
            pass


def fetch_latest_reading():
    """从云服务器获取最新一帧 4G 实时上报数据（自动兼容旧固件分米单位折算为真实米 m）"""
    url = f"{SERVER_BASE}/api/data/latest?api_key={API_KEY}"
    req = urllib.request.Request(url, headers={"X-API-Key": API_KEY})
    try:
        with urllib.request.urlopen(req, timeout=1.5) as resp:
            data = json.loads(resp.read().decode("utf-8"))
            if data.get("code") == 200 and data.get("data"):
                d = data["data"]
                raw_dist = abs(float(d.get("distance") or 0.0))
                if raw_dist > 5.0:
                    raw_dist *= 0.1
                d["distance"] = raw_dist
                d["h_m"] = float(d.get("h_m") or 0.0)
                d["v60_ms"] = float(d.get("v60_ms") or 0.0)
                d["v_drop_ms"] = float(d.get("v_drop_ms") or 0.0)
                return d
    except Exception:
        pass
    return None


def compute_stft_bands(v_seq):
    """
    对速度序列执行 16点短窗汉宁加权 + 64点零填充 FFT (与 RA6M5 片上 C 算法严格一致)
    返回 4 个微多普勒核心频带的归一化能量
    """
    n_win = min(len(v_seq), 16)
    padded = np.zeros(64, dtype=np.float32)
    if n_win > 0:
        seg = np.array(v_seq[-n_win:], dtype=np.float32)
        hann = 0.5 - 0.5 * np.cos(2.0 * np.pi * np.arange(n_win) / max(n_win - 1, 1))
        padded[:n_win] = seg * hann

    spec = np.abs(np.fft.rfft(padded))[:32]
    power = spec ** 2 + 1e-6
    total_p = np.sum(power)

    b0 = float(np.sum(power[0:4]) / total_p)
    b1 = float(np.sum(power[4:12]) / total_p)
    b2 = float(np.sum(power[12:21]) / total_p)
    b3 = float(np.sum(power[21:32]) / total_p)
    return b0, b1, b2, b3


def extract_16d_features(traj_window):
    """
    从连续轨迹窗口 [(t, r_m, speed_ms), ...] 中提取与 RA6M5 片上完全一致的 16 维时频流形特征向量
    严格对齐片上 AI_TRAJ_WIN_LEN = 16 点滑窗长度
    """
    if len(traj_window) > 16:
        traj_window = traj_window[-16:]
    n = len(traj_window)
    if n < 3:
        return np.zeros(INPUT_DIM, dtype=np.float32)

    r_arr = np.array([p[1] for p in traj_window], dtype=np.float32)
    v_arr = np.array([p[2] for p in traj_window], dtype=np.float32)
    t_arr = np.array([p[0] for p in traj_window], dtype=np.float32)

    dt_arr = np.clip(np.diff(t_arr), 0.05, 1.0)
    dr_arr = np.diff(r_arr)
    vr_kin = dr_arr / dt_arr

    v_fused = 0.6 * v_arr[1:] + 0.4 * vr_kin
    a_arr = np.diff(v_fused) / dt_arr[1:]
    jerk_arr = np.diff(a_arr) / dt_arr[2:] if len(a_arr) > 1 else np.array([0.0], dtype=np.float32)

    mid = max(1, len(v_fused) // 2)
    v_first_half = np.abs(v_fused[:mid])
    v_second_half = np.abs(v_fused[mid:])

    mean_r = float(np.clip(np.mean(r_arr) / 6.0, 0.0, 1.5))
    span_r = float(np.clip((np.max(r_arr) - np.min(r_arr)) / 2.5, 0.0, 2.0))
    mean_v = float(np.clip(np.mean(np.abs(v_fused)) / 2.0, 0.0, 2.0))
    max_v = float(np.clip(np.max(np.abs(v_fused)) / 2.5, 0.0, 2.5))

    mean_v_early = float(np.clip(np.mean(v_first_half) / 2.0, 0.0, 2.0))
    mean_v_late = float(np.clip(np.mean(v_second_half) / 2.0, 0.0, 2.0))

    max_acc = float(np.clip(np.max(np.abs(a_arr)) / 6.0, 0.0, 2.5))
    max_jerk = float(np.clip(np.max(np.abs(jerk_arr)) / 25.0, 0.0, 3.0))

    zero_crossings = float(np.sum(np.diff(np.sign(v_fused)) != 0)) / max(len(v_fused), 1)
    collapse_ratio = float(np.clip((mean_v_early - mean_v_late) / (mean_v_early + 0.15), -1.0, 1.5))

    r_late_std = float(np.std(r_arr[mid:]))
    post_stillness = float(np.exp(-r_late_std * 8.0))
    pinn_impact = float(np.clip(max_v * max_jerk * post_stillness, 0.0, 3.0))

    # 双雷达特征融合：若存在 LD6002C 的真实高度 H 与垂直坠速 -dH/dt，深度融合至坍塌比与失衡冲击
    has_h60 = len(traj_window[0]) > 3 and any(p[3] > 0.05 for p in traj_window)
    if has_h60:
        h_arr = np.array([p[3] if len(p) > 3 and p[3] > 0.05 else 1.5 for p in traj_window], dtype=np.float32)
        vd_arr = np.array([abs(p[4]) if len(p) > 4 else 0.0 for p in traj_window], dtype=np.float32)
        h_drop = max(0.0, float(h_arr[0] - np.min(h_arr)))
        max_vd = float(np.max(vd_arr))
        collapse_ratio = float(np.clip(0.5 * collapse_ratio + 0.5 * (h_drop / 1.2), -1.0, 1.5))
        pinn_impact = float(np.clip(pinn_impact * (1.0 + 0.6 * min(max_vd, 2.5)), 0.0, 3.0))

    b0, b1, b2, b3 = compute_stft_bands(v_fused)

    return np.array([
        mean_r, span_r, mean_v, max_v,
        mean_v_early, mean_v_late, max_acc, max_jerk,
        zero_crossings, collapse_ratio, post_stillness, pinn_impact,
        b0, b1, b2, b3
    ], dtype=np.float32)


def generate_physics_prior_dataset(samples_per_class=250, seed=42):
    """
    生成 16 点滑窗物理动力学基线样本（0: 正常, 1: 弯腰/下蹲/坐下, 2: 失衡跌倒），
    同时返回按独立事件分组的 group_ids，杜绝训练/验证集之间的数据泄漏。
    """
    rng = np.random.RandomState(seed)
    X_list, y_list, g_list = [], [], []
    win_len = 16
    group_counter = 0

    for cls in range(NUM_CLASSES):
        for _ in range(samples_per_class):
            t_seq = np.cumsum(rng.uniform(0.08, 0.14, size=win_len))
            if cls == 0:
                # 0: 正常（涵盖正常活动、慢走、微步、站立擦拭与转身活动）
                mode = rng.rand()
                if mode < 0.45:
                    # 慢步移动/进出洗浴区
                    v0 = rng.uniform(0.35, 1.10) * rng.choice([-1.0, 1.0])
                    freq = rng.uniform(1.0, 1.9)
                    v_seq = v0 + 0.18 * np.sin(2 * np.pi * freq * t_seq) + rng.normal(0, 0.06, size=win_len)
                    r_seq = np.clip(2.0 + np.cumsum(v_seq * 0.10), 0.5, 5.5)
                elif mode < 0.75:
                    # 原地站立洗浴、擦拭、抬手微动
                    v_seq = rng.normal(0.0, 0.12, size=win_len) + 0.08 * np.sin(2 * np.pi * 0.9 * t_seq)
                    r_seq = np.clip(rng.uniform(1.2, 2.5) + np.cumsum(v_seq * 0.03), 0.5, 5.5)
                else:
                    # 洗浴区内较快转身/避水
                    v1 = rng.uniform(0.9, 1.8) * rng.choice([-1.0, 1.0])
                    v_seq = np.concatenate([
                        np.full(7, v1) + rng.normal(0, 0.08, 7),
                        np.array([0.0, -v1 * 0.5]),
                        np.full(7, -v1) + rng.normal(0, 0.08, 7)
                    ])
                    r_seq = np.clip(2.2 + np.cumsum(v_seq * 0.09), 0.4, 5.5)
            elif cls == 1:
                # 1: 原地弯腰 / 下蹲 / 坐下 / 拾物难例（受控减速，躯干位移小）
                r_base = rng.uniform(1.0, 3.5)
                if rng.rand() < 0.6:
                    center = rng.uniform(0.5, 1.1)
                    v_seq = 0.55 * np.exp(-((t_seq - center) ** 2) / 0.15) * np.sin(2 * np.pi * 0.8 * (t_seq - center))
                    v_seq += rng.normal(0, 0.04, size=win_len)
                else:
                    v_seq = np.zeros(win_len, dtype=np.float32)
                    v_seq[:7] = rng.uniform(0.5, 1.15) * np.sin(np.linspace(0, np.pi, 7))
                    v_seq[7:] = rng.normal(0, 0.08, size=win_len - 7)
                r_seq = np.clip(r_base + np.cumsum(v_seq * 0.07), 0.5, 5.5)
            else:
                # 2: 失衡快速跌倒 + 触地急停静止
                r_base = rng.uniform(1.2, 4.2)
                v_seq = np.zeros(win_len, dtype=np.float32)
                impact_idx = rng.randint(4, 8)
                peak_v = rng.uniform(1.45, 3.2) * rng.choice([-1.0, 1.0])
                v_seq[impact_idx - 2] = peak_v * 0.35
                v_seq[impact_idx - 1] = peak_v * 0.85
                v_seq[impact_idx] = peak_v
                v_seq[impact_idx + 1] = -peak_v * 0.25
                v_seq[impact_idx + 2:] = rng.normal(0, 0.018, size=win_len - (impact_idx + 2))
                r_seq = np.clip(r_base + np.cumsum(v_seq * 0.09), 0.5, 5.5)

            traj = list(zip(t_seq, np.abs(r_seq), v_seq))
            X_list.append(extract_16d_features(traj))
            y_list.append(cls)
            g_list.append(group_counter)
            group_counter += 1

    return (
        np.array(X_list, dtype=np.float32),
        np.array(y_list, dtype=np.int64),
        np.array(g_list, dtype=np.int64)
    )


def dealias_and_interpolate_traj(raw_traj, is_fall=False):
    """
    将 4G 云端轮询获得的真实轨迹序列 [(t, r_m, v_ms, h_m, vd_ms, v60_ms), ...] 
    进行去重与 10Hz (dt=0.10s) 连续时间线性插值，100% 忠实保留雷达硬件真实测得的物理数据，
    绝不加入任何人工模拟峰值或伪造数据！
    """
    if len(raw_traj) < 3:
        return raw_traj

    anchors = [raw_traj[0]]
    for pt in raw_traj[1:]:
        prev = anchors[-1]
        if abs(pt[1] - prev[1]) > 1e-4 or abs(pt[2] - prev[2]) > 1e-4 or (pt[0] - prev[0]) >= 0.95:
            anchors.append(pt)
    if anchors[-1][0] < raw_traj[-1][0]:
        anchors.append(raw_traj[-1])

    if len(anchors) < 2:
        t_grid = np.arange(len(raw_traj), dtype=np.float32) * 0.10
        return [(float(t_grid[i]), float(raw_traj[i][1]), float(raw_traj[i][2])) for i in range(len(raw_traj))]

    t_anc = np.array([p[0] for p in anchors], dtype=np.float32)
    r_anc = np.array([p[1] for p in anchors], dtype=np.float32)
    v_anc = np.array([p[2] for p in anchors], dtype=np.float32)
    h_anc = np.array([p[3] if len(p) > 3 else 0.0 for p in anchors], dtype=np.float32)
    vd_anc = np.array([p[4] if len(p) > 4 else 0.0 for p in anchors], dtype=np.float32)

    t_span = max(float(t_anc[-1] - t_anc[0]), 1.6)
    n_pts = max(16, int(round(t_span / 0.10)))
    t_dense = np.linspace(float(t_anc[0]), float(t_anc[-1]), n_pts)
    r_dense = np.interp(t_dense, t_anc, r_anc)
    v_dense = np.interp(t_dense, t_anc, v_anc)
    h_dense = np.interp(t_dense, t_anc, h_anc)
    vd_dense = np.interp(t_dense, t_anc, vd_anc)
    t_mcu = np.arange(n_pts, dtype=np.float32) * 0.10

    # 100% 采用雷达硬件真实测量的速度与高度，绝不人为捏造峰值或假数据
    return [(float(t_mcu[i]), float(r_dense[i]), float(v_dense[i]), float(h_dense[i]), float(vd_dense[i])) for i in range(n_pts)]


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -15.0, 15.0)))


def softmax(logits):
    z = logits - np.max(logits, axis=-1, keepdims=True)
    e = np.exp(z)
    return e / np.sum(e, axis=-1, keepdims=True)


def quantize_dequantize_array(arr):
    """模拟 RA6M5 片上 INT8 对称量化与上电 FP32 反量化过程"""
    max_abs = float(np.max(np.abs(arr)))
    scale = max_abs / 127.0 if max_abs > 1e-6 else 1e-4
    q_arr = np.clip(np.round(arr / scale), -127, 127).astype(np.int8)
    return q_arr.astype(np.float32) * np.float32(scale)


def cfc_forward_batch(X, params, dt=0.12, X_prev=None):
    """
    与 API/ai_fall_engine.c 中 ai_pinn_cfc_forward 严格一致的闭式液态连续时间前向传播：
    包含液态时间常数 tau_val、指数衰减状态更新 h_state、以及原型流形距离修正项 -0.05 * ||h - P_c||^2
    """
    W_tau, b_tau = params["W_tau"], params["b_tau"]
    W_g, b_g = params["W_g"], params["b_g"]
    W_pinn = params["W_pinn"]
    W_out, b_out = params["W_out"], params["b_out"]
    prototypes = params["prototypes"]

    if X_prev is None:
        # 构造前一时刻微扰稳态输入，模拟连续时间步递推
        X_prev = X * 0.85

    tau_prev = sigmoid(X_prev @ W_tau + b_tau)
    g_prev = np.tanh(X_prev @ W_g + b_g)
    pinn_prev = np.maximum(0.0, X_prev @ W_pinn)
    h_prev = tau_prev * g_prev + (1.0 - tau_prev) * pinn_prev

    tau_gate = sigmoid(X @ W_tau + b_tau)
    g_act = np.tanh(X @ W_g + b_g)
    pinn_act = np.maximum(0.0, X @ W_pinn)
    h_target = tau_gate * g_act + (1.0 - tau_gate) * pinn_act

    tau_val = 0.08 + 0.92 * tau_gate
    decay = np.exp(-dt / tau_val)
    h_state = decay * h_prev + (1.0 - decay) * h_target
    h = 0.80 * h_target + 0.20 * h_state

    # 计算分类头输出 + 原型欧氏距离平方项 (-0.05 * ||h - P_c||^2)
    # h: (N, H), prototypes: (C, H) -> dist_sq: (N, C)
    diff = h[:, None, :] - prototypes[None, :, :]
    dist_sq = np.sum(diff ** 2, axis=-1)
    logits = (h @ W_out + b_out) - 0.05 * dist_sq
    probs = softmax(logits)

    cache = {
        "X_prev": X_prev, "h_prev": h_prev,
        "tau_gate": tau_gate, "g_act": g_act, "pinn_act": pinn_act,
        "h_target": h_target, "tau_val": tau_val, "decay": decay,
        "h_state": h_state, "h": h, "dist_sq": dist_sq,
        "logits": logits, "probs": probs
    }
    return probs, cache


def split_train_val_by_group(X, y, groups=None, val_ratio=0.20, seed=42):
    """
    按动作采集批次 (group) 与类别分层划分训练集与独立验证集，
    确保同一动作序列的滑窗切片或增强样本绝不会同时出现在训练集和验证集中。
    """
    rng = np.random.RandomState(seed)
    N = len(y)
    if groups is None or len(groups) != N:
        groups = np.arange(N, dtype=np.int64)

    train_mask = np.ones(N, dtype=bool)
    for c in range(NUM_CLASSES):
        cls_indices = np.where(y == c)[0]
        if len(cls_indices) < 2:
            continue
        cls_groups = np.unique(groups[cls_indices])
        rng.shuffle(cls_groups)
        n_val_groups = max(1, int(round(len(cls_groups) * val_ratio)))
        if len(cls_groups) > 1:
            val_g = set(cls_groups[:n_val_groups])
            for idx in cls_indices:
                if groups[idx] in val_g:
                    train_mask[idx] = False
        else:
            # 如果该类只有 1 个 group，按时间顺序取最后 20% 作为验证集
            split_pt = max(1, int(len(cls_indices) * (1.0 - val_ratio)))
            train_mask[cls_indices[split_pt:]] = False

    val_mask = ~train_mask
    return X[train_mask], y[train_mask], X[val_mask], y[val_mask]


def train_pinn_cfc_network(X_train, y_train, X_val=None, y_val=None, epochs=380, lr=0.045):
    """
    训练带物理运动学残差约束 (PINN) 与连续时间液态递推 (CfC) 的分类器，
    并在 INT8 量化 + FP32 反量化后的独立验证集上评估真实准确率、跌倒召回率与误报率。
    """
    np.random.seed(2026)
    if X_val is None or y_val is None or len(y_val) == 0:
        X_train, y_train, X_val, y_val = split_train_val_by_group(X_train, y_train)

    N, D = X_train.shape
    H = HIDDEN_DIM
    C = NUM_CLASSES
    dt = 0.12

    W_tau = np.random.randn(D, H).astype(np.float32) * np.sqrt(1.0 / D)
    b_tau = np.zeros(H, dtype=np.float32) + 0.5
    W_g = np.random.randn(D, H).astype(np.float32) * np.sqrt(2.0 / D)
    b_g = np.zeros(H, dtype=np.float32)
    W_pinn = np.random.randn(D, H).astype(np.float32) * np.sqrt(1.0 / D)
    for idx in [3, 6, 7, 9, 10, 11, 15]:
        W_pinn[idx, :H // 2] += 0.45

    W_out = np.random.randn(H, C).astype(np.float32) * np.sqrt(1.0 / H)
    b_out = np.zeros(C, dtype=np.float32)
    prototypes = np.zeros((C, H), dtype=np.float32)

    Y_onehot = np.eye(C, dtype=np.float32)[y_train]

    # 物理约束目标量：
    # 1) 运动学违背掩码：当最大加速度极低 (feat[6] < 0.25) 或后半程仍在持续移动 (feat[10] < 0.20) 时，
    #    物理上不可能属于“快速失衡冲击并倒地静止”，若输出高跌倒概率则触发物理残差惩罚
    phys_violation = np.maximum(0.0, 0.25 - X_train[:, 6]) + np.maximum(0.0, 0.20 - X_train[:, 10])
    # 2) 物理分支能量对齐目标 E_kin(x)：将 W_pinn 激活均值对齐到运动冲击度物理泛函
    e_kin_target = (0.35 * X_train[:, 3] + 0.35 * X_train[:, 6] + 0.30 * X_train[:, 11])[:, None]

    # 构造序列上一拍特征 X_prev 以训练 CfC 液态时间常数递推状态
    X_prev = np.roll(X_train, 1, axis=0)
    same_cls = (y_train == np.roll(y_train, 1))[:, None]
    X_prev = np.where(same_cls, 0.75 * X_train + 0.25 * X_prev, 0.85 * X_train).astype(np.float32)

    lambda_phys_prob = 0.25
    lambda_phys_branch = 0.08

    for ep in range(epochs):
        params = {
            "W_tau": W_tau, "b_tau": b_tau,
            "W_g": W_g, "b_g": b_g,
            "W_pinn": W_pinn, "W_out": W_out, "b_out": b_out,
            "prototypes": prototypes
        }
        probs, cache = cfc_forward_batch(X_train, params, dt=dt, X_prev=X_prev)
        h = cache["h"]
        h_prev = cache["h_prev"]
        h_target = cache["h_target"]
        tau_gate = cache["tau_gate"]
        tau_val = cache["tau_val"]
        decay = cache["decay"]
        g_act = cache["g_act"]
        pinn_act = cache["pinn_act"]

        # 动态更新类别流形原型
        for c in range(C):
            mask = (y_train == c)
            if np.any(mask):
                cls_mean = np.mean(h[mask], axis=0)
                prototypes[c] = 0.85 * prototypes[c] + 0.15 * cls_mean if ep > 0 else cls_mean

        # 1. 交叉熵主梯度 + PINN 运动学禁止区跌倒概率惩罚梯度
        d_logits = (probs - Y_onehot) / N
        fall_class_idx = C - 1  # 跌倒类别索引 (3分类下为 2)
        p_fall = probs[:, fall_class_idx]
        # d/d(logits) of 0.5 * lambda_phys_prob * phys_violation * p_fall^2
        d_p_fall = (lambda_phys_prob * phys_violation * p_fall) / N
        fall_onehot = np.zeros(C, dtype=np.float32)
        fall_onehot[fall_class_idx] = 1.0
        d_logits += (d_p_fall * p_fall)[:, None] * (fall_onehot - probs)

        dW_out = h.T @ d_logits + 1e-4 * W_out
        db_out = np.sum(d_logits, axis=0)

        # 包含原型距离项 -0.05 * ||h - P_c||^2 对 h 的梯度
        proto_grad_h = -0.10 * (
            h * np.sum(d_logits, axis=1, keepdims=True) - (d_logits @ prototypes)
        )
        dh = d_logits @ W_out.T + proto_grad_h

        # CfC 连续时间递推状态 h = (1 - 0.2 * decay) * h_target + 0.2 * decay * h_prev
        alpha_target = 1.0 - 0.20 * decay
        dh_target = dh * alpha_target
        d_decay = dh * 0.20 * (h_prev - h_target)
        # decay = exp(-dt / tau_val), tau_val = 0.08 + 0.92 * tau_gate
        dtau_from_decay = d_decay * decay * (dt / (tau_val ** 2)) * 0.92

        dg_act = dh_target * tau_gate * (1.0 - g_act ** 2)
        dW_g = X_train.T @ dg_act + 1e-4 * W_g
        db_g = np.sum(dg_act, axis=0)

        dtau = (dh_target * (g_act - pinn_act) + dtau_from_decay) * tau_gate * (1.0 - tau_gate)
        dW_tau = X_train.T @ dtau + 1e-4 * W_tau
        db_tau = np.sum(dtau, axis=0)

        # 2. PINN 物理分支能量对齐残差梯度
        pinn_mean = np.mean(pinn_act, axis=1, keepdims=True)
        d_pinn_phys = (lambda_phys_branch * (pinn_mean - e_kin_target) / (N * H))
        dpinn = (dh_target * (1.0 - tau_gate) + d_pinn_phys) * (X_train @ W_pinn > 0).astype(np.float32)
        dW_pinn = X_train.T @ dpinn + 1e-4 * W_pinn

        cur_lr = lr * (0.995 ** ep)
        W_out -= cur_lr * dW_out
        b_out -= cur_lr * db_out
        W_g -= cur_lr * dW_g
        b_g -= cur_lr * db_g
        W_tau -= cur_lr * dW_tau
        b_tau -= cur_lr * db_tau
        W_pinn -= cur_lr * dW_pinn

    # 锁定训练集最终原型
    fp32_params = {
        "W_tau": W_tau, "b_tau": b_tau,
        "W_g": W_g, "b_g": b_g,
        "W_pinn": W_pinn, "W_out": W_out, "b_out": b_out,
        "prototypes": prototypes
    }
    _, train_cache = cfc_forward_batch(X_train, fp32_params, dt=dt, X_prev=X_prev)
    h_final = train_cache["h"]
    for c in range(C):
        mask = (y_train == c)
        if np.any(mask):
            prototypes[c] = np.mean(h_final[mask], axis=0)

    # 模拟 INT8 对称量化 + FP32 反量化后的真实片上参数
    q_params = {
        "W_tau": quantize_dequantize_array(W_tau),
        "b_tau": quantize_dequantize_array(b_tau),
        "W_g": quantize_dequantize_array(W_g),
        "b_g": quantize_dequantize_array(b_g),
        "W_pinn": quantize_dequantize_array(W_pinn),
        "W_out": quantize_dequantize_array(W_out),
        "b_out": quantize_dequantize_array(b_out),
        "prototypes": quantize_dequantize_array(prototypes),
    }

    train_probs, _ = cfc_forward_batch(X_train, q_params, dt=dt, X_prev=X_prev)
    train_preds = np.argmax(train_probs, axis=1)
    train_acc = float(np.mean(train_preds == y_train)) * 100.0

    val_probs, _ = cfc_forward_batch(X_val, q_params, dt=dt)
    val_preds = np.argmax(val_probs, axis=1)
    val_acc = float(np.mean(val_preds == y_val)) * 100.0

    # 计算独立验证集上的跌倒召回率 (Recall) 与非跌倒误报率 (FPR)
    fall_mask = (y_val == 2)
    non_fall_mask = (y_val != 2)
    fall_recall = float(np.mean(val_preds[fall_mask] == 2)) * 100.0 if np.any(fall_mask) else 100.0
    fall_fpr = float(np.mean(val_preds[non_fall_mask] == 2)) * 100.0 if np.any(non_fall_mask) else 0.0

    return {
        "W_tau": W_tau, "b_tau": b_tau,
        "W_g": W_g, "b_g": b_g,
        "W_pinn": W_pinn,
        "W_out": W_out, "b_out": b_out,
        "prototypes": prototypes,
        "accuracy": val_acc,
        "train_accuracy": train_acc,
        "fall_recall": fall_recall,
        "fall_fpr": fall_fpr,
        "total_samples": N + len(y_val),
        "val_samples": len(y_val)
    }


def quantize_int8_2d(name, scale_macro, arr):
    rows, cols = arr.shape
    max_abs = float(np.max(np.abs(arr)))
    scale = max_abs / 127.0 if max_abs > 1e-6 else 1e-4
    q_arr = np.clip(np.round(arr / scale), -127, 127).astype(np.int8)
    lines = [
        f"#define {scale_macro} ({scale:.7f}f)",
        f"static const int8_t {name}[{rows}][{cols}] = {{"
    ]
    for r in range(rows):
        vals = ", ".join(f"{int(v):4d}" for v in q_arr[r])
        lines.append(f"  {{ {vals} }},")
    lines.append("};\n")
    return "\n".join(lines)


def quantize_int8_1d(name, scale_macro, arr):
    max_abs = float(np.max(np.abs(arr)))
    scale = max_abs / 127.0 if max_abs > 1e-6 else 1e-4
    q_arr = np.clip(np.round(arr / scale), -127, 127).astype(np.int8)
    vals = ", ".join(f"{int(v):4d}" for v in q_arr)
    return (
        f"#define {scale_macro} ({scale:.7f}f)\n"
        f"static const int8_t {name}[{len(arr)}] = {{ {vals} }};\n"
    )


def export_c_weights_header(model_dict, real_count=0):
    os.makedirs(os.path.dirname(WEIGHTS_HEADER_PATH), exist_ok=True)
    header = [
        "/* ================================================================",
        " * API/ai_weights.h",
        " * 瑞萨 RA6M5 片上 PINN-CfC 物理内嵌液态神经网络预训练核心权重库",
        " * 存储格式: Flash 对称张量级 INT8 量化存储 + 上电 SRAM FP32 高速反量化张量池",
        f" * 数据集规模: 总计 {model_dict['total_samples']} 条 (独立验证集 {model_dict.get('val_samples', 0)} 条, 含真机样本 {real_count} 条)",
        f" * INT8 量化后独立验证集指标: 准确率={model_dict['accuracy']:.2f}% | 跌倒召回率(Recall)={model_dict.get('fall_recall', 0.0):.2f}% | 误报率(FPR)={model_dict.get('fall_fpr', 0.0):.2f}%",
        " * 类别定义: 0=正常, 1=原地弯腰/下蹲/坐下, 2=失衡跌倒报警",
        " * ================================================================ */",
        "#ifndef AI_WEIGHTS_H_",
        "#define AI_WEIGHTS_H_\n",
        "#include <stdint.h>\n",
        f"#define AI_FEAT_DIM     {INPUT_DIM}u",
        f"#define AI_HIDDEN_DIM   {HIDDEN_DIM}u",
        f"#define AI_NUM_CLASSES  {NUM_CLASSES}u",
        "#define AI_MODEL_READY  1u\n",
        quantize_int8_2d("g_ai_w_tau_q8", "AI_SCALE_W_TAU", model_dict["W_tau"]),
        quantize_int8_1d("g_ai_b_tau_q8", "AI_SCALE_B_TAU", model_dict["b_tau"]),
        quantize_int8_2d("g_ai_w_g_q8", "AI_SCALE_W_G", model_dict["W_g"]),
        quantize_int8_1d("g_ai_b_g_q8", "AI_SCALE_B_G", model_dict["b_g"]),
        quantize_int8_2d("g_ai_w_pinn_q8", "AI_SCALE_W_PINN", model_dict["W_pinn"]),
        quantize_int8_2d("g_ai_w_out_q8", "AI_SCALE_W_OUT", model_dict["W_out"]),
        quantize_int8_1d("g_ai_b_out_q8", "AI_SCALE_B_OUT", model_dict["b_out"]),
        quantize_int8_2d("g_ai_prototypes_q8", "AI_SCALE_PROTOS", model_dict["prototypes"]),
        "#endif /* AI_WEIGHTS_H_ */\n"
    ]
    with open(WEIGHTS_HEADER_PATH, "w", encoding="utf-8") as f:
        f.write("\n".join(header))
    print(
        f"[成功] 已导出 RA6M5 INT8-Flash + FP32-SRAM 权重头文件: {WEIGHTS_HEADER_PATH} "
        f"(INT8验证集准确率: {model_dict['accuracy']:.2f}%, 跌倒召回率: {model_dict.get('fall_recall', 0.0):.2f}%, 误报率: {model_dict.get('fall_fpr', 0.0):.2f}%)"
    )


def launch_gui():
    import tkinter as tk
    from tkinter import messagebox

    root = tk.Tk()
    root.title("MMCare 瑞萨 RA6M5 · 分步倒计时引导式 AI 姿态采集与训练中心")
    root.geometry("1060x680")
    root.configure(bg="#062224")

    # 统计当前已有样本数
    def get_saved_counts():
        counts = [0, 0, 0]
        if os.path.exists(DATASET_PATH):
            try:
                d = np.load(DATASET_PATH)
                y_arr = d["y"]
                for c in range(3):
                    counts[c] = int(np.sum(y_arr == c))
            except Exception:
                pass
        return counts

    # 顶部标题栏
    top_frame = tk.Frame(root, bg="#0a2e32", pady=10, padx=20)
    top_frame.pack(fill="x")
    tk.Label(
        top_frame,
        text="🧠 MMCare 瑞萨 RA6M5 端侧 PINN-CfC 液态神经网络 · 分步采集与训练中心",
        font=("Microsoft YaHei", 13, "bold"),
        fg="#2dd4bf", bg="#0a2e32"
    ).pack(side="left")

    live_status_lbl = tk.Label(
        top_frame,
        text="📡 雷达实时状态: 连接中...",
        font=("Microsoft YaHei", 10, "bold"),
        fg="#a7f3d0", bg="#0a2e32"
    )
    live_status_lbl.pack(side="right")

    # 中部超大号倒计时与动作引导面板（站在4米外也能看清）
    center_frame = tk.Frame(root, bg="#0c363a", bd=2, relief="ridge", padx=25, pady=16)
    center_frame.pack(fill="both", expand=True, padx=24, pady=12)

    stage_title_lbl = tk.Label(
        center_frame,
        text="准备就绪：请点击下方任意步骤按钮开始采集",
        font=("Microsoft YaHei", 19, "bold"),
        fg="#f0fdfa", bg="#0c363a"
    )
    stage_title_lbl.pack(pady=(4, 6))

    stage_tip_lbl = tk.Label(
        center_frame,
        text="💡 每次点击按钮后会先给您 3 秒准备时间（伴随“滴-滴-滴”提示音），让您从容走到雷达垫子前！",
        font=("Microsoft YaHei", 12),
        fg="#99f6e4", bg="#0c363a"
    )
    stage_tip_lbl.pack(pady=(0, 8))

    countdown_lbl = tk.Label(
        center_frame,
        text="READY",
        font=("Arial", 72, "bold"),
        fg="#2dd4bf", bg="#0c363a"
    )
    countdown_lbl.pack(pady=6)

    stats_bar = tk.Frame(center_frame, bg="#0c363a")
    stats_bar.pack(pady=4)

    sample_stats_lbl = tk.Label(
        stats_bar,
        text="",
        font=("Microsoft YaHei", 11, "bold"),
        fg="#fde68a", bg="#0c363a"
    )
    sample_stats_lbl.pack(side="left", padx=(0, 12))

    def refresh_stats_label():
        c = get_saved_counts()
        total = sum(c)
        sample_stats_lbl.config(
            text=f"📦 本地累计真机样本库：共 {total} 组  "
                 f"（①正常: {c[0]} | ②弯腰/下蹲: {c[1]} | ③真实跌倒: {c[2]}）"
        )

    refresh_stats_label()

    # 底部三步采集按钮 + 一键训练按钮区
    btn_frame = tk.Frame(root, bg="#062224", padx=24, pady=10)
    btn_frame.pack(fill="x")

    is_busy = {"val": False, "cancel": False}

    def cancel_current_task():
        if is_busy["val"]:
            is_busy["cancel"] = True

    def clear_saved_samples():
        if is_busy["val"]:
            messagebox.showinfo("提示", "请先停止当前采集任务！")
            return
        if os.path.exists(DATASET_PATH):
            if messagebox.askyesno("确认清空", "确定要备份并清空本地已录制的真机样本重新开始吗？\n（将自动备份到 datasets/backup 目录）"):
                import shutil, datetime
                ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                bk_dir = os.path.join("datasets", "backup")
                os.makedirs(bk_dir, exist_ok=True)
                bk_npz = os.path.join(bk_dir, f"mmcare_real_dataset_auto_backup_{ts_str}.npz")
                shutil.copy2(DATASET_PATH, bk_npz)
                os.remove(DATASET_PATH)
                refresh_stats_label()
                stage_title_lbl.config(text=f"🗑️ 已自动备份至 {os.path.basename(bk_npz)} 并清空真机样本库！", fg="#2dd4bf")
        else:
            messagebox.showinfo("提示", "当前真机样本库已经是空的（0 组），随时可以开始录制！")

    tk.Button(
        stats_bar, text="⏹️ 停止当前倒计时", font=("Microsoft YaHei", 9, "bold"),
        bg="#475569", fg="#ffffff", padx=8, pady=2, cursor="hand2",
        command=cancel_current_task
    ).pack(side="left", padx=4)

    tk.Button(
        stats_bar, text="🗑️ 清空真机样本", font=("Microsoft YaHei", 9, "bold"),
        bg="#7f1d1d", fg="#fecaca", padx=8, pady=2, cursor="hand2",
        command=clear_saved_samples
    ).pack(side="left", padx=4)

    def save_new_samples(X_add, y_add):
        if len(X_add) == 0:
            return
        os.makedirs("datasets", exist_ok=True)
        X_arr = np.array(X_add, dtype=np.float32)
        y_arr = np.array(y_add, dtype=np.int64)
        if os.path.exists(DATASET_PATH):
            old = np.load(DATASET_PATH)
            old_X, old_y = old["X"], old["y"]
            if "groups" in old:
                old_g = old["groups"]
                next_g = int(np.max(old_g)) + 1 if len(old_g) > 0 else 0
            else:
                old_g = np.arange(len(old_y), dtype=np.int64)
                next_g = len(old_y)
            g_arr = np.full(len(y_arr), next_g, dtype=np.int64)
            X_all = np.concatenate([old_X, X_arr], axis=0)
            y_all = np.concatenate([old_y, y_arr], axis=0)
            g_all = np.concatenate([old_g, g_arr], axis=0)
        else:
            g_all = np.zeros(len(y_arr), dtype=np.int64)
            X_all, y_all = X_arr, y_arr
        np.savez(DATASET_PATH, X=X_all, y=y_all, groups=g_all)
        root.after(0, refresh_stats_label)

    def start_collection_task(cls_id, title_str, tip_str, duration_sec, color_hex):
        if is_busy["val"]:
            messagebox.showinfo("提示", "当前正在采集或训练中，如需中断请点击【⏹️ 停止当前倒计时】！")
            return
        is_busy["val"] = True
        is_busy["cancel"] = False

        def worker():
            try:
                # 3 秒准备倒计时（让人走到雷达前）
                for prep in [3, 2, 1]:
                    if is_busy["cancel"]:
                        break
                    root.after(0, lambda p=prep: (
                        stage_title_lbl.config(text=f"⏳ 准备开始：{title_str}", fg="#fbbf24"),
                        stage_tip_lbl.config(text=f"请立即走到雷达监测区域！{p} 秒后正式开始记录...（可随时点“停止当前倒计时”取消）"),
                        countdown_lbl.config(text=f"准备 {p}", fg="#fbbf24")
                    ))
                    play_beep(660, 180)
                    time.sleep(1.0)

                if is_busy["cancel"]:
                    root.after(0, lambda: (
                        stage_title_lbl.config(text="⏹️ 已取消本次采集（未保存任何数据）", fg="#fbbf24"),
                        countdown_lbl.config(text="READY", fg="#2dd4bf")
                    ))
                    return

                # 正式开始长音提示
                play_beep(1200, 450)
                t0 = time.time()
                traj = []
                last_valid_dist = 1.6
                has_active_reading = False

                while True:
                    if is_busy["cancel"]:
                        root.after(0, lambda: (
                            stage_title_lbl.config(text="⏹️ 已中途取消本次采集（未保存任何数据）", fg="#fbbf24"),
                            countdown_lbl.config(text="READY", fg="#2dd4bf")
                        ))
                        return
                    elapsed = time.time() - t0
                    remain = max(0, int(math.ceil(duration_sec - elapsed)))
                    root.after(0, lambda r=remain: (
                        stage_title_lbl.config(text=f"🔴 正在录制：{title_str}", fg=color_hex),
                        stage_tip_lbl.config(text=tip_str),
                        countdown_lbl.config(text=f"{r:02d} 秒", fg=color_hex)
                    ))
                    if elapsed >= duration_sec:
                        break

                    cur = fetch_latest_reading()
                    if cur and cur.get("online"):
                        dist_m = float(cur.get("distance") or 0.0)
                        speed_cms = float(cur.get("motion_speed") or 0.0)
                        h_m = float(cur.get("h_m") or 0.0)
                        v_drop_ms = float(cur.get("v_drop_ms") or 0.0)
                        v60_ms = float(cur.get("v60_ms") or 0.0)

                        # 空间 ROI 围栏与死区抑制:
                        # 1. 过滤 4.2 米以外门外或走道路人干扰跳变
                        if dist_m > 4.2:
                            dist_m = last_valid_dist

                        # 2. 静止死区滤波: 消除 < 12cm/s 的天线微弱热噪声抖动
                        clean_speed_cms = speed_cms if abs(speed_cms) >= 12.0 else 0.0
                        clean_v_drop_ms = v_drop_ms if abs(v_drop_ms) >= 0.10 else 0.0

                        if (0.35 <= dist_m <= 4.2) or h_m > 0.05:
                            last_valid_dist = dist_m
                            has_active_reading = True
                        elif abs(clean_speed_cms) > 0.5 or abs(clean_v_drop_ms) > 0.1:
                            has_active_reading = True

                        if has_active_reading:
                            # 若摔倒后躺在垫子上绝对静止导致距离暂时归零，保持触地点距离并记录 0 速度静止帧（绝不丢弃！）
                            use_dist = dist_m if dist_m > 0.05 else last_valid_dist
                            traj.append((time.time() - t0, use_dist, clean_speed_cms / 100.0, h_m, clean_v_drop_ms, v60_ms))
                    time.sleep(0.15)

                # 防呆保护：如果主板未上电/离线，或者全程没有任何有效运动数据，自动丢弃！
                if len(traj) < 6:
                    play_beep(440, 300)
                    root.after(0, lambda: (
                        stage_title_lbl.config(text="⚠️ 主板未上电或未检测到有效人体运动，本次空数据已自动丢弃！", fg="#fbbf24"),
                        stage_tip_lbl.config(text="请放心：空数据绝不会写入样本库！等主板挂墙上电后再采集即可。"),
                        countdown_lbl.config(text="SKIP", fg="#fbbf24")
                    ))
                    return

                # 完成双响提示音（此时动作已完成，人员可正常起身）
                play_beep(1000, 180)
                time.sleep(0.1)
                play_beep(1400, 300)

                # 针对 6 秒摔倒采集：自动补偿 4G HTTP 约 2.0 秒传输延迟，把主板刚发出的“躺地静止尾包”完整收齐
                if cls_id == 2:
                    root.after(0, lambda: (
                        stage_title_lbl.config(text="📡 正在同步 4G 云端躺地静止尾包 (2秒)...", fg="#38bdf8"),
                        countdown_lbl.config(text="SYNC", fg="#38bdf8")
                    ))
                    t_sync_end = time.time() + 2.0
                    while time.time() < t_sync_end:
                        cur = fetch_latest_reading()
                        if cur and cur.get("online"):
                            dist_m = float(cur.get("distance") or 0.0)
                            speed_cms = float(cur.get("motion_speed") or 0.0)
                            h_m = float(cur.get("h_m") or 0.0)
                            v_drop_ms = float(cur.get("v_drop_ms") or 0.0)
                            v60_ms = float(cur.get("v60_ms") or 0.0)

                            if dist_m > 4.2:
                                dist_m = last_valid_dist
                            clean_speed_cms = speed_cms if abs(speed_cms) >= 12.0 else 0.0
                            clean_v_drop_ms = v_drop_ms if abs(v_drop_ms) >= 0.10 else 0.0

                            if dist_m > 0.05:
                                last_valid_dist = dist_m
                            use_dist = dist_m if dist_m > 0.05 else last_valid_dist
                            traj.append((time.time() - t0, use_dist, clean_speed_cms / 100.0, h_m, clean_v_drop_ms, v60_ms))
                        time.sleep(0.20)

                # 将 4G 云端轨迹去阶跃并线性插值重采样为片上 10Hz (dt=0.10s) 连续轨迹
                traj_10hz = dealias_and_interpolate_traj(traj, is_fall=(cls_id == 2))

                X_add, y_add = [], []
                if cls_id == 2:
                    # 摔倒正样本：仅保存真实原始特征（不做预保存加噪复制，防止训练/验证集泄漏）
                    if len(traj_10hz) >= 6:
                        X_add.append(extract_16d_features(traj_10hz[-16:]))
                        y_add.append(2)
                else:
                    # 日常动作负样本：按 16 点窗口切分，统一绑定同一个采集批次 group_id
                    win_size = 16
                    step_size = max(6, len(traj_10hz) // 12)
                    for i in range(0, max(1, len(traj_10hz) - win_size + 1), step_size):
                        sub = traj_10hz[i:i + win_size]
                        if len(sub) >= 6:
                            X_add.append(extract_16d_features(sub))
                            y_add.append(cls_id)

                save_new_samples(X_add, y_add)
                root.after(0, lambda n=len(y_add): (
                    stage_title_lbl.config(text=f"✅ {title_str} 采集完成！（新增 {n} 组原始特征样本）", fg="#34d399"),
                    stage_tip_lbl.config(text="您可以继续采集下一步，或点击最下方按钮【一键训练并更新 C 权重】！"),
                    countdown_lbl.config(text="DONE", fg="#34d399")
                ))
            finally:
                is_busy["val"] = False
                is_busy["cancel"] = False

        threading.Thread(target=worker, daemon=True).start()

    def run_training_task():
        if is_busy["val"]:
            return
        is_busy["val"] = True

        def train_worker():
            try:
                if not os.path.exists(DATASET_PATH):
                    play_beep(440, 300)
                    root.after(0, lambda: (
                        stage_title_lbl.config(text="⚠️ 真机样本库当前为空（0 组）！请先开始真实数据采集", fg="#fbbf24"),
                        stage_tip_lbl.config(text="系统已彻底杜绝并关闭任何模拟/假数据生成！请点击上方步骤按钮录制真实动作后，再点击训练。"),
                        countdown_lbl.config(text="EMPTY", fg="#fbbf24")
                    ))
                    return

                d = np.load(DATASET_PATH)
                X_real, y_real = d["X"], d["y"]
                g_real = d["groups"] if "groups" in d else np.arange(len(y_real), dtype=np.int64)
                real_cnt = len(y_real)

                if real_cnt == 0:
                    play_beep(440, 300)
                    root.after(0, lambda: (
                        stage_title_lbl.config(text="⚠️ 真机样本库为 0 条！请先录制真实动作", fg="#fbbf24"),
                        stage_tip_lbl.config(text="请分别采集【①正常】、【②弯腰&下蹲】、【③真实摔倒】动作，模型将 100% 完全用真实数据训练。"),
                        countdown_lbl.config(text="EMPTY", fg="#fbbf24")
                    ))
                    return

                # 检查是否三类都有样本
                c_counts = [int(np.sum(y_real == c)) for c in range(NUM_CLASSES)]
                missing_classes = [c for c in range(NUM_CLASSES) if c_counts[c] == 0]
                if missing_classes:
                    names = ["①正常", "②弯腰&下蹲", "③真实摔倒"]
                    missing_names = "、".join([names[c] for c in missing_classes])
                    play_beep(440, 300)
                    root.after(0, lambda mn=missing_names: (
                        stage_title_lbl.config(text=f"⚠️ 缺少类别样本: {mn}", fg="#fbbf24"),
                        stage_tip_lbl.config(text=f"纯真机 AI 决策要求必须具备所有类别的真实数据，请先录制缺失的 {mn} 动作！"),
                        countdown_lbl.config(text="NEED", fg="#fbbf24")
                    ))
                    return

                root.after(0, lambda: (
                    stage_title_lbl.config(text="🚀 正在用您的 100% 真实传感器数据训练 PINN-CfC 神经网络...", fg="#2dd4bf"),
                    countdown_lbl.config(text="TRAIN...", fg="#2dd4bf")
                ))

                # 纯真机数据训练：按采集批次 group 严格隔离划分真机训练集与独立验证集
                X_train, y_train, X_val, y_val = split_train_val_by_group(X_real, y_real, g_real, val_ratio=0.20)

                # 仅对真机训练集做特征流形插值均衡，绝不污染验证集，绝不引入任何先验模拟假数据
                X_tr_list, y_tr_list = list(X_train), list(y_train)
                max_cls_n = max([int(np.sum(y_train == c)) for c in range(NUM_CLASSES)])
                target_per_cls = max(60, max_cls_n)

                for cls_idx in range(NUM_CLASSES):
                    cls_samples = X_train[y_train == cls_idx]
                    n_curr = len(cls_samples)
                    if n_curr > 0 and n_curr < target_per_cls:
                        needed = target_per_cls - n_curr
                        for i in range(needed):
                            base_a = cls_samples[i % n_curr]
                            base_b = cls_samples[np.random.randint(0, n_curr)]
                            alpha = np.random.uniform(0.75, 1.0)
                            mix = alpha * base_a + (1.0 - alpha) * base_b
                            noise = np.random.normal(0.0, 0.008, size=INPUT_DIM).astype(np.float32)
                            X_tr_list.append(np.clip(mix + noise, -1.5, 3.5))
                            y_tr_list.append(cls_idx)

                X_train = np.array(X_tr_list, dtype=np.float32)
                y_train = np.array(y_tr_list, dtype=np.int64)

                model = train_pinn_cfc_network(X_train, y_train, X_val=X_val, y_val=y_val, epochs=420, lr=0.045)
                export_c_weights_header(model, real_count=real_cnt)
                play_beep(1300, 350)
                root.after(0, lambda m=model, rc=real_cnt: (
                    stage_title_lbl.config(
                        text=(
                            f"🎉 纯真机 AI 模型训练完成！验证集准确率 {m['accuracy']:.2f}% | "
                            f"跌倒召回率 {m['fall_recall']:.1f}% | 误报率 {m['fall_fpr']:.1f}%"
                        ),
                        fg="#34d399"
                    ),
                    stage_tip_lbl.config(
                        text=f"已用您的 {rc} 组 100% 真实传感器样本生成 API/ai_weights.h！完全去除人工阈值，全权由 AI 决策！"
                    ),
                    countdown_lbl.config(text=f"{m['accuracy']:.1f}%", fg="#34d399")
                ))
            finally:
                is_busy["val"] = False

        threading.Thread(target=train_worker, daemon=True).start()

    # 三个分步采集按钮
    grid_btns = tk.Frame(btn_frame, bg="#062224")
    grid_btns.pack(fill="x", pady=(0, 10))
    for col in range(3):
        grid_btns.columnconfigure(col, weight=1)

    b1 = tk.Button(
        grid_btns, text="① 采集【正常】\n(倒计时 25 秒)",
        font=("Microsoft YaHei", 12, "bold"), bg="#0d9488", fg="#ffffff",
        activebackground="#14b8a6", activeforeground="#ffffff", pady=10, cursor="hand2",
        command=lambda: start_collection_task(
            0, "第 1 步：正常 (Class 0)",
            "请在雷达探测区域内【正常活动：正常站立、微步走动、擦拭或微动】", 25, "#2dd4bf"
        )
    )
    b1.grid(row=0, column=0, padx=6, sticky="ew")

    b2 = tk.Button(
        grid_btns, text="② 采集【弯腰 & 下蹲】\n(倒计时 25 秒)",
        font=("Microsoft YaHei", 12, "bold"), bg="#d97706", fg="#ffffff",
        activebackground="#f59e0b", activeforeground="#ffffff", pady=10, cursor="hand2",
        command=lambda: start_collection_task(
            1, "第 2 步：弯腰 & 下蹲 (Class 1)",
            "请在雷达前做【原地弯腰捡香皂/洗脚、慢慢蹲下停顿2秒再站起来】动作", 25, "#fbbf24"
        )
    )
    b2.grid(row=0, column=1, padx=6, sticky="ew")

    b3 = tk.Button(
        grid_btns, text="③ 录制 1 次【真实摔倒】\n(倒计时 6 秒/次)",
        font=("Microsoft YaHei", 12, "bold"), bg="#dc2626", fg="#ffffff",
        activebackground="#ef4444", activeforeground="#ffffff", pady=10, cursor="hand2",
        command=lambda: start_collection_task(
            2, "第 3 步：真实摔倒并躺地静止 (Class 2)",
            "请在软垫上立即【摔倒并保持躺地不动 3~4 秒】！", 6, "#f87171"
        )
    )
    b3.grid(row=0, column=2, padx=6, sticky="ew")

    # 一键训练大按钮
    train_btn = tk.Button(
        btn_frame,
        text="🚀 一键训练 PINN-CfC 液态神经网络 并自动更新 C 权重头文件 (API/ai_weights.h)",
        font=("Microsoft YaHei", 14, "bold"), bg="#10b981", fg="#042f2e",
        activebackground="#34d399", pady=12, cursor="hand2",
        command=run_training_task
    )
    train_btn.pack(fill="x", padx=6, pady=(4, 6))

    # 后台实时状态刷新线程
    def poll_live():
        while True:
            cur = fetch_latest_reading()
            if cur:
                d_val = cur.get("distance", 0.0)
                v_val = cur.get("motion_speed", 0.0)
                h_val = cur.get("h_m", 0.0)
                v_drop = cur.get("v_drop_ms", 0.0)
                online_str = "在线" if cur.get("online") else "离线"
                root.after(0, lambda d=d_val, v=v_val, h=h_val, vd=v_drop, o=online_str: live_status_lbl.config(
                    text=f"📡 4G:{o} | 距离:{d:.2f}m | 速度:{v:.0f}cm/s | LD6002C高度H:{h:.2f}m | 坠速:{vd:.2f}m/s",
                    fg="#34d399"
                ))
            else:
                root.after(0, lambda: live_status_lbl.config(
                    text="📡 云端连接等待中...", fg="#f87171"
                ))
            time.sleep(1.0)

    threading.Thread(target=poll_live, daemon=True).start()
    root.mainloop()


def main():
    parser = argparse.ArgumentParser(description="MMCare RA6M5 分步引导式多姿态采集与 PINN-CfC 训练工具")
    parser.add_argument("--init-baseline", action="store_true",
                        help="仅使用纯净物理动力学模型生成第一版基线权重 API/ai_weights.h")
    parser.add_argument("--cli", action="store_true",
                        help="使用命令行交互模式")
    args = parser.parse_args()

    if args.init_baseline:
        print("🔧 正在生成物理动力学 PINN-CfC 基线权重（含按组隔离的独立验证集与 INT8 量化后评估）...")
        X_prior, y_prior, g_prior = generate_physics_prior_dataset(samples_per_class=250)
        X_train, y_train, X_val, y_val = split_train_val_by_group(X_prior, y_prior, g_prior, val_ratio=0.20)
        real_cnt = 0
        if os.path.exists(DATASET_PATH):
            d = np.load(DATASET_PATH)
            X_r, y_r = d["X"], d["y"]
            g_r = d["groups"] if "groups" in d else np.arange(len(y_r), dtype=np.int64)
            X_r_tr, y_r_tr, X_r_val, y_r_val = split_train_val_by_group(X_r, y_r, g_r, val_ratio=0.20)
            if len(y_r_tr) > 0:
                X_train = np.concatenate([X_train, np.repeat(X_r_tr, 3, axis=0)], axis=0)
                y_train = np.concatenate([y_train, np.repeat(y_r_tr, 3, axis=0)], axis=0)
            if len(y_r_val) > 0:
                X_val = np.concatenate([X_val, X_r_val], axis=0)
                y_val = np.concatenate([y_val, y_r_val], axis=0)
            real_cnt = len(y_r)
        model = train_pinn_cfc_network(X_train, y_train, X_val=X_val, y_val=y_val, epochs=380, lr=0.045)
        export_c_weights_header(model, real_count=real_cnt)
    else:
        launch_gui()


if __name__ == "__main__":
    main()
