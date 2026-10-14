import os
import shutil
import csv
import numpy as np

dataset_path = os.path.join("datasets", "mmcare_real_dataset.npz")
backup_dir_1 = os.path.join("datasets", "backup")
backup_dir_2 = os.path.join("backups", "Dataset_Backup_20261014_Pre_DualRadar")

os.makedirs(backup_dir_1, exist_ok=True)
os.makedirs(backup_dir_2, exist_ok=True)

if os.path.exists(dataset_path):
    target_npz_1 = os.path.join(backup_dir_1, "mmcare_real_dataset_backup_20261014_pre_dualradar_70samples.npz")
    target_csv_1 = os.path.join(backup_dir_1, "mmcare_real_dataset_backup_20261014_pre_dualradar_70samples.csv")
    shutil.copy2(dataset_path, target_npz_1)
    
    # 读取并转存为 CSV 明细 (使用 with 确保立即释放文件锁)
    with np.load(dataset_path) as d:
        X = np.array(d["X"])
        y = np.array(d["y"])
        groups = np.array(d["groups"]) if "groups" in d else np.zeros(len(y), dtype=int)
    
    feat_names = [
        "mean_r", "span_r", "mean_v", "max_v",
        "mean_v_early", "mean_v_late", "max_acc", "max_jerk",
        "zero_crossings", "collapse_ratio", "post_stillness", "pinn_impact",
        "stft_b0", "stft_b1", "stft_b2", "stft_b3", "label_class", "event_group"
    ]
    
    with open(target_csv_1, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(feat_names)
        for i in range(len(y)):
            row = list(X[i]) + [int(y[i]), int(groups[i])]
            writer.writerow(row)

    # 2. 备份到工程顶级 backups 目录
    target_npz_2 = os.path.join(backup_dir_2, "mmcare_real_dataset_backup_20261014_pre_dualradar_70samples.npz")
    target_csv_2 = os.path.join(backup_dir_2, "mmcare_real_dataset_backup_20261014_pre_dualradar_70samples.csv")
    shutil.copy2(target_npz_1, target_npz_2)
    shutil.copy2(target_csv_1, target_csv_2)

    print(f"[OK] 现有 70 条样本已成功备份到:\n  1) {target_npz_1}\n  2) {target_csv_1}\n  3) {target_npz_2}")
    
    # 3. 将生产数据集重置为全新状态 (移除原文件)
    try:
        os.remove(dataset_path)
        print(f"[OK] 生产数据集 {dataset_path} 已清空重置为全新状态！")
    except Exception as e:
        print(f"[WARN] 移除原文件异常: {e}")
else:
    print(f"[INFO] 生产数据集 {dataset_path} 当前不存在，已是全新状态。")
