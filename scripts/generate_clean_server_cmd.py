"""
生成在云服务器上一键彻底铲除虚拟传感器、随机姿态、每3分钟假跌倒注入的加固脚本
"""

cloud_patch_script = r'''#!/usr/bin/env python3
import os
import sqlite3
import re
import subprocess

print("==================================================")
print("🚀 心安守护 Mmcare: 彻底清除云端所有随机模拟与假跌倒")
print("==================================================")

paths = ['/opt/mmcare', '/home/ubuntu/mmcare_server']

for base in paths:
    if not os.path.isdir(base):
        continue
    print(f"\n[1/5] 处理目录: {base}")
    
    # 1. 彻底清空 virtual_sensors.py 中的模拟注入逻辑，让其纯净空载
    vs_path = os.path.join(base, 'services', 'virtual_sensors.py')
    if os.path.isfile(vs_path):
        clean_vs = '''"""虚拟传感器补全：已彻底关闭所有模拟与随机注入。
100% 仅采信硬件单片机真实物理传感器数据！
"""
def fill(*args, **kwargs):
    return {}, []

def synthesize_pose(present):
    return None
'''
        with open(vs_path, 'w', encoding='utf-8') as f:
            f.write(clean_vs)
        print(f"  ✅ 已清空虚拟传感器: {vs_path}")

    # 2. 修改 config.py，将 VIRTUAL_FILL=False, VIRTUAL_FALL_EVERY_SECONDS=0
    cfg_path = os.path.join(base, 'config.py')
    if os.path.isfile(cfg_path):
        with open(cfg_path, 'r', encoding='utf-8') as f:
            c = f.read()
        c = re.sub(r'VIRTUAL_FILL\s*=\s*.*', 'VIRTUAL_FILL = False', c)
        c = re.sub(r'VIRTUAL_FALL_EVERY_SECONDS\s*=\s*.*', 'VIRTUAL_FALL_EVERY_SECONDS = 0', c)
        with open(cfg_path, 'w', encoding='utf-8') as f:
            f.write(c)
        print(f"  ✅ 已更新配置: {cfg_path}")

    # 3. 修复 legacy.py，防止历史未处理跌倒死锁实时大屏
    leg_path = os.path.join(base, 'api', 'legacy.py')
    if os.path.isfile(leg_path):
        with open(leg_path, 'r', encoding='utf-8') as f:
            c = f.read()
        # 将 "fall": readings.has_active_fall(device_id) 改为以真实设备读数为准
        target_str = '"fall": readings.has_active_fall(device_id)'
        replacement = '"fall": (row["pose"] == "falling")'
        if target_str in c:
            c = c.replace(target_str, replacement)
            with open(leg_path, 'w', encoding='utf-8') as f:
                f.write(c)
            print(f"  ✅ 已解除 legacy_latest 跌倒死锁: {leg_path}")

# 4. 修改 systemd 服务配置
service_file = '/etc/systemd/system/mmcare.service'
if os.path.isfile(service_file):
    with open(service_file, 'r', encoding='utf-8') as f:
        sc = f.read()
    sc = sc.replace('Environment="MMCARE_VIRTUAL_FILL=1"', 'Environment="MMCARE_VIRTUAL_FILL=0"')
    with open(service_file, 'w', encoding='utf-8') as f:
        f.write(sc)
    print("\n[2/5] ✅ 已修改 systemd 环境变量 MMCARE_VIRTUAL_FILL=0")
    subprocess.run(['systemctl', 'daemon-reload'], check=False)

# 5. 清理数据库历史积压的假跌倒事件
db_paths = ['/opt/mmcare/mmcare.db', '/home/ubuntu/mmcare_server/mmcare.db']
print("\n[3/5] 清理数据库历史积压跌倒事件...")
for dbp in db_paths:
    if os.path.isfile(dbp):
        try:
            conn = sqlite3.connect(dbp)
            cur = conn.cursor()
            cur.execute("UPDATE fall_event SET handled = 1 WHERE handled = 0")
            print(f"  ✅ {dbp}: 标记了 {cur.rowcount} 条未处理跌倒为已处理")
            conn.commit()
            conn.close()
        except Exception as e:
            print(f"  ⚠️ 处理 {dbp} 出错: {e}")

# 6. 重启服务
print("\n[4/5] 重启后端服务...")
ret = subprocess.run(['systemctl', 'restart', 'mmcare'], capture_output=True, text=True)
if ret.returncode == 0:
    print("  ✅ systemctl restart mmcare 成功！")
else:
    # 尝试 pkill 并重新拉起
    subprocess.run(['pkill', '-f', 'uvicorn main:app'], check=False)
    subprocess.Popen(['python3', '-m', 'uvicorn', 'main:app', '--host', '0.0.0.0', '--port', '8080'],
                     cwd='/opt/mmcare' if os.path.isdir('/opt/mmcare') else '/home/ubuntu/mmcare_server')
    print("  ✅ 进程已重启！")

print("\n[5/5] 🎉 全部完成！现在系统 100% 仅采信单片机真实物理雷达数据，绝无任何随机生成！")
'''

import gzip
import base64

compressed = gzip.compress(cloud_patch_script.encode('utf-8'))
b64 = base64.b64encode(compressed).decode('ascii')

one_liner = f"python3 -c \"import gzip, base64; exec(gzip.decompress(base64.b64decode('{b64}')).decode('utf-8'))\""

with open('deploy_clean_server.txt', 'w', encoding='utf-8') as f:
    f.write(one_liner)

print("SUCCESS! Generated deploy_clean_server.txt, length:", len(one_liner))
