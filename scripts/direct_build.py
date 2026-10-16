import subprocess
import os

armclang = r'D:\MDK536\Keil\ARM\ARMCLANG\bin\armclang.exe'
armlink = r'D:\MDK536\Keil\ARM\ARMCLANG\bin\armlink.exe'

inc_dirs = [
    './API', './RTE/_Target_1', './ra/arm/CMSIS_5/CMSIS/Core/Include',
    './ra/fsp/inc', './ra/fsp/inc/api', './ra/fsp/inc/instances',
    './ra_cfg/fsp_cfg', './ra_cfg/fsp_cfg/bsp', './ra_gen', '.'
]

inc_flags = []
for d in inc_dirs:
    inc_flags.extend(['-I', d])

base_cflags = [
    armclang, '-xc', '-std=c99', '--target=arm-arm-none-eabi',
    '-mcpu=cortex-m33', '-mfpu=fpv5-sp-d16', '-mfloat-abi=hard',
    '-c', '-fno-rtti', '-funsigned-char', '-D__EVAL', '-D__MICROLIB',
    '-gdwarf-4', '-Oz', '-ffunction-sections',
    '-Wno-license-management',
    '-D__UVISION_VERSION=543', '-D_RTE_'
] + inc_flags

for src, obj in [('src/hal_entry.c', 'Objects/hal_entry.o'),
                 ('API/radar_uart.c', 'Objects/radar_uart.o'),
                 ('API/ml307r.c', 'Objects/ml307r.o'),
                 ('API/ai_fall_engine.c', 'Objects/ai_fall_engine.o')]:
    cmd = base_cflags + ['-o', obj, src]
    print(f"Compiling {src} -> {obj}...")
    res = subprocess.run(cmd, capture_output=True, encoding='utf-8', errors='replace')
    if res.returncode != 0:
        print("Compile Error:\n", res.stderr)
        exit(1)
    print("Done.")

print("Linking...")
if os.path.exists('Objects/Mmcare.axf'):
    try:
        os.remove('Objects/Mmcare.axf')
    except Exception:
        pass
link_cmd = [armlink, '--via=Objects/Mmcare.lnp']
res = subprocess.run(link_cmd, capture_output=True, encoding='utf-8', errors='replace')
print("Link output:")
print(res.stdout)
print(res.stderr)
if res.returncode == 0:
    print("\nSUCCESS! AxF built successfully!")
    fromelf = r'D:\MDK536\Keil\ARM\ARMCLANG\bin\fromelf.exe'
    hex_cmd = [fromelf, '--i32combined', '--output=Objects/Mmcare_latest.hex', 'Objects/Mmcare.axf']
    h_res = subprocess.run(hex_cmd, capture_output=True, text=True)
    if h_res.returncode == 0:
        import shutil
        try:
            shutil.copyfile('Objects/Mmcare_latest.hex', 'Objects/Mmcare.hex')
            print("SUCCESS! Generated Objects/Mmcare.hex ready to flash!")
        except Exception:
            print("SUCCESS! Objects/Mmcare.hex is locked by Keil/Flasher, generated Objects/Mmcare_latest.hex successfully!")
    else:
        print("fromelf warning:", h_res.stderr)
else:
    print(f"\nLink failed with code {res.returncode}")
