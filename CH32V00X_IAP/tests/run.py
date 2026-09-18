"""Bootloader 全套主机端验证：协议审计 -> 整机仿真 -> 用错场景仿真 -> 固件编译。

用法：
    python tests/run.py            全部跑一遍
    python tests/run.py quick      只跑协议审计和仿真(不用工具链)

仿真里 Boot 和 APP 跑的都是工程里的源码本体，只把 Flash、串口、计时换成模型；
升级用的固件取 CH32V00X_APP_ADC/obj 下 MounRiver 编译出来的 hex。
"""
import pathlib
import subprocess
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = pathlib.Path(tempfile.gettempdir()) / 'boot_tests'
CC = ['gcc', '-std=c99', '-O2', '-Wall', '-Wextra', '-Wno-unused-parameter']


def boot_audit():
    OUT.mkdir(parents=True, exist_ok=True)
    exe = OUT / 'boot_audit.exe'
    subprocess.run(CC + ['-I' + str(HERE / 'stubs'), '-I' + str(ROOT / 'System'),
                         str(HERE / 'boot_audit.c'), '-o', str(exe)], check=True)
    result = subprocess.run([str(exe)], capture_output=True, text=True)
    print('  ' + (result.stdout + result.stderr).strip().replace('\n', '\n  '))
    return result.returncode == 0


def main():
    quick = len(sys.argv) > 1 and sys.argv[1] == 'quick'

    print('===== 协议审计 =====')
    ok_audit = boot_audit()

    print('\n===== 整机仿真 =====')
    ok_sim = subprocess.run([sys.executable, str(HERE / 'sim_upgrade.py')]).returncode == 0

    print('\n===== 用错场景仿真 =====')
    ok_mistake = subprocess.run([sys.executable, str(HERE / 'sim_mistakes.py')]).returncode == 0

    ok_build = True
    if not quick:
        print('\n===== 固件编译 =====')
        ok_build = subprocess.run([sys.executable, str(HERE / 'build_firmware.py')]).returncode == 0

    print('\n审计 %s | 仿真 %s | 用错场景 %s | 编译 %s'
          % ('OK' if ok_audit else 'FAIL', 'OK' if ok_sim else 'FAIL',
             'OK' if ok_mistake else 'FAIL',
             'OK' if ok_build else ('FAIL' if not quick else '跳过')))
    return 0 if ok_audit and ok_sim and ok_mistake and ok_build else 1


if __name__ == '__main__':
    sys.exit(main())
