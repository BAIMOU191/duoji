"""用已安装的 WCH RISC-V 工具链全量编译 Bootloader，检查能否放进 BOOT 区。

用法：python tests/build_firmware.py
编译参数与 MounRiver 生成的 obj/makefile 一致；工具链装在别处就改下面的 TOOL。
产物落在系统临时目录，不覆盖工程里的 obj/。
"""
import pathlib, subprocess, sys, tempfile

TOOL = pathlib.Path(r'D:/SoftWare/MounRiver/MounRiver_Studio2/resources/app/resources'
                    r'/win32/components/WCH/Toolchain/RISC-V Embedded GCC12/bin')
GCC = TOOL / 'riscv-wch-elf-gcc.exe'
SIZE = TOOL / 'riscv-wch-elf-size.exe'
FLAGS = ['-march=rv32ec_zmmul_xw', '-mabi=ilp32e', '-msmall-data-limit=0', '-msave-restore',
         '-Os', '-fsigned-char', '-ffunction-sections', '-fdata-sections', '-fno-common',
         '-Wunused', '-Wuninitialized', '-g', '-std=gnu99']
BOOT_SIZE = 3328   # CH32V006 BOOT 区容量，与 SRC/Ld/Link.ld 一致


def build(root: pathlib.Path, out: pathlib.Path):
    out.mkdir(parents=True, exist_ok=True)
    inc = [root / p for p in ['SRC/Core', 'SRC/Debug', 'SRC/Peripheral/inc', 'User', 'System']]
    sources = []
    for folder in ['User', 'System', 'SRC/Core', 'SRC/Debug', 'SRC/Peripheral/src']:
        sources += sorted((root / folder).glob('*.c'))
    sources += list((root / 'SRC/Startup').glob('*.S'))
    objects, failed = [], 0
    for src in sources:
        obj = out / (src.stem + '.o')
        objects.append(str(obj))
        r = subprocess.run([str(GCC), *FLAGS, *['-I' + str(p) for p in inc],
                            '-c', str(src), '-o', str(obj)], capture_output=True, text=True)
        if r.returncode != 0:
            failed += 1
            print('=== COMPILE FAIL:', src.name, '===')
            print(r.stderr[:4000])
        elif r.stderr.strip() and src.parent.name in ('User', 'System'):
            print('--- warnings in', src.name, '---')
            print(r.stderr[:3000])
    if failed:
        return None
    elf = out / 'CH32V00X_IAP.elf'
    r = subprocess.run([str(GCC), *FLAGS, '-T', str(root / 'SRC/Ld/Link.ld'), '-nostartfiles',
                        '-Wl,--gc-sections,-Map,' + str(out / 'CH32V00X_IAP.map'),
                        '--specs=nano.specs', '--specs=nosys.specs',
                        '-o', str(elf), *objects], capture_output=True, text=True)
    if r.returncode != 0:
        print('=== LINK FAIL ===')
        print(r.stderr[:4000])
        return None
    report = subprocess.check_output([str(SIZE), str(elf)], text=True).strip()
    print(report)
    text, data = (int(v) for v in report.splitlines()[1].split()[:2])
    used = text + data
    print(f'BOOT 区占用 {used} / {BOOT_SIZE} 字节，剩余 {BOOT_SIZE - used}')
    return elf


if __name__ == '__main__':
    root = pathlib.Path(__file__).resolve().parents[1]
    out = pathlib.Path(tempfile.gettempdir()) / ('fwbuild_' + root.name)
    print('==================', root.name, '==================')
    ok = build(root, out) is not None
    print('BUILD OK' if ok else 'BUILD ERRORS')
    sys.exit(0 if ok else 1)
