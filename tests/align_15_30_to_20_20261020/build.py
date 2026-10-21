"""Build the three ADC app variants (normal + calibration) from source and report size."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).parent / 'output' / 'build'
WCH = Path(r'D:\SoftWare\MounRiver\MounRiver_Studio2\resources\app\resources\win32\components\WCH\Toolchain\RISC-V Embedded GCC12\bin')
CC = WCH / 'riscv-wch-elf-gcc.exe'
SIZE = WCH / 'riscv-wch-elf-size.exe'
OBJCOPY = WCH / 'riscv-wch-elf-objcopy.exe'
FLAGS = ['-march=rv32ec_zmmul_xw', '-mabi=ilp32e', '-msmall-data-limit=0',
         '-msave-restore', '-Os', '-std=gnu99', '-fsigned-char',
         '-ffunction-sections', '-fdata-sections', '-fno-common', '-Wall', '-Wextra']
PROJECTS = ['CH32V00X_APP_ADC_15kg', 'CH32V00X_APP_ADC_20kg', 'CH32V00X_APP_ADC_30kg']


def run(args, log):
    p = subprocess.run([str(x) for x in args], capture_output=True, text=True,
                       encoding='utf-8', errors='replace')
    log.parent.mkdir(parents=True, exist_ok=True)
    log.write_text(p.stdout + p.stderr, encoding='utf-8')
    if p.returncode:
        raise RuntimeError(f'{log}\n{p.stdout}\n{p.stderr}')
    return p.stdout + p.stderr


def build(name, project=None, out=None):
    project = Path(project) if project else ROOT / name
    out = Path(out) if out else OUT / name
    out.mkdir(parents=True, exist_ok=True)
    inc = ['-I' + str(project / d) for d in
           ['Application', 'Common', 'Drivers', 'System', 'User',
            'SRC/Core', 'SRC/Debug', 'SRC/Peripheral/inc']]
    sources = []
    for d in ['Application', 'Common', 'Drivers', 'System', 'User',
              'SRC/Core', 'SRC/Debug', 'SRC/Peripheral/src']:
        sources += sorted((project / d).glob('*.c'))
    sources += sorted((project / 'SRC/Startup').glob('*.S'))
    jobs, objects = [], []
    for src in sources:
        if src.name == 'main.c':   # compiled per mode below
            continue
        rel = src.relative_to(project).as_posix().replace('/', '_')
        obj = out / (rel + '.o')
        objects.append(obj)
        jobs.append((src, obj))

    def compile_one(job):
        src, obj = job
        return run([CC, *FLAGS, *inc, '-c', src, '-o', obj], obj.with_suffix('.log'))

    with ThreadPoolExecutor(max_workers=4) as pool:
        warnings = ''.join(pool.map(compile_one, jobs))
    lines = []
    for mode, defines in [('normal', []), ('calibration', ['-DAPP_MODE_CALIB'])]:
        main = out / ('main_' + mode + '.o')
        warnings += run([CC, *FLAGS, *inc, *defines, '-c', project / 'User/main.c', '-o', main],
                        main.with_suffix('.log'))
        elf = out / ('CH32V00X_' + mode + '.elf')
        warnings += run([CC, *FLAGS, '-T', project / 'SRC/Ld/Link.ld', '-nostartfiles',
                         '-Wl,--gc-sections', '-Wl,-Map,' + str(elf.with_suffix('.map')),
                         '--specs=nano.specs', '--specs=nosys.specs', '-o', elf, main, *objects],
                        out / (mode + '_link.log'))
        size = run([SIZE, elf], out / (mode + '_size.log')).strip().splitlines()[-1]
        for fmt, ext in [('ihex', 'hex'), ('binary', 'bin')]:
            run([OBJCOPY, '-O', fmt, elf, elf.with_suffix('.' + ext)], out / (mode + '_' + ext + '.log'))
        lines.append(f'{name} {mode}: {size}')
    return '\n'.join(lines), warnings


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    OUT.mkdir(parents=True, exist_ok=True)
    projects = sys.argv[1:] or PROJECTS
    total = 0
    with ThreadPoolExecutor(max_workers=3) as pool:
        for text, warnings in pool.map(build, projects):
            print(text, flush=True)
            total += warnings.count('warning:')
    print(f'PASS: {2 * len(projects)} firmware builds; compiler warnings:', total)
