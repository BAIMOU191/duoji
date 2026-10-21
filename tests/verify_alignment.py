"""Build all firmware modes from source, without using stale MRS objects."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import subprocess
import sys
import re
import shutil

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'tests/alignment_output'
WCH = Path(r'D:\SoftWare\MounRiver\MounRiver_Studio2\resources\app\resources\win32\components\WCH\Toolchain\RISC-V Embedded GCC12\bin')
CC = WCH / 'riscv-wch-elf-gcc.exe'
FLAGS = ['-march=rv32ec_zmmul_xw', '-mabi=ilp32e', '-msmall-data-limit=0',
         '-msave-restore', '-Os', '-std=gnu99', '-fsigned-char',
         '-ffunction-sections', '-fdata-sections', '-fno-common', '-Wall', '-Wextra']

def verify_framework(projects):
    base = ROOT/'CH32V00X_APP_ADC_30kg'
    base_config = (base/'Application/A_Config.h').read_text(encoding='utf-8')
    defaults = re.findall(r'^#define (CFG_DEF_\w+)\s+([^\n]+)', base_config, re.M)
    for project in projects:
        for source in (base/'Common').glob('*'):
            if source.is_file():
                assert source.read_bytes()==(project/'Common'/source.name).read_bytes(), str(project/source.name)
        if 'ADC' in project.name:
            for directory in ['Application', 'Drivers', 'System', 'User']:
                for source in (base/directory).glob('*'):
                    if not source.is_file() or source.name in ['A_Config.h', 'A_Sensor.c', 'A_Sensor.h', 'main.c']: continue
                    assert source.read_text(encoding='utf-8') == (project/directory/source.name).read_text(encoding='utf-8'), str(project/source.name)
            for source in project.rglob('*.c'):
                if source.name=='C_Traj_Planner.c': continue
                assert not re.search(r'\bC_Traj_(?:GetPos|SetRangeOpen)\s*\(', source.read_text(encoding='utf-8')), str(source)
        config = (project/'Application/A_Config.h').read_text(encoding='utf-8')
        for key, value in defaults:
            actual = re.search(r'^#define '+key+r'\s+([^\n]+)', config, re.M)
            assert actual and actual[1] == value, str(project/key)
        sensor = (project/'Application/A_Sensor.c').read_text(encoding='utf-8')
        assert 'A_Current_ReadSigned' in sensor, str(project)
        if 'IIC' in project.name:
            adc = (project/'Drivers/D_adc.c').read_text(encoding='utf-8')
            assert re.search(r'^#define RANKS\s+1U', adc, re.M)
            assert 'ADC_Channel_2' not in adc and 'GPIO_Init(GPIOC' not in adc
            assert 'D_ADC_Encoder_Read' not in adc
            assert 'D_MT6701_Read_Angle' in sensor
            for rel in ['Drivers/D_motor.c', 'Drivers/D_motor.h', 'Drivers/D_tim.c',
                        'Drivers/D_tim.h', 'User/ch32v00X_it.c', 'Application/A_Tasks.c']:
                assert (base/rel).read_bytes() == (project/rel).read_bytes(), str(project/rel)
        for directory in ['Application', 'Common', 'System', 'User']:
            for source in (project/directory).glob('*'):
                if source.suffix in ['.c', '.h']:
                    assert 'MOTOR_CALIBRATION_VALID' not in source.read_text(encoding='utf-8'), str(source)
    print('PASS: ADC framework and defaults aligned to current 30kg; IIC retains single-rank ADC and magnetic feedback', flush=True)

def calibrated(project):
    config = (project/'Application/A_Config.h').read_text(encoding='utf-8')
    values = {k:int(v) for k,v in re.findall(r'^#define (MOTOR_\w+)\s+(-?\d+)', config, re.M)}
    return all(values.get(k, 0) > 0 for k in ['MOTOR_KV_UV', 'MOTOR_KA_NV', 'MOTOR_CAL_MV',
        'MOTOR_MIN_STEP_CDEG', 'MOTOR_RATED_VEL_DPS', 'MOTOR_RATED_ACC_DPS2', 'MOTOR_RATED_DEC_DPS2'])

def test_profile(project):
    """Inject a valid model before static assertions in an isolated source copy only."""
    staged = OUT/project.name/'synthetic_model_TARGET_CHECK_ONLY'/'profile'
    for directory in ['Application', 'Common', 'Drivers', 'System', 'User', 'SRC']:
        shutil.copytree(project/directory, staged/directory, dirs_exist_ok=True)
    fixture = (ROOT/'tests/variant_alignment_sim.c').read_text(encoding='utf-8')
    definitions = fixture.split('#ifdef ALIGN_TEST_CALIBRATED\n',1)[1].split('#endif',1)[0]
    path = staged/'Application/A_Config.h'
    config = path.read_text(encoding='utf-8')
    for key,value in re.findall(r'^#define (MOTOR_\w+)\s+(\d+)', definitions, re.M):
        config = re.sub(r'^(#define '+key+r'\s+)\S+', lambda m:m[1]+value, config, flags=re.M)
    path.write_text(config, encoding='utf-8')
    main = staged/'User/main.c'
    main.write_text(re.sub(r'^#define APP_MODE_CALIB\s*$', '// #define APP_MODE_CALIB',
                          main.read_text(encoding='utf-8'), flags=re.M), encoding='utf-8')
    return staged

def run(args, log):
    p = subprocess.run([str(x) for x in args], capture_output=True, text=True, encoding='utf-8', errors='replace')
    log.write_text(p.stdout+p.stderr, encoding='utf-8')
    if p.returncode:
        raise RuntimeError(str(log)+'\n'+p.stdout+p.stderr)
    return p.stdout+p.stderr

def build(project, synthetic=False):
    name = project.name
    out = OUT / project.name
    if synthetic:
        project = test_profile(project)
        out = out / 'synthetic_model_TARGET_CHECK_ONLY'
    out.mkdir(parents=True, exist_ok=True)
    inc = ['-I'+str(project/d) for d in ['Application', 'Common', 'Drivers', 'System', 'User',
                                       'SRC/Core', 'SRC/Debug', 'SRC/Peripheral/inc']]
    sources = []
    for d in ['Application', 'Common', 'Drivers', 'System', 'User', 'SRC/Core', 'SRC/Debug', 'SRC/Peripheral/src']:
        sources += list((project/d).glob('*.c'))
    sources += list((project/'SRC/Startup').glob('*.S'))
    jobs = []
    objects = []
    for src in sources:
        if src.name == 'main.c': continue
        rel = src.relative_to(project).as_posix().replace('/', '_')
        obj = out / (rel+'.o')
        objects.append(obj)
        jobs.append((src, obj))
    def compile(job):
        src, obj = job
        return run([CC, *FLAGS, *inc, '-c', src, '-o', obj], obj.with_suffix('.log'))
    with ThreadPoolExecutor(max_workers=4) as pool:
        warnings = ''.join(pool.map(compile, jobs))
    for mode in (['NORMAL_TEST_ONLY', 'CALIBRATION_TEST_ONLY'] if synthetic else ['normal', 'calibration']):
        main = out / ('main_'+mode+'.o')
        defines = ['-DAPP_MODE_CALIB'] if mode.lower().startswith('calibration') else []
        warnings += run([CC, *FLAGS, *inc, *defines, '-c', project/'User/main.c', '-o', main], main.with_suffix('.log'))
        elf = out / ('CH32V00X_'+mode+'.elf')
        warnings += run([CC, *FLAGS, '-T', project/'SRC/Ld/Link.ld', '-nostartfiles',
                        '-Wl,--gc-sections', '-Wl,-Map,'+str(elf.with_suffix('.map')),
                        '--specs=nano.specs', '--specs=nosys.specs', '-o', elf, main, *objects],
                       out / (mode+'_link.log'))
        size = run([WCH/'riscv-wch-elf-size.exe', elf], out / (mode+'_size.log'))
        for fmt, ext in ([] if synthetic else [('ihex', 'hex'), ('binary', 'bin')]):
            run([WCH/'riscv-wch-elf-objcopy.exe', '-O', fmt, elf, elf.with_suffix('.'+ext)], out/(mode+'_'+ext+'.log'))
        print(name, mode, size.strip().splitlines()[-1], flush=True)
    if warnings.strip(): print('Compiler diagnostics:', name, warnings[:5000], flush=True)
    return warnings.count('warning:')

def simulate(project):
    out = OUT / project.name
    out.mkdir(parents=True, exist_ok=True)
    staged = test_profile(project)
    modes = (['stored'] if calibrated(project) else []) + ['synthetic_calibration', 'calibration_run', 'protection_run']
    for mode in modes:
        profile = project if mode == 'stored' else staged
        inc = ['-I'+str(ROOT/'tests/stubs')]+['-I'+str(profile/d) for d in
               ['Application', 'Common', 'Drivers', 'System', 'SRC/Debug']]
        exe = out / ('simulation_'+mode+'.exe')
        defines = ['-DALIGN_TEST_CALIBRATED'] if mode != 'stored' else []
        if mode == 'calibration_run': defines.append('-DALIGN_TEST_CALIBRATION_RUN')
        if mode == 'protection_run': defines.append('-DALIGN_TEST_REAL_PROTECT')
        run([r'D:\SoftWare\mingw64\bin\gcc.exe', '-std=gnu99', '-O2', '-Wall', '-Wextra',
             *inc, *defines, ROOT/'tests/variant_alignment_sim.c', '-lm', '-o', exe],
            out / (mode+'_host_compile.log'))
        result = run([exe], out / (mode+'_host_run.log'))
        if mode == 'calibration_run':
            assert '#error' not in result and '@DONE' in result, 'Calibration self-check failed: '+str(out)
            result = result.strip().splitlines()[-1]
        print(project.name, mode, result.strip(), flush=True)

if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    OUT.mkdir(exist_ok=True)
    (OUT/'.gitignore').write_text('*\n!.gitignore\n', encoding='utf-8')
    projects = sorted(ROOT.glob('CH32V00X_APP*'))
    verify_framework(projects)
    if '--build-only' not in sys.argv:
        for project in projects: simulate(project)
    if '--host-only' not in sys.argv:
        count = 0
        for project in projects:
            synthetic = not calibrated(project)
            if synthetic: print('TEST COPY ONLY: pending production calibration preserved for', project.name, flush=True)
            count += build(project, synthetic=synthetic)
        print('PASS: 10 firmware builds (pending profiles use calibrated test copies); compiler warnings:', count)
