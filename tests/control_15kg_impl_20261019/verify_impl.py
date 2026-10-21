"""Regression of the actual 15kg production implementation; other variants are read only."""
from pathlib import Path
import contextlib
import hashlib
import importlib.util
import io
import json
import shutil
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
PROJECT=ROOT/'CH32V00X_APP_ADC_15kg'
OUT=HERE/'output'
sys.path.insert(0,str(ROOT/'tests/control_optimization_20261019'))
import run_optimization as old


def compile_host(name,source,project=PROJECT,defines=()):
    include=['-I'+str(ROOT/'tests/stubs'),'-I'+str(HERE),'-I'+str(ROOT/'tests')]
    include+=['-I'+str(project/d) for d in ('Application','Common','Drivers','System','SRC/Debug')]
    exe=OUT/(name+'.exe')
    old.run([old.CC,'-std=gnu99','-O2','-Wall','-Wextra','-Wl,--image-base,0x400000',*include,*defines,source,'-lm','-o',exe],OUT/'logs'/(name+'_compile.log'))
    return exe


def simulation():
    OUT.mkdir(exist_ok=True)
    baseline=HERE/'baseline_project'
    for directory in ('Application','Common','Drivers','System','User','SRC'):
        shutil.copytree(PROJECT/directory,baseline/directory,dirs_exist_ok=True)
    for file in (HERE/'before').rglob('*'):
        if file.is_file():shutil.copy2(file,baseline/file.relative_to(HERE/'before'))
    shutil.copy2(old.HERE/'source/sim.c',HERE/'sim.c')
    shutil.copy2(old.HERE/'source/sim_extended.c',HERE/'sim_extended.c')
    result={}
    for name,project in (('before',baseline),('after',PROJECT)):
        exe=compile_host(name,HERE/'sim.c',project)
        rows=old.parse(old.run([exe,'15kg','matrix','NUL','*','*'],OUT/'logs'/(name+'_matrix.txt')))
        assert len(rows)==351 and all(row['hard_violations']==0 for row in rows.values())
        result[name]=rows
        print('MATRIX',name,len(rows),flush=True)
    exe=compile_host('after_extended',HERE/'sim_extended.c')
    extra={}
    commands=['ext_long2','ext_long60','ext_stream200','ext_gap','ext_pause','ext_stop','ext_release','ext_jitter','ext_hold','ext_timed90']
    for load in ('noload','inertia4','grav20%_r4','randload','grav30%_r4_bl0.6'):
        for cmd in commands:
            extra.update(old.parse(old.run([exe,'15kg','matrix','NUL',load,cmd],OUT/'logs'/f'extra_{load}_{cmd}.txt')))
    for speed in (.5,1,2,4):
        for load in ('noload','grav20%_r4'):
            extra.update({key+f'_speed{speed}':row for key,row in old.parse(old.run([exe,'15kg','matrix','NUL',load,'slowv',f'SLOWV={speed}'],OUT/'logs'/f'extra_slow{speed}_{load}.txt')).items()})
    assert len(extra)==58
    (OUT/'matrix.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    (OUT/'extra.json').write_text(json.dumps(extra,indent=2),encoding='utf-8')


def checks():
    unit=(old.HERE/'unit.c').read_text(encoding='utf-8').replace('#if OPT_STREAM','#if 1')
    unit=unit.replace('PosCtrlGains_t gains={TUNE_POS_BW_RADS,TUNE_VEL_BW_RADS,TUNE_INT_BW_RADS};',
                      'PosCtrlGains_t gains={TUNE_POS_BW_RADS,TUNE_VEL_BW_RADS,TUNE_INT_BW_RADS,TUNE_LOAD_RATIO_X10};')
    unit=unit.replace('    PosCtrlDbg_t dbg;','''    PosCtrlDbg_t dbg;
    /* A positive but sub-Q16 inertia must not turn into a zero divisor. */
    MotorCal_t tiny=cal;tiny.ka_nv=1;
    C_PosCtrl_Init(&tiny,CTRL_VEL_NOISE_CDPS,TUNE_OBS_BW_HZ);
    PosCtrlGains_t clipped={20,100,5,65535U};
    assert(C_PosCtrl_SetGains(&clipped));
    assert(clipped.load_ratio_x10==POS_CTRL_LOAD_RATIO_MAX_X10 && clipped.pos_bw>=1U);
    puts("PASS small_positive_inertia_and_load_ratio_bounds");''')
    (HERE/'unit.c').write_text(unit,encoding='utf-8')
    unit_exe=compile_host('boundary',HERE/'unit.c')
    print(old.run([unit_exe],OUT/'logs/boundary.txt').strip(),flush=True)
    param_exe=compile_host('parameter_unit',HERE/'parameter_unit.c')
    print(old.run([param_exe],OUT/'logs/parameter_unit.txt').strip(),flush=True)
    fixture=HERE/'protocol_fixture.c'
    fixture.write_text('#include "variant_alignment_sim.c"\n#include "encoder_stub.h"\n',encoding='utf-8')
    for name,defines in [('protocol',()),('protection',('-DALIGN_TEST_REAL_PROTECT',)),('calibration',('-DALIGN_TEST_CALIBRATION_RUN',))]:
        exe=compile_host(name,fixture,defines=defines)
        output=old.run([exe],OUT/'logs'/(name+'.txt'))
        print(name,output.strip().splitlines()[-1],flush=True)


def target():
    spec=importlib.util.spec_from_file_location('build_actual',ROOT/'tests/verify_alignment.py')
    build=importlib.util.module_from_spec(spec);spec.loader.exec_module(build)
    build.OUT=OUT/'target'
    captured=io.StringIO()
    with contextlib.redirect_stdout(captured):
        warnings=build.build(PROJECT)
    (OUT/'target_build.txt').write_text(captured.getvalue(),encoding='utf-8')
    print(captured.getvalue(),flush=True)
    (OUT/'target_status.json').write_text(json.dumps({'passed':True,'warnings':warnings}),encoding='utf-8')


def verify_scope():
    start=json.loads((HERE/'source_hashes_before.json').read_text())
    changed=[name for name,sha in start.items() if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=sha]
    allowed={'CH32V00X_APP_ADC_15kg/'+rel for rel in ('Application/A_Config.h','Application/A_Servo.c','Common/C_Pos_Ctrl.c','Common/C_Pos_Ctrl.h')}
    assert set(changed)==allowed,changed
    (OUT/'changed_sources.json').write_text(json.dumps(changed,indent=2),encoding='utf-8')
    print('PASS scope: only the four authorized 15kg sources changed',flush=True)


if __name__=='__main__':
    if '--checks-only' not in sys.argv:simulation()
    checks();target();verify_scope()
