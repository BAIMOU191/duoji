from pathlib import Path
import importlib.util,sys,json,hashlib,shutil,difflib,re,subprocess
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=HERE/'output'
PROJECT=ROOT/'CH32V00X_APP_ADC_20kg'
spec=importlib.util.spec_from_file_location('support',ROOT/'tests/sync_noncontrol_adc_20261019/verify.py')
s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s);s.HERE=HERE;s.OUT=OUT
sys.path.insert(0,str(ROOT/'tests/control_optimization_20261019'))
import run_optimization as sim

def host():
    (OUT/'20').mkdir(exist_ok=True)
    for label,p in (('before',HERE/'before'),('after',PROJECT)):
        exe=s.compile_host('jolt_'+label,ROOT/'tests/replan_jolt_20261019/jolt.c',p,20)
        output=s.run([exe],exe.with_suffix('.run.log'))
        rows=[]
        for line in output.splitlines():
            if line.startswith('CASE,'):rows.append({k:float(v) for k,v in (x.split('=') for x in line.split(',')[1:])})
        assert len(rows)==486 and all(r['hard']==0 for r in rows)
        (OUT/f'jolt_{label}.json').write_text(json.dumps(rows,indent=2))
        print(label,'486 cases; faults',sum(r['fault'] for r in rows),'out',sum(abs(r['final'])>.5 for r in rows),flush=True)
    for name,src,defs in (
        ('planner',HERE/'planner_unit.c',()),
        ('noncontrol',ROOT/'tests/noncontrol_15kg_20261019/regression.c',()),
        ('controller',ROOT/'tests/noncontrol_15kg_20261019/unit.c',()),
        ('parameters',HERE/'parameters.c',()),
        ('protocol',HERE/'protocol_fixture.c',()),
        ('protection',HERE/'protocol_fixture.c',('-DALIGN_TEST_REAL_PROTECT',)),
        ('calibration',HERE/'protocol_fixture.c',('-DALIGN_TEST_CALIBRATION_RUN',))):
        exe=s.compile_host(name,src,PROJECT,20,defs)
        print(name,s.run([exe],exe.with_suffix('.run.log')).strip().splitlines()[-1],flush=True)
    rows={}
    for label,p in (('before',HERE/'before'),('after',PROJECT)):
        exe=s.compile_host('matrix_'+label,sim.HERE/'source/sim.c',p,20)
        rows[label]=sim.parse(s.run([exe,'20kg','matrix','NUL','*','*'],exe.with_suffix('.run.log')))
        assert len(rows[label])==351 and all(r['hard_violations']==0 for r in rows[label].values())
        print(label,'main matrix 351; hard limits PASS',flush=True)
    (OUT/'matrix.json').write_text(json.dumps(rows,indent=2))
    extended=s.compile_host('extended',sim.HERE/'source/sim_extended.c',PROJECT,20)
    extra={}
    for ld in ('noload','inertia4','grav20%_r4','randload','grav30%_r4_bl0.6'):
        for cmd in ('ext_long2','ext_long60','ext_stream200','ext_gap','ext_pause','ext_stop','ext_release','ext_jitter','ext_hold','ext_timed90'):
            extra.update(sim.parse(s.run([extended,'20kg','matrix','NUL',ld,cmd],OUT/'extended'/f'{ld}_{cmd}.log')))
    assert len(extra)==50 and all(r['hard_violations']==0 for r in extra.values())
    (OUT/'extra.json').write_text(json.dumps(extra,indent=2));print('extended 50 PASS hard limits',flush=True)

def target():
    spec=importlib.util.spec_from_file_location('build',ROOT/'tests/verify_alignment.py')
    build=importlib.util.module_from_spec(spec);spec.loader.exec_module(build);build.OUT=OUT/'target'
    warnings=build.build(PROJECT)
    (OUT/'target_status.json').write_text(json.dumps({'passed':True,'warnings':warnings}))

def scope():
    before=json.loads((HERE/'source_hashes_before.json').read_text())
    after={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in before}
    changed=[n for n in before if before[n]!=after[n]]
    assert set(changed)=={'CH32V00X_APP_ADC_20kg/'+n for n in ('Application/A_Servo.c','Common/C_Traj_Planner.c','Common/C_Traj_Planner.h')},changed
    (OUT/'source_hashes_after.json').write_text(json.dumps(after,indent=2))
    diff=''
    for rel in ('Application/A_Servo.c','Common/C_Traj_Planner.c','Common/C_Traj_Planner.h'):
        diff+=''.join(difflib.unified_diff((HERE/'before'/rel).read_text(encoding='utf-8').splitlines(True),(PROJECT/rel).read_text(encoding='utf-8').splitlines(True),fromfile='before/'+rel,tofile='after/'+rel))
    (HERE/'implementation.diff').write_text(diff,encoding='utf-8')
    print('PASS scope: only 20kg planner and timed command dispatch changed',flush=True)

def package():
    scope()
    assert json.loads((OUT/'target_status.json').read_text())['passed']
    assert not re.search(r'^\s*#define\s+APP_MODE_CALIB\b',(PROJECT/'User/main.c').read_text(encoding='utf-8'),re.M)
    for name in ('noncontrol','planner','parameters','protocol','protection','calibration'):
        assert 'PASS' in (OUT/'20'/(name+'.run.log')).read_text(encoding='utf-8'),name
    assert len(json.loads((OUT/'jolt_after.json').read_text()))==486
    spec=importlib.util.spec_from_file_location('build',ROOT/'tests/verify_alignment.py')
    build=importlib.util.module_from_spec(spec);spec.loader.exec_module(build)
    src=OUT/'target'/PROJECT.name;obj=PROJECT/'obj';obj.mkdir(exist_ok=True)
    # The linked normal image must not acquire software floating point routines.
    symbols=s.run([build.WCH/'riscv-wch-elf-nm.exe',src/'CH32V00X_normal.elf'],OUT/'symbols.log')
    assert not re.search(r'\b__(?:add|sub|mul|div)(?:df|sf)3\b|\b(?:sqrt|pow|lround)\b',symbols)
    for ext in ('elf','hex','bin','map'):shutil.copy2(src/('CH32V00X_normal.'+ext),obj/('CH32V00X_APP.'+ext))
    with (obj/'CH32V00X_APP.lst').open('w',encoding='utf-8') as f:
        p=subprocess.run([str(build.WCH/'riscv-wch-elf-objdump.exe'),'-S','-d',str(obj/'CH32V00X_APP.elf')],stdout=f,stderr=subprocess.PIPE,text=True)
        assert p.returncode==0,p.stderr
    hashes={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in obj.glob('CH32V00X_APP.*')}
    (OUT/'firmware_hashes.json').write_text(json.dumps(hashes,indent=2))
    assert (obj/'CH32V00X_APP.bin').read_bytes()==(src/'CH32V00X_normal.bin').read_bytes()
    print('PASS normal firmware packaged; no floating point routines')
    print('BIN bytes:',(obj/'CH32V00X_APP.bin').stat().st_size,'SHA256:',hashes['CH32V00X_APP.bin'])

if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8');OUT.mkdir(exist_ok=True)
    if '--target' in sys.argv:target()
    elif '--scope' in sys.argv:scope()
    elif '--package' in sys.argv:package()
    else:host();scope()
