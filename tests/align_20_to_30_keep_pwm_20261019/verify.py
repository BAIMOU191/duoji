from pathlib import Path
import importlib.util,hashlib,json,re,shutil,sys,subprocess,difflib
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=HERE/'output'
P20=ROOT/'CH32V00X_APP_ADC_20kg'
P30=ROOT/'CH32V00X_APP_ADC_30kg'
FOLDERS=('Application','Common','Drivers','System','User','SRC')
KEEP={'Application/A_Config.h','Common/C_Pos_Ctrl.c','Common/C_Pos_Ctrl.h'}
spec=importlib.util.spec_from_file_location('support',ROOT/'tests/sync_noncontrol_adc_20261019/verify.py')
s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s);s.OUT=OUT
sys.path.insert(0,str(ROOT/'tests/control_optimization_20261019'))
import run_optimization as sim

def sources(project):
    return {f.relative_to(project).as_posix():f for d in FOLDERS for f in (project/d).rglob('*') if f.is_file()}
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def apply():
    assert not (HERE/'source_hashes_before.json').exists(),'Already snapshotted; do not apply twice'
    hashes={f.relative_to(ROOT).as_posix():digest(f) for p in ROOT.glob('CH32V00X_APP*') if p.is_dir() for d in FOLDERS for f in (p/d).rglob('*') if f.is_file()}
    (HERE/'source_hashes_before.json').write_text(json.dumps(hashes,indent=2))
    for d in (*FOLDERS,'obj'):
        if (P20/d).exists():shutil.copytree(P20/d,HERE/'before'/d)
    old=sources(P20);ref=sources(P30);assert old.keys()==ref.keys()
    changed=[]
    for rel in old:
        if rel not in KEEP and old[rel].read_bytes()!=ref[rel].read_bytes():
            shutil.copy2(ref[rel],old[rel]);changed.append(rel)
    (OUT/'changed_sources.json').write_text(json.dumps(changed,indent=2))
    print('Aligned 20kg files:',changed,flush=True)
    scope()

def scope():
    old=json.loads((HERE/'source_hashes_before.json').read_text())
    after={n:digest(ROOT/n) for n in old}
    changed={n for n in old if old[n]!=after[n]}
    expected={'CH32V00X_APP_ADC_20kg/'+n for n in ('Application/A_Servo.c','Application/A_Sensor.c','Application/A_Sensor.h','Common/C_Traj_Planner.c','Common/C_Traj_Planner.h')}
    assert changed==expected,changed
    for rel in KEEP:assert digest(P20/rel)==digest(HERE/'before'/rel),rel
    for rel,f in sources(P20).items():
        if rel not in KEEP:assert f.read_bytes()==(P30/rel).read_bytes(),rel
    def normalized_config(text):
        text=re.sub(r'/\*.*?\*/','',text,flags=re.S)
        return re.sub(r'^#define (FW_TORQUE|MOTOR_\w+)\s+\S+',r'#define \1 PROFILE',text,flags=re.M)
    assert normalized_config((P20/'Application/A_Config.h').read_text(encoding='utf-8'))==normalized_config((P30/'Application/A_Config.h').read_text(encoding='utf-8'))
    assert 'C_Traj_PlanTimed' not in (P20/'Application/A_Servo.c').read_text(encoding='utf-8')
    (OUT/'source_hashes_after.json').write_text(json.dumps(after,indent=2))
    patch=''.join(''.join(difflib.unified_diff((HERE/'before'/rel).read_text(encoding='utf-8').splitlines(True),(P20/rel).read_text(encoding='utf-8').splitlines(True),fromfile='before/'+rel,tofile='after/'+rel)) for rel in json.loads((OUT/'changed_sources.json').read_text()))
    (HERE/'implementation.diff').write_text(patch,encoding='utf-8')
    print('PASS common code matches 30kg; 20kg calibration and PWM controller preserved byte-for-byte; all other projects unchanged',flush=True)

def host():
    scope();(OUT/'20').mkdir(parents=True,exist_ok=True)
    for name,source,defs in (
        ('continuity',ROOT/'tests/pwm_opt_20kg_20261019/continuity.c',()),
        ('controller',ROOT/'tests/noncontrol_15kg_20261019/unit.c',()),
        ('regression',ROOT/'tests/noncontrol_15kg_20261019/regression.c',()),
        ('parameters',ROOT/'tests/replan_20kg_impl_20261019/parameters.c',()),
        ('protocol',ROOT/'tests/replan_20kg_impl_20261019/protocol_fixture.c',()),
        ('protection',ROOT/'tests/replan_20kg_impl_20261019/protocol_fixture.c',('-DALIGN_TEST_REAL_PROTECT',)),
        ('calibration',ROOT/'tests/replan_20kg_impl_20261019/protocol_fixture.c',('-DALIGN_TEST_CALIBRATION_RUN',)),
    ):
        exe=s.compile_host(name,source,P20,20,defs)
        print(name,s.run([exe],exe.with_suffix('.run.log')).splitlines()[-1],flush=True)
    matrices={};jolts={};pwms={}
    spec=importlib.util.spec_from_file_location('pwm',ROOT/'tests/pwm_jitter_20kg_20261019/run.py')
    pwm=importlib.util.module_from_spec(spec);spec.loader.exec_module(pwm)
    for label,project in (('before',HERE/'before'),('after',P20)):
        exe=s.compile_host('matrix_'+label,sim.HERE/'source/sim.c',project,20)
        matrices[label]=sim.parse(s.run([exe,'20kg','matrix','NUL','*','*'],OUT/(label+'_matrix.log')))
        assert len(matrices[label])==351 and all(r['hard_violations']==0 for r in matrices[label].values())
        exe=s.compile_host('jolt_'+label,ROOT/'tests/replan_jolt_20261019/jolt.c',project,20)
        output=s.run([exe],OUT/(label+'_jolt.log'))
        jolts[label]=[{k:float(v) for k,v in (item.split('=') for item in line.split(',')[1:])} for line in output.splitlines() if line.startswith('CASE,')]
        assert len(jolts[label])==486 and all(r['hard']==0 for r in jolts[label])
        exe=s.compile_host('pwm_'+label,ROOT/'tests/pwm_jitter_20kg_20261019/check.c',project,20)
        pwms[label]={}
        for mode,T in ((0,0),(0,1000),(3,0),(3,1000),(4,0),(4,1000),(5,0),(6,0),(8,0),(9,0)):
            for noise in (0,1,2,5):
                tag=f'm{mode}_T{T}_n{noise}';csv=OUT/(label+'_'+tag+'.csv')
                s.run([exe,mode,T,noise,0,csv],OUT/'pwm.log');pwms[label][tag]=pwm.analyze(csv)
                if label=='after' and T==0:assert csv.read_bytes()==(OUT/('before_'+tag+'.csv')).read_bytes(),tag
        print(label,'matrix fault events',sum(r['fault'] for r in matrices[label].values()),'retarget fault events',sum(r['fault'] for r in jolts[label]),flush=True)
    (OUT/'matrix.json').write_text(json.dumps(matrices,indent=2));(OUT/'jolt.json').write_text(json.dumps(jolts,indent=2));(OUT/'pwm.json').write_text(json.dumps(pwms,indent=2))
    new_fault=[k for k in matrices['before'] if matrices['after'][k]['fault']>matrices['before'][k]['fault']]
    jolt_fault=[i for i,(a,b) in enumerate(zip(jolts['before'],jolts['after'])) if b['fault']>a['fault']]
    print('New fault cases:',new_fault,jolt_fault,flush=True)
    assert not new_fault and not jolt_fault
    assert all(pwms['after'][k]['all']['fault']==0 for k in pwms['after'])
    summary={'passed':True,'matrix_cases_each':351,'retarget_cases_each':486,'pwm_cases_each':40,'T0_pwm_identical':True,'new_fault_cases':new_fault+jolt_fault,'retarget_outside_0_5_deg':{lab:sum(abs(r['final'])>.5 for r in rows) for lab,rows in jolts.items()}}
    (OUT/'host_status.json').write_text(json.dumps(summary,indent=2));scope();print(json.dumps(summary),flush=True)

def target():
    assert json.loads((OUT/'host_status.json').read_text())['passed'];scope()
    spec=importlib.util.spec_from_file_location('build',ROOT/'tests/verify_alignment.py')
    build=importlib.util.module_from_spec(spec);spec.loader.exec_module(build);build.OUT=OUT/'target'
    build.build(P20)
    normal=build.OUT/P20.name
    build.OUT=OUT/'diagnostic';build.FLAGS+=['-DTASK_FIREWATER_ENABLE=1'];build.build(P20)
    diagnostic=build.OUT/P20.name
    for label,folder in (('normal',normal),('diagnostic',diagnostic)):
        symbols=s.run([build.WCH/'riscv-wch-elf-nm.exe',folder/'CH32V00X_normal.elf'],OUT/(label+'_symbols.log'))
        assert not re.search(r'\b__(?:add|sub|mul|div)(?:df|sf)3\b|\b(?:sqrt|pow|lround)\b',symbols)
        if label=='diagnostic':assert 'Task_Print' in symbols
    assert not re.search(r'^\s*#define\s+APP_MODE_CALIB\b',(P20/'User/main.c').read_text(encoding='utf-8'),re.M)
    for ext in ('elf','hex','bin','map'):shutil.copy2(normal/('CH32V00X_normal.'+ext),P20/'obj'/('CH32V00X_APP.'+ext))
    with (P20/'obj/CH32V00X_APP.lst').open('w',encoding='utf-8') as f:
        p=subprocess.run([str(build.WCH/'riscv-wch-elf-objdump.exe'),'-S','-d',str(P20/'obj/CH32V00X_APP.elf')],stdout=f,stderr=subprocess.PIPE,text=True);assert p.returncode==0,p.stderr
    files=[P20/'obj/CH32V00X_APP.bin',P20/'obj/CH32V00X_APP.hex',diagnostic/'CH32V00X_normal.bin',diagnostic/'CH32V00X_normal.hex']
    (OUT/'firmware.json').write_text(json.dumps({str(f.relative_to(ROOT)):digest(f) for f in files},indent=2))
    scope();print('PASS normal and diagnostic packaged',flush=True)

if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8');HERE.mkdir(exist_ok=True);OUT.mkdir(exist_ok=True)
    if '--apply' in sys.argv:apply()
    elif '--target' in sys.argv:target()
    else:host()
