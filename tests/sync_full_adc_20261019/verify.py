from pathlib import Path
import difflib,hashlib,importlib.util,json,re,shutil,sys
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=HERE/'output'
FILES=('Application/A_Config.h','Application/A_Servo.c','Common/C_Pos_Ctrl.c','Common/C_Pos_Ctrl.h')
spec=importlib.util.spec_from_file_location('support',ROOT/'tests/sync_noncontrol_adc_20261019/verify.py')
s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s)
s.HERE=HERE;s.OUT=OUT
sys.path.insert(0,str(ROOT/'tests/control_optimization_20261019'))
import run_optimization as sim

def checks(kg):
    dst=OUT/str(kg);dst.mkdir(parents=True,exist_ok=True)
    project=ROOT/f'CH32V00X_APP_ADC_{kg}kg';before=HERE/'before'/project.name
    for name,p in (('regression_before',before),('regression_after',project)):
        exe=s.compile_host(name,ROOT/'tests/noncontrol_15kg_20261019/regression.c',p,kg)
        print(str(kg)+'kg '+s.run([exe],exe.with_suffix('.run.log')).strip().splitlines()[-1],flush=True)
    for name,source,defines in (
        ('unit',HERE/'unit.c',()),('parameters',HERE/'parameters.c',()),
        ('protocol',HERE/'protocol_fixture.c',()),('protection',HERE/'protocol_fixture.c',('-DALIGN_TEST_REAL_PROTECT',)),
        ('calibration',HERE/'protocol_fixture.c',('-DALIGN_TEST_CALIBRATION_RUN',))):
        exe=s.compile_host(name,source,project,kg,defines)
        print(str(kg)+'kg '+s.run([exe],exe.with_suffix('.run.log')).strip(),flush=True)
    rows={}
    for label,p in (('before',before),('after',project)):
        exe=s.compile_host('matrix_'+label,HERE/'sim.c',p,kg)
        rows[label]=sim.parse(s.run([exe,str(kg)+'kg','matrix','NUL','*','*'],exe.with_suffix('.run.log')))
        assert len(rows[label])==351 and all(row['hard_violations']==0 for row in rows[label].values())
        print(str(kg)+'kg '+label+' matrix 351; hard limits PASS',flush=True)
    (dst/'matrix.json').write_text(json.dumps(rows,indent=2))
    exe=s.compile_host('extended',HERE/'sim_extended.c',project,kg)
    extra={}
    commands=['ext_long2','ext_long60','ext_stream200','ext_gap','ext_pause','ext_stop','ext_release','ext_jitter','ext_hold','ext_timed90']
    for load in ('noload','inertia4','grav20%_r4','randload','grav30%_r4_bl0.6'):
        for cmd in commands:
            extra.update(sim.parse(s.run([exe,str(kg)+'kg','matrix','NUL',load,cmd],dst/'extended'/f'{load}_{cmd}.log')))
    for speed in (.5,1,2,4):
        for load in ('noload','grav20%_r4'):
            extra.update({key+f'_speed{speed}':row for key,row in sim.parse(s.run([exe,str(kg)+'kg','matrix','NUL',load,'slowv',f'SLOWV={speed}'],dst/'extended'/f'{load}_{speed}.log')).items()})
    assert len(extra)==58 and all(row['hard_violations']==0 for row in extra.values())
    (dst/'extra.json').write_text(json.dumps(extra,indent=2))
    print(str(kg)+'kg extended 58; hard limits PASS',flush=True)
    spec=importlib.util.spec_from_file_location('target_build',ROOT/'tests/verify_alignment.py')
    build=importlib.util.module_from_spec(spec);spec.loader.exec_module(build)
    build.OUT=dst/'target';warnings=build.build(project)
    (dst/'status.json').write_text(json.dumps({'passed':True,'warnings':warnings}))

def alignment():
    base=ROOT/'CH32V00X_APP_ADC_15kg'
    def profile(text):
        return {k:v for k,v in re.findall(r'^#define (FW_TORQUE|MOTOR_\w+)\s+(\S+)',text,re.M)}
    def normalized(text):
        text=re.sub(r'/\*.*?\*/','',text,flags=re.S)
        return re.sub(r'^#define (FW_TORQUE|MOTOR_\w+)\s+\S+',r'#define \1 PROFILE',text,flags=re.M)
    for kg in (20,30):
        p=ROOT/f'CH32V00X_APP_ADC_{kg}kg'
        old=(HERE/'before'/p.name/'Application/A_Config.h').read_text(encoding='utf-8')
        now=(p/'Application/A_Config.h').read_text(encoding='utf-8')
        assert profile(old)==profile(now)
        assert normalized(now)==normalized((base/'Application/A_Config.h').read_text(encoding='utf-8'))
        for d in ('Application','Common','Drivers','System','User','SRC'):
            a={f.relative_to(base).as_posix() for f in (base/d).rglob('*') if f.suffix in ('.c','.h','.S','.ld')}
            b={f.relative_to(p).as_posix() for f in (p/d).rglob('*') if f.suffix in ('.c','.h','.S','.ld')}
            assert a==b
            for rel in a-{'Application/A_Config.h'}:
                assert (base/rel).read_text(encoding='utf-8')==(p/rel).read_text(encoding='utf-8'),(kg,rel)
    print('PASS all ADC source text aligned; only identity, motor profile and explanatory comments differ')

def package():
    alignment()
    start=json.loads((HERE/'source_hashes_before.json').read_text())
    after={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in start}
    changed=[n for n in start if start[n]!=after[n]]
    assert set(changed)=={f'CH32V00X_APP_ADC_{kg}kg/{rel}' for kg in (20,30) for rel in FILES},changed
    (OUT/'source_hashes_after.json').write_text(json.dumps(after,indent=2))
    (OUT/'changed_sources.json').write_text(json.dumps(changed,indent=2))
    # Reuse the validated packaging path, allowing full relative source paths in the diff list.
    result={}
    import subprocess
    spec=importlib.util.spec_from_file_location('build_paths',ROOT/'tests/verify_alignment.py')
    build=importlib.util.module_from_spec(spec);spec.loader.exec_module(build)
    for kg in (20,30):
        p=ROOT/f'CH32V00X_APP_ADC_{kg}kg';dst=OUT/str(kg)
        assert json.loads((dst/'status.json').read_text())['passed']
        assert not re.search(r'^\s*#define\s+APP_MODE_CALIB\b',(p/'User/main.c').read_text(encoding='utf-8'),re.M)
        patch=''
        for rel in FILES:
            patch+=''.join(difflib.unified_diff((HERE/'before'/p.name/rel).read_text(encoding='utf-8').splitlines(True),(p/rel).read_text(encoding='utf-8').splitlines(True),fromfile='before/'+rel,tofile='after/'+rel))
        (dst/'implementation.diff').write_text(patch,encoding='utf-8')
        obj=p/'obj';obj.mkdir(exist_ok=True)
        for ext in ('elf','hex','bin','map'):shutil.copy2(dst/'target'/p.name/('CH32V00X_normal.'+ext),obj/('CH32V00X_APP.'+ext))
        with (obj/'CH32V00X_APP.lst').open('w',encoding='utf-8') as file:
            cp=subprocess.run([str(build.WCH/'riscv-wch-elf-objdump.exe'),'-S','-d',str(obj/'CH32V00X_APP.elf')],stdout=file,stderr=subprocess.PIPE,text=True)
            assert cp.returncode==0,cp.stderr
        result[kg]={'bin_bytes':(obj/'CH32V00X_APP.bin').stat().st_size,'hashes':{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in obj.glob('CH32V00X_APP.*')}}
    (OUT/'package.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))

if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8');OUT.mkdir(exist_ok=True)
    if '--package' in sys.argv:package()
    elif '--alignment' in sys.argv:alignment()
    else:checks(int(sys.argv[1]))
