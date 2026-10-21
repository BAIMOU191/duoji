"""Apply only the validated noncontrol delta; retain each variant's control code."""
from pathlib import Path
import difflib, hashlib, importlib.util, json, re, shutil, subprocess, sys

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
OUT=HERE/'output'
NAMES=('A_Config.c','A_Protect.c','A_Protect.h','A_Servo.c','A_Servo.h','A_UartCmd.c')
CC=Path(r'D:\SoftWare\mingw64\bin\gcc.exe')

def run(args,log):
    p=subprocess.run(list(map(str,args)),cwd=ROOT,capture_output=True,text=True,encoding='utf-8',errors='replace')
    log.parent.mkdir(parents=True,exist_ok=True);log.write_text(p.stdout+p.stderr,encoding='utf-8')
    if p.returncode:raise RuntimeError(str(log)+'\n'+p.stdout+p.stderr)
    return p.stdout

def sync():
    assert not (HERE/'source_hashes_before.json').exists(),'Snapshot already exists; do not apply twice'
    hashes={f.relative_to(ROOT).as_posix():hashlib.sha256(f.read_bytes()).hexdigest()
        for p in ROOT.glob('CH32V00X_APP*') if p.is_dir()
        for d in ('Application','Common','Drivers','System','User','SRC')
        for f in (p/d).rglob('*') if f.suffix in ('.c','.h','.S','.ld')}
    base=ROOT/'tests/noncontrol_15kg_20261019/before/Application'
    fixed=ROOT/'CH32V00X_APP_ADC_15kg/Application'
    planned={}
    for kg in (20,30):
        p=ROOT/f'CH32V00X_APP_ADC_{kg}kg'
        for name in NAMES:
            old=(base/name).read_text(encoding='utf-8').splitlines(True)
            new=(fixed/name).read_text(encoding='utf-8').splitlines(True)
            source=(p/'Application'/name).read_text(encoding='utf-8')
            for tag,i,j,k,l in reversed(difflib.SequenceMatcher(None,old,new,autojunk=False).get_opcodes()):
                if tag=='equal':continue
                a=''.join(old[i:j]);b=''.join(new[k:l])
                if tag=='insert':
                    a=''.join(old[max(0,i-3):i]);b=a+b
                assert a and source.count(a)==1,(kg,name,tag,a,source.count(a))
                source=source.replace(a,b,1)
            planned[p/'Application'/name]=source
    # Back up the entire source before the first mutation.
    (HERE/'source_hashes_before.json').write_text(json.dumps(hashes,indent=2))
    for kg in (20,30):
        p=ROOT/f'CH32V00X_APP_ADC_{kg}kg';dest=HERE/'before'/p.name
        for d in ('Application','Common','Drivers','System','User','SRC'):shutil.copytree(p/d,dest/d)
        if (p/'obj').exists():
            (dest/'obj').mkdir()
            for ext in ('bin','hex','elf','map','lst'):
                f=p/'obj'/('CH32V00X_APP.'+ext)
                if f.exists():shutil.copy2(f,dest/'obj'/f.name)
    for file,source in planned.items():
        with file.open('w',encoding='utf-8',newline='\n') as f:f.write(source)
    print('PASS applied exactly the 15kg noncontrol delta to 20kg and 30kg',flush=True)

def compile_host(name,source,project,kg,defines=()):
    inc=['-I'+str(ROOT/'tests/stubs'),'-I'+str(ROOT/'tests')]
    inc+=['-I'+str(project/d) for d in ('Application','Common','Drivers','System','SRC/Debug')]
    exe=OUT/str(kg)/(name+'.exe')
    run([CC,'-std=gnu99','-O2','-Wall','-Wextra','-Wl,--image-base,0x400000',*inc,*defines,source,'-lm','-o',exe],exe.with_suffix('.compile.log'))
    return exe

def checks(kg):
    dst=OUT/str(kg);dst.mkdir(parents=True,exist_ok=True)
    p=ROOT/f'CH32V00X_APP_ADC_{kg}kg';before=HERE/'before'/p.name
    for label,project in (('before',before),('after',p)):
        exe=compile_host('regression_'+label,ROOT/'tests/noncontrol_15kg_20261019/regression.c',project,kg)
        output=run([exe,*(['baseline'] if label=='before' else [])],exe.with_suffix('.run.log'))
        print(str(kg)+'kg '+label+' '+output.strip().splitlines()[-1],flush=True)
    fixture=HERE/'protocol_fixture.c'
    for label,defines in (('protocol',()),('protection',('-DALIGN_TEST_REAL_PROTECT',)),('calibration',('-DALIGN_TEST_CALIBRATION_RUN',))):
        exe=compile_host(label,fixture,p,kg,defines)
        result=run([exe],exe.with_suffix('.run.log'))
        print(str(kg)+'kg '+label+' '+result.strip().splitlines()[-1],flush=True)
    sys.path.insert(0,str(ROOT/'tests/control_optimization_20261019'))
    import run_optimization as sim
    rows={}
    for label,project in (('before',before),('after',p)):
        exe=compile_host('matrix_'+label,sim.HERE/'source/sim.c',project,kg)
        rows[label]=sim.parse(run([exe,str(kg)+'kg','matrix','NUL','*','*'],exe.with_suffix('.run.log')))
        assert len(rows[label])==351 and all(r['hard_violations']==0 for r in rows[label].values())
        print(str(kg)+'kg '+label+' matrix 351; hard limits PASS',flush=True)
    (dst/'matrix.json').write_text(json.dumps(rows,indent=2))
    changed=[k for k in rows['before'] if rows['before'][k]!=rows['after'][k]]
    (dst/'matrix_changed.json').write_text(json.dumps(changed,indent=2))
    print(str(kg)+'kg matrix changed scenarios: '+str(len(changed)),flush=True)
    spec=importlib.util.spec_from_file_location('target_build',ROOT/'tests/verify_alignment.py')
    build=importlib.util.module_from_spec(spec);spec.loader.exec_module(build)
    build.OUT=dst/'baseline_target';build.build(before)
    build.OUT=dst/'target';warnings=build.build(p)
    (dst/'status.json').write_text(json.dumps({'passed':True,'warnings':warnings,'matrix_changed':len(changed)}))

def package():
    start=json.loads((HERE/'source_hashes_before.json').read_text())
    after={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in start}
    changed=[n for n in start if start[n]!=after[n]]
    allowed={f'CH32V00X_APP_ADC_{kg}kg/Application/{n}' for kg in (20,30) for n in NAMES}
    assert set(changed)==allowed,changed
    (OUT/'source_hashes_after.json').write_text(json.dumps(after,indent=2))
    (OUT/'changed_sources.json').write_text(json.dumps(changed,indent=2))
    spec=importlib.util.spec_from_file_location('build_paths',ROOT/'tests/verify_alignment.py')
    build=importlib.util.module_from_spec(spec);spec.loader.exec_module(build)
    result={}
    for kg in (20,30):
        p=ROOT/f'CH32V00X_APP_ADC_{kg}kg';dst=OUT/str(kg)
        assert json.loads((dst/'status.json').read_text())['passed']
        assert not re.search(r'^\s*#define\s+APP_MODE_CALIB\b',(p/'User/main.c').read_text(encoding='utf-8'),re.M)
        patch=[];add=remove=0
        for name in NAMES:
            before=(HERE/'before'/p.name/'Application'/name).read_text(encoding='utf-8')
            now=(p/'Application'/name).read_text(encoding='utf-8')
            delta=list(difflib.unified_diff(before.splitlines(True),now.splitlines(True),fromfile='before/Application/'+name,tofile='after/Application/'+name))
            patch.extend(delta)
            add+=sum(s.startswith('+') and not s.startswith('+++') for s in delta)
            remove+=sum(s.startswith('-') and not s.startswith('---') for s in delta)
        (dst/'implementation.diff').write_text(''.join(patch),encoding='utf-8')
        obj=p/'obj';obj.mkdir(exist_ok=True)
        for ext in ('elf','hex','bin','map'):shutil.copy2(dst/'target'/p.name/('CH32V00X_normal.'+ext),obj/('CH32V00X_APP.'+ext))
        with (obj/'CH32V00X_APP.lst').open('w',encoding='utf-8') as file:
            cp=subprocess.run([str(build.WCH/'riscv-wch-elf-objdump.exe'),'-S','-d',str(obj/'CH32V00X_APP.elf')],stdout=file,stderr=subprocess.PIPE,text=True)
            assert cp.returncode==0,cp.stderr
        result[kg]={'added_lines':add,'removed_lines':remove,'net_lines':add-remove,
            'bin_bytes':(obj/'CH32V00X_APP.bin').stat().st_size,
            'baseline_bin_bytes':(dst/'baseline_target'/p.name/'CH32V00X_normal.bin').stat().st_size,
            'hashes':{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in obj.glob('CH32V00X_APP.*')}}
    (OUT/'package.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    print('PASS source scope: only 12 application sources changed; calibration and control algorithms unchanged')

if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8');OUT.mkdir(exist_ok=True)
    if '--sync' in sys.argv:sync()
    elif '--package' in sys.argv:package()
    else:checks(int(sys.argv[1]))
