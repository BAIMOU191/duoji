from pathlib import Path
import subprocess, sys, json, hashlib, importlib.util, shutil, difflib, re
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
PROJECT=ROOT/'CH32V00X_APP_ADC_15kg'
OUT=HERE/'output'
CC=Path(r'D:\SoftWare\mingw64\bin\gcc.exe')

def run(args, log):
    p=subprocess.run(list(map(str,args)),capture_output=True,text=True,encoding='utf-8',errors='replace')
    log.parent.mkdir(parents=True,exist_ok=True);log.write_text(p.stdout+p.stderr,encoding='utf-8')
    if p.returncode:raise RuntimeError(str(log)+'\n'+p.stdout+p.stderr)
    return p.stdout

def host(baseline=False):
    project=HERE/'before' if baseline else PROJECT
    inc=['-I'+str(ROOT/'tests/stubs'),'-I'+str(ROOT/'tests')]
    inc+=['-I'+str(project/d) for d in ('Application','Common','Drivers','System','SRC/Debug')]
    exe=OUT/('baseline.exe' if baseline else 'regression.exe')
    run([CC,'-std=gnu99','-O2','-Wall','-Wextra','-Wl,--image-base,0x400000',*inc,HERE/'regression.c','-lm','-o',exe],exe.with_suffix('.compile.log'))
    print(run([exe,*(['baseline'] if baseline else [])],exe.with_suffix('.run.log')),flush=True)

def target(baseline=False):
    spec=importlib.util.spec_from_file_location('target_build',ROOT/'tests/verify_alignment.py')
    build=importlib.util.module_from_spec(spec);spec.loader.exec_module(build)
    build.OUT=OUT/('baseline_target' if baseline else 'target')
    warnings=build.build(HERE/'before' if baseline else PROJECT)
    if not baseline:(OUT/'target_status.json').write_text(json.dumps({'passed':True,'warnings':warnings}))

def broad():
    spec=importlib.util.spec_from_file_location('impl',ROOT/'tests/control_15kg_impl_20261019/verify_impl.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    m.HERE=HERE;m.OUT=OUT/'broad';m.OUT.mkdir(exist_ok=True)
    for f in ('parameter_unit.c','encoder_stub.h'):
        shutil.copy2(ROOT/'tests/control_15kg_restore_speed_20261019'/f,HERE/f)
    for f in ('sim.c','sim_extended.c'):
        shutil.copy2(m.old.HERE/'source'/f,HERE/f)
    m.checks()
    rows={}
    for name,project in (('before',HERE/'before'),('after',PROJECT)):
        exe=m.compile_host('matrix_'+name,HERE/'sim.c',project)
        rows[name]=m.old.parse(m.old.run([exe,'15kg','matrix','NUL','*','*'],m.OUT/'logs'/('matrix_'+name+'.txt')))
        assert len(rows[name])==351 and all(r['hard_violations']==0 for r in rows[name].values())
        print('PASS hard output limits: '+name+' 351 scenarios',flush=True)
    (m.OUT/'matrix.json').write_text(json.dumps(rows,indent=2))
    changed=[key for key in rows['before'] if rows['before'][key]!=rows['after'][key]]
    (m.OUT/'matrix_changed.json').write_text(json.dumps(changed,indent=2))
    print('Matrix changed scenarios:',len(changed),flush=True)

def package():
    assert json.loads((OUT/'target_status.json').read_text())['passed']
    assert not re.search(r'^\s*#define\s+APP_MODE_CALIB\b',(PROJECT/'User/main.c').read_text(encoding='utf-8'),re.M)
    start=json.loads((HERE/'source_hashes_before.json').read_text())
    after={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in start}
    changed=[name for name in start if start[name]!=after[name]]
    allowed={'CH32V00X_APP_ADC_15kg/Application/'+name for name in (
        'A_Config.c','A_Protect.c','A_Protect.h','A_Servo.c','A_Servo.h','A_UartCmd.c')}
    assert set(changed)==allowed,changed
    (OUT/'source_hashes_after.json').write_text(json.dumps(after,indent=2))
    (OUT/'changed_sources.json').write_text(json.dumps(changed,indent=2))
    patch=[];added=removed=0
    for name in sorted(changed):
        rel=Path(name).relative_to(PROJECT.name)
        d=list(difflib.unified_diff((HERE/'before'/rel).read_text(encoding='utf-8').splitlines(True),
                                   (ROOT/name).read_text(encoding='utf-8').splitlines(True),
                                   fromfile='before/'+rel.as_posix(),tofile='after/'+rel.as_posix()))
        patch.extend(d)
        added+=sum(s.startswith('+') and not s.startswith('+++') for s in d)
        removed+=sum(s.startswith('-') and not s.startswith('---') for s in d)
    (HERE/'implementation.diff').write_text(''.join(patch),encoding='utf-8')
    spec=importlib.util.spec_from_file_location('build_paths',ROOT/'tests/verify_alignment.py')
    build=importlib.util.module_from_spec(spec);spec.loader.exec_module(build)
    dest=PROJECT/'obj';dest.mkdir(exist_ok=True)
    for ext in ('elf','hex','bin','map'):
        shutil.copy2(OUT/'target'/PROJECT.name/('CH32V00X_normal.'+ext),dest/('CH32V00X_APP.'+ext))
    with (dest/'CH32V00X_APP.lst').open('w',encoding='utf-8') as file:
        result=subprocess.run([str(build.WCH/'riscv-wch-elf-objdump.exe'),'-S','-d',str(dest/'CH32V00X_APP.elf')],stdout=file,stderr=subprocess.PIPE,text=True)
        assert result.returncode==0,result.stderr
    metadata={'changed_sources':changed,'added_lines':added,'removed_lines':removed,'net_lines':added-removed,
              'bin_bytes':(dest/'CH32V00X_APP.bin').stat().st_size,
              'previous_disk_bin_bytes':(HERE/'before/obj/CH32V00X_APP.bin').stat().st_size,
              'source_baseline_bin_bytes':(OUT/'baseline_target/before/CH32V00X_normal.bin').stat().st_size,
              'firmware_hashes':{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in dest.glob('CH32V00X_APP.*')}}
    (OUT/'package.json').write_text(json.dumps(metadata,indent=2))
    print('PASS source scope: six 15kg application files; other variants and control algorithms unchanged')
    print('Code lines:',added,'added,',removed,'removed,',added-removed,'net')
    print('BIN bytes:',metadata['bin_bytes'],'recompiled baseline:',metadata['source_baseline_bin_bytes'])
    print('BIN SHA256:',metadata['firmware_hashes']['CH32V00X_APP.bin'])

if __name__=='__main__':
    OUT.mkdir(exist_ok=True)
    if '--baseline' in sys.argv:host(True)
    elif '--baseline-target' in sys.argv:target(True)
    elif '--target' in sys.argv:target()
    elif '--broad' in sys.argv:broad()
    elif '--package' in sys.argv:package()
    else:host()
