from pathlib import Path
import importlib.util,json,hashlib,subprocess,shutil,difflib,re
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
PROJECT=ROOT/'CH32V00X_APP_ADC_20kg'
spec=importlib.util.spec_from_file_location('ex',HERE/'experiment.py')
ex=importlib.util.module_from_spec(spec);spec.loader.exec_module(ex)
OUT=ex.OUT

def main():
    assert json.loads((OUT/'selected.json').read_text())['checks_passed']
    manifest=json.loads((HERE/'source_hashes_before.json').read_text())
    changed=[n for n,h in manifest.items() if hashlib.sha256((ROOT/n).read_bytes()).hexdigest()!=h]
    assert set(changed)=={'CH32V00X_APP_ADC_20kg/Common/'+n for n in ('C_Pos_Ctrl.c','C_Pos_Ctrl.h')},changed
    diff=''.join(''.join(difflib.unified_diff((HERE/'before/Common'/name).read_text(encoding='utf-8').splitlines(True),(PROJECT/'Common'/name).read_text(encoding='utf-8').splitlines(True),fromfile='before/Common/'+name,tofile='after/Common/'+name)) for name in ('C_Pos_Ctrl.c','C_Pos_Ctrl.h'))
    (HERE/'implementation.diff').write_text(diff,encoding='utf-8')

    exe=ex.s.compile_host('final_matrix',ROOT/'tests/control_optimization_20261019/source/sim.c',PROJECT,20)
    matrix=ex.sim.parse(ex.s.run([exe,'20kg','matrix','NUL','*','*'],OUT/'final_matrix.log'))
    assert matrix==json.loads((OUT/'matrix.json').read_text())['m1_r0']
    exe=ex.s.compile_host('final_pwm',ROOT/'tests/pwm_jitter_20kg_20261019/check.c',PROJECT,20)
    names=json.loads((OUT/'pwm.json').read_text())['m1_r0']
    for name in names:
        m=re.fullmatch(r'm1_r0_m(\d+)_T(\d+)_n(\d+)',name);assert m
        mode,T,noise=m.groups();trace=OUT/'final_trace.csv'
        ex.s.run([exe,mode,T,noise,'0',trace],OUT/'final_pwm.log')
        assert trace.read_bytes()==(OUT/(name+'.csv')).read_bytes(),name
    exe=ex.s.compile_host('final_jolt',ROOT/'tests/replan_jolt_20261019/jolt.c',PROJECT,20)
    txt=ex.s.run([exe],OUT/'final_jolt.log')
    rows=[{k:float(v) for k,v in (x.split('=') for x in line.split(',')[1:])} for line in txt.splitlines() if line.startswith('CASE,')]
    assert rows==json.loads((OUT/'verification.json').read_text())['candidate']['jolt']
    exe=ex.s.compile_host('final_continuity',HERE/'continuity.c',PROJECT,20)
    print(ex.s.run([exe],OUT/'final_continuity.log'),flush=True)
    print('PASS production matches candidate: 351 matrix + 486 retarget + 40 PWM traces',flush=True)

    spec=importlib.util.spec_from_file_location('target',ROOT/'tests/verify_alignment.py')
    build=importlib.util.module_from_spec(spec);spec.loader.exec_module(build)
    build.OUT=OUT/'target'
    warnings_before=build.build(HERE/'before')
    warnings_after=build.build(PROJECT)
    build.OUT=OUT/'diagnostic';build.FLAGS += ['-DTASK_FIREWATER_ENABLE=1']
    warnings_debug=build.build(PROJECT)
    normal=OUT/'target'/PROJECT.name
    diagnostic=OUT/'diagnostic'/PROJECT.name
    for label,p in (('normal',normal),('diagnostic',diagnostic)):
        nm=ex.s.run([build.WCH/'riscv-wch-elf-nm.exe',p/'CH32V00X_normal.elf'],OUT/(label+'_symbols.log'))
        assert not re.search(r'\b__(?:add|sub|mul|div)(?:df|sf)3\b|\b(?:sqrt|pow|lround)\b',nm)
        if label=='diagnostic':assert 'Task_Print' in nm
    assert not re.search(r'^\s*#define\s+APP_MODE_CALIB\b',(PROJECT/'User/main.c').read_text(encoding='utf-8'),re.M)
    for ext in ('elf','hex','bin','map'):shutil.copy2(normal/('CH32V00X_normal.'+ext),PROJECT/'obj'/('CH32V00X_APP.'+ext))
    with (PROJECT/'obj/CH32V00X_APP.lst').open('w',encoding='utf-8') as f:
        result=subprocess.run([str(build.WCH/'riscv-wch-elf-objdump.exe'),'-S','-d',str(PROJECT/'obj/CH32V00X_APP.elf')],stdout=f,stderr=subprocess.PIPE,text=True);assert result.returncode==0,result.stderr
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (PROJECT/'obj/CH32V00X_APP.bin',PROJECT/'obj/CH32V00X_APP.hex',diagnostic/'CH32V00X_normal.bin',diagnostic/'CH32V00X_normal.hex')}
    (OUT/'firmware.json').write_text(json.dumps({'checks_passed':True,'hashes':hashes,'warnings':[warnings_before,warnings_after,warnings_debug],'changed_sources':changed},indent=2))
    assert changed==[n for n,h in manifest.items() if hashlib.sha256((ROOT/n).read_bytes()).hexdigest()!=h]
    print('PASS normal packaged; diagnostic available; production scope exactly two 20kg controller files',flush=True)
    print(json.dumps(hashes,indent=2),flush=True)
if __name__=='__main__':main()
