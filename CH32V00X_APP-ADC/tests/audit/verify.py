"""Regression against the user's hardware-validated snapshot; no IAP execution."""
import pathlib, subprocess, sys, json, itertools, hashlib
from run_audit import ROOT, HERE, OUT, compile_host
sys.path.insert(0,str(ROOT/'tests/sim'))
import run

def execute(args):
    r=subprocess.run(list(map(str,args)),capture_output=True,text=True,timeout=60)
    if r.returncode: raise RuntimeError(f'{args}: {r.returncode}\n{r.stdout}\n{r.stderr}')
    return r.stdout.strip()

if __name__=='__main__':
    results={}
    exe=compile_host('sanitized_servo',ROOT,HERE/'servo_audit.c',True)
    results['sanitized_state']={s:execute([exe,s]) for s in ['stop','release','pause','range','save','protection','invalid_range']}
    exe=compile_host('sanitized_sim',ROOT,ROOT/'tests/sim/servo_sim.c',True)
    results['sanitized_regression']=execute([exe,'regression'])
    for name in ['planner','protect']:
        exe=OUT/f'sanitized_{name}.exe'
        execute(['gcc','-std=c99','-O1','-g','-Wall','-Wextra','-Wno-unused-parameter','-fsanitize=undefined','-fsanitize-undefined-trap-on-error',
                 '-I'+str(ROOT/'tests/sim/stubs'),'-I'+str(ROOT/'Application'),'-I'+str(ROOT/'Common'),'-I'+str(ROOT/'Drivers'),HERE/f'{name}_audit.c','-o',exe])
        results[name]=execute([exe])
    print('Undefined-behavior checks passed.',flush=True)
    results['nominal']={};results['sweep']={};results['waveforms_identical']={}
    for version,source in [('audit_before',HERE/'baseline'),('audit_after',ROOT)]:
        run.build(version,source)
        results['nominal'][version]={m:run.run(version,m) for m in ['coords','cycle','rapid','jitter','boot-low','boot-high','disturb','load','unload']}
        results['sweep'][version]=[]
        for i,(g,t,f,d) in enumerate(itertools.product([.7,1,1.3],[.7,1,1.3],[1,1.5],[1,4])):
            r=run.run(version,'cycle',(g,t,f,9,d),f'_sweep{i}')
            r['params']=[g,t,f,d];results['sweep'][version].append(r)
        print(f'{version}: nominal and 36 plant variants complete.',flush=True)
    for m in ['coords','cycle','rapid','jitter','boot-low','boot-high','disturb','load','unload']:
        results['waveforms_identical'][m]=(run.HERE/'results'/f'audit_before_{m}.csv').read_bytes()==(run.HERE/'results'/f'audit_after_{m}.csv').read_bytes()
    for r in results['sweep']['audit_after']:
        assert abs(r['final_pos']-20250)<100
        assert r['tail_nonzero_pwm']==0
    for m in ['cycle','jitter','boot-low','boot-high','disturb','unload']:
        assert results['nominal']['audit_after'][m]['tail_nonzero_pwm']==0,m
    results['unchanged_files']={}
    for folder in ['Application','Common','Drivers','System','User']:
        for before in (HERE/'baseline'/folder).glob('*'):
            if before.is_file():
                rel=before.relative_to(HERE/'baseline');after=ROOT/rel
                if after.exists() and before.read_bytes()!=after.read_bytes(): print(f'Changed: {rel}',flush=True)
    for rel in ['Application/A_Parameter.h','System/iap.c','System/iap.h']:
        assert (ROOT/rel).read_bytes()==(HERE/'baseline'/rel).read_bytes(),rel
        results['unchanged_files'][rel]=hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()
    (OUT/'verification.json').write_text(json.dumps(results,indent=2))
    print(json.dumps({'planner':results['planner'],'protect':results['protect'],'waveforms_identical':results['waveforms_identical']},indent=2))
