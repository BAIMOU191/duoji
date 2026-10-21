from pathlib import Path
import importlib.util,json,hashlib,subprocess
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=HERE/'output'
spec=importlib.util.spec_from_file_location('support',ROOT/'tests/sync_noncontrol_adc_20261019/verify.py')
s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s);s.OUT=OUT
def main():
    (OUT/'20').mkdir(parents=True,exist_ok=True)
    projects={'before':ROOT/'tests/replan_20kg_impl_20261019/before','after':ROOT/'CH32V00X_APP_ADC_20kg'}
    manifest={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for p in ('CH32V00X_APP_ADC_15kg','CH32V00X_APP_ADC_20kg','CH32V00X_APP_ADC_30kg') for f in (ROOT/p).rglob('*') if f.is_file() and f.suffix in ('.c','.h') and 'obj' not in f.parts}
    (HERE/'source_hashes.json').write_text(json.dumps(manifest,indent=2))
    rows=[]
    for label,project in projects.items():
        exe=s.compile_host(label,HERE/'check.c',project,20)
        cases=[(step,T,0,1) for step in (-12000,-9000,-4500,-1000,-500,-100,100,500,1000,3000,4500,9000,12000) for T in (0,1000)]
        if label in ('before','after'):
            cases += [(step,0,diag,factor) for step in (1000,4500,9000) for diag,factors in ((1,[1]),(2,[.7,1.1,1.2,1.3]),(3,[.6,1.5]),(4,[.5,1.5]),(5,[.85,1.05,1.1,1.15]),(6,[6400,8400]),(7,[1]),(8,[1]),(9,[1.3]),(10,[1.15]),(11,[1]),(12,[1.3]),(13,[1.15]),(14,[1]),(15,[1.3])) for factor in factors]
        for step,T,diag,factor in cases:
            name=f'{label}_{step}_{T}_{diag}_{factor:g}'
            trace=OUT/(name+'.csv')
            p=subprocess.run([str(exe),str(step),str(T),str(diag),str(factor),str(trace)],capture_output=True,text=True)
            assert p.returncode==0,p.stderr
            row={k:float(v) for k,v in (item.split('=') for item in p.stdout.strip().split(','))};row['version']=label;row['trace']=str(trace)
            rows.append(row)
    (OUT/'results.json').write_text(json.dumps(rows,indent=2))
    for r in rows:
        if r['version']=='after' and r['T']==0 and r['step']==9000:print(r)
    old={(r['step'],r['T'],r['diag'],r['factor']):r for r in rows if r['version']=='before'}
    new={(r['step'],r['T'],r['diag'],r['factor']):r for r in rows if r['version']=='after'}
    assert old.keys()==new.keys()
    assert all(Path(old[k]['trace']).read_bytes()==Path(new[k]['trace']).read_bytes() for k in old)
    assert all(r['fault']==0 and r['hard']==0 for r in rows)
    assert all(hashlib.sha256((ROOT/f).read_bytes()).hexdigest()==h for f,h in manifest.items())
    print('Production sources unchanged; cases:',len(rows))
if __name__=='__main__':main()
