from pathlib import Path
import importlib.util,json,hashlib,subprocess,csv
import numpy as np
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=HERE/'output'
spec=importlib.util.spec_from_file_location('support',ROOT/'tests/sync_noncontrol_adc_20261019/verify.py')
s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s);s.OUT=OUT

def stats(data,mask):
    keys=data.dtype.names
    selected=data[mask]
    if len(selected)<2:return {}
    pwm=data['pwm'];du=np.diff(pwm,prepend=pwm[0]);idx=np.flatnonzero(mask)
    high=pwm-np.convolve(np.pad(pwm,(5,5),mode='edge'),np.ones(11)/11,mode='valid')
    ix=idx[np.argmax(np.abs(du[mask]))]
    return {'n':len(selected),'pwm_min':int(selected['pwm'].min()),'pwm_max':int(selected['pwm'].max()),'pwm_pp_pct':float(np.ptp(selected['pwm'])/24),'max_step_ticks':int(abs(du[ix])),'max_step_pct':float(abs(du[ix])/24),'step_p95_pct':float(np.percentile(abs(du[mask]),95)/24),'hf_rms_pct':float(np.sqrt(np.mean(high[mask]**2))/24),'max_step_t':int(data['t'][ix]),'maxstep_before':{k:float(data[k][max(0,ix-1)]) for k in keys},'maxstep_after':{k:float(data[k][ix]) for k in keys},'fault':int(selected['fault'].max())}

def analyze(path):
    a=np.genfromtxt(path,delimiter=',',names=True)
    windows={'all':np.ones(len(a),dtype=bool),'move':a['state']==0,'cruise':(a['state']==0)&(abs(a['refa'])<1)&(abs(a['refv'])>30),'cruise250_450':(a['t']>=250)&(a['t']<=450)&(abs(a['refa'])<1)&(abs(a['refv'])>30),'settle':a['state']==1,'hold':(a['state']==2)&(a['t']>300),'last500':a['t']>a['t'][-1]-500}
    return {name:stats(a,mask) for name,mask in windows.items()}

def main():
    (OUT/'20').mkdir(parents=True,exist_ok=True)
    manifest={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for p in ('CH32V00X_APP_ADC_15kg','CH32V00X_APP_ADC_20kg','CH32V00X_APP_ADC_30kg') for f in (ROOT/p).rglob('*') if f.is_file() and f.suffix in ('.c','.h') and 'obj' not in f.parts}
    (HERE/'source_hashes.json').write_text(json.dumps(manifest,indent=2))
    scenarios=[(mode,T,noise,0) for mode in range(10) for T in ((0,1000) if mode not in (5,6) else (0,)) for noise in (0,1,2,5)]
    scenarios += [(mode,T,noise,diag) for mode,T in ((0,0),(0,1000),(3,1000),(5,0),(8,0)) for noise in (0,1,2) for diag in (1,2,3,4,5)]
    results={}
    for label,project in (('before',ROOT/'tests/replan_20kg_impl_20261019/before'),('after',ROOT/'CH32V00X_APP_ADC_20kg')):
        exe=s.compile_host(label,HERE/'check.c',project,20)
        for mode,T,noise,diag in scenarios:
            name=f'{label}_m{mode}_T{T}_n{noise}_d{diag}'
            trace=OUT/(name+'.csv')
            p=subprocess.run([str(exe),str(mode),str(T),str(noise),str(diag),str(trace)],capture_output=True,text=True)
            assert p.returncode==0,p.stderr
            results[name]=analyze(trace)
    (OUT/'results.json').write_text(json.dumps(results,indent=2))
    assert all(hashlib.sha256((ROOT/f).read_bytes()).hexdigest()==h for f,h in manifest.items())
    assert all(v['all']['fault']==0 for v in results.values())
    for name,r in results.items():
        if name.startswith('after') and '_d0' in name and '_n1_' in name:print(name,{w:{k:v for k,v in q.items() if k in ('max_step_pct','step_p95_pct','hf_rms_pct','pwm_pp_pct','max_step_t')} for w,q in r.items()})
    t0=[k for k in results if k.startswith('after') and '_T0_' in k]
    assert all((OUT/(k+'.csv')).read_bytes()==(OUT/(k.replace('after_','before_')+'.csv')).read_bytes() for k in t0)
    print('Production unchanged; cases:',len(results))
if __name__=='__main__':main()
