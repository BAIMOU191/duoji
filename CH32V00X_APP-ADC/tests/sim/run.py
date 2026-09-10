"""Compile actual C firmware algorithms and run repeatable host plant simulations."""
import csv, json, subprocess, pathlib, sys
ROOT=pathlib.Path(__file__).resolve().parents[2]
HERE=pathlib.Path(__file__).resolve().parent
def build(version, source=None):
    src=pathlib.Path(source) if source else HERE/'baseline' if version=='baseline' else ROOT
    exe=HERE/'results'/f'{version}.exe'
    args=['gcc','-std=c99','-O2','-Wall','-Wextra','-Wno-unused-parameter',
          '-I'+str(HERE/'stubs'),'-I'+str(src/'Application'),'-I'+str(ROOT/'Application'),
          '-I'+str(src/'Common'),'-I'+str(ROOT/'Common'),'-I'+str(ROOT/'Drivers'),str(HERE/'servo_sim.c'),
          str(src/'Application/A_Sensor.c')]
    args += [str(src/'Common'/f'{name}.c') for name in ['C_Pos_Ctrl','C_Speed_Observer','C_Traj_Planner']]
    subprocess.run(args+['-lm','-o',str(exe)],check=True)
    return exe
def run(version, mode, params=(), suffix=''):
    dest=HERE/'results'/f'{version}_{mode}{suffix}.csv'
    with dest.open('w') as f:subprocess.run([str(HERE/'results'/f'{version}.exe'),mode,*map(str,params)],stdout=f,check=True)
    if mode=='coords':return dest.read_text()
    with dest.open() as f:rows=[{k:float(v) for k,v in row.items()} for row in csv.DictReader(f)]
    tail=rows[-1000:]
    result={'final_pos':rows[-1]['pos'],'tail_motion_pp':max(r['pos'] for r in tail)-min(r['pos'] for r in tail),
            'tail_nonzero_pwm':sum(r['pwm']!=0 for r in tail),'tail_max_pwm':max(abs(r['pwm']) for r in tail),
            'escape_entries':sum(r['enc_state']==1 and (i==0 or rows[i-1]['enc_state']!=1) for i,r in enumerate(rows)),
            'max_abs_pwm':max(abs(r['pwm']) for r in rows)}
    if mode=='cycle':
        result['moves']=[]
        for start in range(0,12000,3000):
            block=rows[start:start+3000]
            reverse=len(params)>5 and params[5]==2
            target=(26800 if start//3000%2==0 else 6750) if reverse else ((0 if version=='magnetic' else 200) if start//3000%2==0 else 20250)
            direction=1 if target>block[0]['pos'] else -1
            done=next((i for i,r in enumerate(block) if r['done']),2999)
            last=block[-1000:]
            result['moves'].append({'done_ms':done,'overshoot':max(0,max(direction*(r['pos']-target) for r in block)),
              'post_done_travel':sum(abs(block[i]['pos']-block[i-1]['pos']) for i in range(done+1,len(block))),
              'final_error':block[-1]['pos']-target,'tail_pwm':sum(r['pwm']!=0 for r in last)})
    return result
if __name__=='__main__':
    versions=sys.argv[1:] or ['baseline','fixed'];report={}
    for v in versions:
        build(v);report[v]={m:run(v,m) for m in ['coords','cycle','rapid','jitter','boot-low','boot-high','disturb','load','unload']}
    (HERE/'results'/'summary.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
