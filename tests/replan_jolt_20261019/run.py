from pathlib import Path
import subprocess,json,sys,hashlib
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=HERE/'output'
CC=Path(r'D:\SoftWare\mingw64\bin\gcc.exe')
PROFILES=('base','merge10','c2','c2_merge10')

def command(args,log):
    p=subprocess.run(list(map(str,args)),capture_output=True,text=True,encoding='utf-8',errors='replace')
    log.parent.mkdir(parents=True,exist_ok=True);log.write_text(p.stdout+p.stderr,encoding='utf-8')
    if p.returncode:raise RuntimeError(str(log)+'\n'+p.stdout[-2000:]+p.stderr[-3000:])
    return p.stdout

def compile(kg,name):
    p=HERE/'profiles'/f'{kg}_{name}'
    special='c2' in name or 'segments' in name
    if special:
        f=p/'Common/C_Traj_Planner.c';original=(HERE/'profiles'/f'{kg}_base/Common/C_Traj_Planner.c').read_text(encoding='utf-8')
        mapping='\n'.join('#define C_Traj_'+a+' Legacy_'+b for a,b in (('Plan','Plan'),('Step','Step'),('Stop','Stop'),('Hold','Hold'),('Is_Done','Done')))
        f.write_text('#include <stdint.h>\nuint8_t Legacy_Done(void);\n'+mapping+'\n'+original+'\n'+(HERE/('segments.c' if 'segments' in name else 'prototype.c')).read_text(encoding='utf-8'),encoding='utf-8')
    inc=['-I'+str(ROOT/'tests/stubs'),'-I'+str(HERE)]
    inc+=['-I'+str(p/d) for d in ('Application','Common','Drivers','System','SRC/Debug')]
    exe=OUT/f'{kg}_{name}.exe'
    command([CC,'-std=gnu99','-O2','-Wall','-Wextra','-Wl,--image-base,0x400000',*inc,*(['-DC2_TEST'] if special else []),HERE/'jolt.c','-lm','-o',exe],exe.with_suffix('.compile.log'))
    return exe

def parse(output):
    rows=[]
    for line in output.splitlines():
        if not line.startswith('CASE,'):continue
        d={k:float(v) for k,v in (s.split('=') for s in line.split(',')[1:])}
        assert all(__import__('math').isfinite(v) for v in d.values()),d
        rows.append(d)
    assert len(rows)==486,len(rows)
    assert all(r['hard']==0 for r in rows)
    return rows

if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8');OUT.mkdir(exist_ok=True)
    kg=int(sys.argv[1]);names=sys.argv[2:] or PROFILES
    for name in names:
        exe=compile(kg,name)
        rows=parse(command([exe],exe.with_suffix('.run.log')))
        (OUT/f'{kg}_{name}.json').write_text(json.dumps(rows,indent=2))
        print(kg,name,len(rows),'faults',sum(r['fault'] for r in rows),'fallbacks',sum(r['fallback'] for r in rows),flush=True)
    before=json.loads((HERE/'source_hashes_before.json').read_text())
    assert all(hashlib.sha256((ROOT/n).read_bytes()).hexdigest()==sha for n,sha in before.items())
    print('PASS production sources unchanged',flush=True)
