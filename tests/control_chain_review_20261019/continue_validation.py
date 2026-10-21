"""Resume Claude's conservative-package validation; isolated copies, fail-fast commands."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import subprocess,hashlib,json,shutil,importlib.util,sys,re,time
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
SRC=HERE/'sim_conservative'
OUT=HERE/'continued_validation'
CC=Path(r'D:\SoftWare\mingw64\bin\gcc.exe')
PACKAGES={
 'base0':{},
 'A0':dict(CX_EMF_PCT=100,CX_STREAM=1),
 'T1':dict(CX_STILL_MS=200,CX_EMF_PCT=100,CX_STREAM=1),
 'T2':dict(CX_STILL_MS=200,CX_EMF_PCT=100,CX_STREAM=1,CX_CORR=1,CX_J_X10=20,CX_J_FF=0),
 'T3':dict(CX_STILL_MS=200,CX_EMF_PCT=100,CX_STREAM=1,CX_CORR=1,CX_J_X10=20,CX_J_FF=0,CX_DOB_BW=30,CX_DOB_GATE=1,CX_DOB_GATE_MV=100),
}
for j in (10,30,40):PACKAGES[f'T3_J{j}']={**PACKAGES['T3'],'CX_J_X10':j}
def execute(args,log,cwd=ROOT):
    p=subprocess.run(list(map(str,args)),cwd=cwd,capture_output=True,text=True,encoding='utf-8',errors='replace')
    log.parent.mkdir(parents=True,exist_ok=True);log.write_text(p.stdout+p.stderr,encoding='utf-8')
    if p.returncode:raise RuntimeError(f'{log}: exit={p.returncode}\n{p.stdout[-1500:]}\n{p.stderr[-3000:]}')
    return p.stdout
def parse(output):
    result={}
    for line in output.splitlines():
        if line.startswith('CAL,') or not line:continue
        p=line.split(',');assert len(p)>5,line
        metrics={}
        for v in p[3:]:
            k,x=v.split('=',1);metrics[k]=float(x)
        assert all(__import__('math').isfinite(x) for x in metrics.values()),line
        key='|'.join(p[1:3]);assert key not in result,key
        result[key]=metrics
    return result
def freeze():
    OUT.mkdir(exist_ok=True)
    dst=OUT/'source';dst.mkdir(exist_ok=True)
    shutil.copy2(SRC/'sim.c',dst/'sim.c');shutil.copytree(SRC/'cons',dst/'cons',dirs_exist_ok=True)
    hashes={}
    for kg in (15,20,30):
        p=ROOT/f'CH32V00X_APP_ADC_{kg}kg'
        hashes[p.name]={str(f.relative_to(p)):hashlib.sha256(f.read_bytes()).hexdigest()
          for d in ('Application','Common','Drivers','System','User','SRC') for f in (p/d).rglob('*') if f.suffix in ('.c','.h','.S','.ld')}
    (OUT/'production_hashes.json').write_text(json.dumps(hashes,indent=2),encoding='utf-8')
    (OUT/'packages.json').write_text(json.dumps(PACKAGES,indent=2),encoding='utf-8')
    return dst
def compile_host(src,kg,candidate,unit=False,instrumented=False):
    project=ROOT/f'CH32V00X_APP_ADC_{kg}kg'
    name=('unit_' if unit else 'inst_' if instrumented else '')+('candidate' if candidate else 'base')+str(kg)
    inc=['-I'+str(ROOT/'tests/stubs')]+(['-I'+str(src/'cons')] if candidate else [])
    inc+=['-I'+str(project/d) for d in ('Application','Common','Drivers','System','SRC/Debug')]
    exe=OUT/(name+'.exe')
    args=[CC,'-std=gnu99','-O2','-Wall','-Wextra','-Wl,--image-base,0x400000',*(['-DCONS'] if candidate else []),*inc]
    execute([*args,(src/'unit.c' if unit else src/'sim_instrumented.c' if instrumented else src/'sim.c'),'-lm','-o',exe],OUT/'logs'/(name+'_compile.log'))
    return exe
def matrix(src):
    exes={(kg,c):compile_host(src,kg,c) for kg in (15,20,30) for c in (False,True)}
    output={};comparison={}
    def job(item):
        name,kg=item;knobs=PACKAGES[name]
        raw=execute([exes[kg,name!='base0'],f'{kg}kg','matrix','NUL','*','*',*[f'{k}={v}' for k,v in knobs.items()]],OUT/'logs'/f'{name}_{kg}.txt')
        data=parse(raw);assert len(data)==351,(name,kg,len(data))
        archived=json.loads((SRC/'runs'/f'{name}.json').read_text())[str(kg)]
        differences=[]
        for k,d in data.items():
            for metric,value in d.items():
                if metric in archived[k] and value!=archived[k][metric]:differences.append((k,metric,archived[k][metric],value))
        return name,kg,data,differences
    with ThreadPoolExecutor(max_workers=3) as pool:
        for fut in as_completed([pool.submit(job,(name,kg)) for name in PACKAGES for kg in (15,20,30)]):
            name,kg,data,diff=fut.result();output.setdefault(name,{})[str(kg)]=data;comparison.setdefault(name,{})[str(kg)]=diff
            print(f'MATRIX {name} {kg}kg: {len(data)} cases; archived metric differences={len(diff)}',flush=True)
    (OUT/'matrix.json').write_text(json.dumps(output,indent=2),encoding='utf-8')
    (OUT/'archive_comparison.json').write_text(json.dumps(comparison,indent=2),encoding='utf-8')
    return exes
def instrument_source(src):
    s=(src/'sim.c').read_text(encoding='utf-8')
    s=s.replace('static Plant P;','static double audit_rw=3.2;\nstatic Plant P;')
    s=s.replace('p.rw = 3.2;','p.rw = audit_rw;')
    # Parse the assumed winding resistance before load cases are created.
    marker='    LoadCase loads[48]; int nl = 0;'
    s=s.replace(marker,'    for(int ai=6;ai<argc;ai++) if(!strncmp(argv[ai],"RW=",3)) audit_rw=atof(argv[ai]+3);\n'+marker)
    s=s.replace('r.arr_ms = -1; last_cmd_seen = g_motion_end;', 'r.arr_ms = -1; in_win=0; r.settle_ms=-1; last_cmd_seen = g_motion_end;')
    s=s.replace('r = run(nm, cmds[ci].s, cmds[ci].dur);','r = run(nm, cmds[ci].s, !strcmp(cmds[ci].name,"slowv") ? (int)ceil(20000/g_slowv)+2000 : cmds[ci].dur);')
    (src/'sim_instrumented.c').write_text(s,encoding='utf-8')
def extra(src):
    instrument_source(src)
    extra_results={}
    for kg in (15,20,30):
        unit=compile_host(src,kg,True,unit=True)
        extra_results[f'unit_{kg}']=execute([unit],OUT/'logs'/f'unit_{kg}.txt')
        for cand in (False,True):
            exe=compile_host(src,kg,cand,instrumented=True)
            package='T3' if cand else 'base0';knobs=PACKAGES[package]
            for speed in (.5,1,2,4):
                for load in ('noload','grav20%_r4'):
                    key=f'{package}_{kg}_slow{speed}_{load}'
                    extra_results[key]=parse(execute([exe,f'{kg}kg','matrix','NUL',load,'slowv',f'SLOWV={speed}',*[f'{k}={v}' for k,v in knobs.items()]],OUT/'logs'/(key+'.txt')))
            for rw in (1.6,2.4,3.2,4.8):
                for load in ('inertia8','grav40%_r6'):
                    key=f'{package}_{kg}_RW{rw}_{load}'
                    extra_results[key]=parse(execute([exe,f'{kg}kg','matrix','NUL',load,'reverse400',f'RW={rw}',*[f'{k}={v}' for k,v in knobs.items()]],OUT/'logs'/(key+'.txt')))
        print(f'EXTRA {kg}kg: boundary probes, complete slow moves, resistance sensitivity complete',flush=True)
    (OUT/'extra.json').write_text(json.dumps(extra_results,indent=2),encoding='utf-8')
def verify_hashes():
    changed=[]
    for p,files in json.loads((OUT/'production_hashes.json').read_text()).items():
        for rel,h in files.items():
            if hashlib.sha256((ROOT/p/rel).read_bytes()).hexdigest()!=h:changed.append(p+'/'+rel)
    (OUT/'source_changes_during_run.json').write_text(json.dumps(changed,indent=2),encoding='utf-8')
    print('Production changes during run:',changed,flush=True)
if __name__=='__main__':
    src=freeze() if '--extra-only' not in sys.argv else OUT/'source'
    shutil.copy2(HERE/'continuation_unit.c',src/'unit.c')
    if '--extra-only' not in sys.argv:matrix(src)
    extra(src);verify_hashes()
