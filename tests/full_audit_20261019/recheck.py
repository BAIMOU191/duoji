from pathlib import Path
import subprocess,json,hashlib
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
OUT=HERE/'recheck_output'
CC=Path(r'D:\SoftWare\mingw64\bin\gcc.exe')
def run(cmd,log):
    r=subprocess.run(list(map(str,cmd)),cwd=ROOT,capture_output=True,text=True,encoding='utf-8',errors='replace')
    log.write_text(r.stdout+r.stderr,encoding='utf-8')
    if r.returncode:raise RuntimeError(str(log)+'\n'+r.stdout+r.stderr)
    return r.stdout
def hashes():
    original=json.loads((HERE/'source_hashes.json').read_text())
    for project,files in original.items():
        for rel,h in files.items():assert hashlib.sha256((ROOT/project/rel).read_bytes()).hexdigest()==h,(project,rel)
if __name__=='__main__':
    OUT.mkdir(exist_ok=True);hashes()
    evidence={}
    for kg in (15,20,30):
        project=ROOT/f'CH32V00X_APP_ADC_{kg}kg'
        inc=['-I'+str(ROOT/'tests/stubs')]+['-I'+str(project/d) for d in ('Application','Common','Drivers','System','SRC/Debug')]
        common=[CC,'-std=gnu99','-O2','-Wall','-Wextra','-Wl,--image-base,0x400000']
        for candidate in (False,True):
            stem=f'{kg}kg'+('_candidate_n1' if candidate else '_production')
            if candidate:
                dest=OUT/f'candidate_{kg}kg';dest.mkdir(exist_ok=True)
                text=(project/'Common/C_Traj_Planner.c').read_text(encoding='utf-8')
                assert text.count('if (n < 2) return;')==1
                (dest/'C_Traj_Planner.c').write_text(text.replace('if (n < 2) return;','if (n < 1) return;'),encoding='utf-8')
                extra=['-DCANDIDATE_N1','-I'+str(dest)]
            else:extra=[]
            exe=OUT/(stem+'.exe')
            run([*common,*extra,*inc,HERE/'recheck.c','-lm','-o',exe],OUT/(stem+'_compile.log'))
            result=run([exe],OUT/(stem+'_results.txt'))
            rows=[]
            for line in result.splitlines():
                if not line.startswith('RECHECK,'):continue
                fields=line.split(',');row={'probe':fields[1]}
                for field in fields[2:]:
                    key,value=field.split('=',1)
                    try:value=float(value) if '.' in value else int(value)
                    except ValueError:pass
                    row[key]=value
                rows.append(row)
            evidence[stem]=rows
            print(stem+'\n'+result,flush=True)
    (OUT/'evidence.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf-8')
    hashes();print('PASS: 288 production source hashes unchanged',flush=True)
