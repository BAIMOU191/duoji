import sys,pathlib,subprocess,json,itertools
ROOT=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tests/audit'))
from run_audit import compile_host
HERE=ROOT/'tests/pause'
report={}
for version,source in [('before',HERE/'baseline'),('after',ROOT)]:
    exe=compile_host('pause_'+version,source,HERE/'pause_sim.c',version=='after')
    rows=[]
    for direction,before,wait,timed,new_command in itertools.product([1,-1],[50,100,180,300],[20,100,500,2000],[0,1500],[0,1]):
        r=subprocess.run([str(exe),*map(str,[direction,before,wait,timed,new_command])],capture_output=True,text=True,check=True,timeout=15)
        row=json.loads(r.stdout);rows.append(row)
        if version=='after':
            assert abs(row['start_ref']-row['resume_pos'])<=1,row
            assert abs(row['final_pos']-(26800 if direction>0 else 200))<80,row
            assert row['reverse_cdeg']<1,row
    report[version]=rows
    print(version,'cases',len(rows),'maximum reverse degrees',max(r['reverse_cdeg'] for r in rows)/100,flush=True)
(HERE/'results/verification.json').write_text(json.dumps(report,indent=2))
