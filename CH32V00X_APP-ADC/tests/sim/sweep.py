import itertools,json
from run import run,HERE
report={}
for version in ['baseline','fixed']:
    if not (HERE/'results'/f'{version}.exe').exists():continue
    results=[]
    for i,(g,t,f,d) in enumerate(itertools.product([.7,1,1.3],[.7,1,1.3],[1,1.5],[1,4])):
        r=run(version,'cycle',(g,t,f,9,d),f'_sweep{i}')
        r['params']=[g,t,f,d];results.append(r)
    report[version]=results
(HERE/'results'/'sweep.json').write_text(json.dumps(report,indent=2))
for v,rr in report.items():
    worst=sorted(rr,key=lambda r:max(m['post_done_travel'] for m in r['moves']),reverse=True)[:4]
    print(v,json.dumps(worst,indent=2))
