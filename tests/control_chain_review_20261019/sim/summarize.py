import sys,re
def load(f):
    out={}
    for line in open(f):
        if line.startswith('CAL'): continue
        p=line.strip().split(',')
        v,ld,cm=p[:3]; d={}
        for kv in p[3:]:
            k,val=kv.split('='); d[k]=float(val)
        out[(ld,cm)]=d
    return out
vs={v:load(f'm{v}.txt') for v in (15,20,30)}
keys=list(vs[30].keys())
cols=sys.argv[1].split(',') if len(sys.argv)>1 else ['track','os','ring','settle','final','stops','sat','freeze']
filt=sys.argv[2] if len(sys.argv)>2 else None
print('load|cmd'.ljust(36),' | '.join(f'{c:>22}' for c in cols))
for k in keys:
    if filt and not re.search(filt,'|'.join(k)): continue
    row=[]
    for c in cols:
        row.append('/'.join(f"{vs[v][k][c]:g}" for v in (15,20,30)))
    print(('|'.join(k)).ljust(36),' | '.join(f'{x:>22}' for x in row))
