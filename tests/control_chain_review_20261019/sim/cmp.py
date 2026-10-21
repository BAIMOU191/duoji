import sys,re
def load(f):
    out={}
    for line in open(f):
        if line.startswith('CAL'): continue
        p=line.strip().split(',')
        d={}
        for kv in p[3:]:
            k,val=kv.split('='); d[k]=float(val)
        out[(p[1],p[2])]=d
    return out
v=sys.argv[1]; cols=sys.argv[2].split(','); filt=sys.argv[3] if len(sys.argv)>3 else None
sets=[('cur',f'm{v}.txt'),('J4',f'p0{v}.txt'),('J4+id',f'pid{v}.txt'),('Jtrue',f'pj{v}.txt')]
D={n:load(f) for n,f in sets}
print(f'== {v}kg   每格: 现状 / 方案(J=4固定) / 方案(J初值4+在线辨识) / 方案(J已知)')
print('load|cmd'.ljust(34),' | '.join(f'{c:>26}' for c in cols))
for k in D['cur']:
    if filt and not re.search(filt,'|'.join(k)): continue
    row=[]
    for c in cols:
        row.append('/'.join(f"{D[n][k][c]:g}" for n,_ in sets))
    print(('|'.join(k)).ljust(34),' | '.join(f'{x:>26}' for x in row))
