import statistics as st
def load(f):
    out={}
    for line in open(f):
        if line.startswith('CAL'): continue
        p=line.strip().split(','); d={}
        for kv in p[3:]:
            k,val=kv.split('='); d[k]=float(val)
        out[(p[1],p[2])]=d
    return out
loads_loaded=['inertia2','inertia4','inertia8','grav20%_r4','grav40%_r6','grav60%_r8','stepload+3V','stepload-3V','randload','grav30%_r4_bl0.6','grav30%_r4_6.4V','push_r4','push_r8','push_r4_bl']
pm=['bigstep','reverse400','reverse_arrive','timed90','hold']
for v in (15,20,30):
    print(f'== {v}kg  点到点指令(大阶跃/中途换向/到位换向/带时间/到位后推) × 14种负载 = {len(loads_loaded)*len(pm)} 例')
    for name,f in [('现状',f'm{v}.txt'),('方案 J=4固定',f'p0{v}.txt'),('方案 J初值4+在线辨识',f'pid{v}.txt'),('方案 J已知',f'pj{v}.txt')]:
        D=load(f); os_=[];ring=0;out=0;flt=0;stall=0;tr=[]
        for l in loads_loaded:
            for c in pm:
                d=D[(l,c)]
                if l=='grav60%_r8' and c in('bigstep','reverse400'): pass
                os_.append(d['os']); ring+=d['ring']; tr.append(d['track'])
                if abs(d['final'])>0.5: out+=1
                flt+=d['fault']; stall+=d['stall']
        os_s=sorted(os_)
        print(f'  {name:<22} 过冲 中位 {st.median(os_):5.2f}° / 90%分位 {os_s[int(.9*len(os_s))]:5.2f}° / 最大 {max(os_):6.2f}° | 来回摆次数合计 {ring:4.0f} | 终点超出±0.5° {out:2d} 例 | 过流卸力 {flt:.0f} 次 | 堵转放弃 {stall:.0f} 次')
    # no-load penalty
    for name,f in [('现状',f'm{v}.txt'),('方案 J=4固定',f'p0{v}.txt'),('方案 J已知',f'pj{v}.txt')]:
        D=load(f); print(f'  空载 {name:<12}', ' '.join(f"{c}:os{D[('noload',c)]['os']:.2f}/末{D[('noload',c)]['final']:.2f}" for c in pm))
