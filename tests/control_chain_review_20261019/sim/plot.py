import csv, math, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif']=['Microsoft YaHei','SimHei']; plt.rcParams['axes.unicode_minus']=False
OUT=sys.argv[1]
def rd(f):
    R=list(csv.DictReader(open(f)))
    return {k:[float(r[k]) for r in R] for k in R[0] if k!='case'}
C1='#2a6fdb'; C2='#e0662b'; G='#888'
# F1 惯量负载大动作
a=rd('fig/f1_cur.csv'); b=rd('fig/f1_pro.csv')
fig,ax=plt.subplots(3,1,figsize=(10,8),sharex=True)
for d,c,l in [(a,C1,'现状'),(b,C2,'方案原型(惯量比J=4、扰动补偿、能力规划)')]:
    ax[0].plot(d['t'],[(x-tg)/100 for x,tg in zip(d['x'],d['target'])],c,lw=1,label=l)
    ax[1].plot(d['t'],[u/1000 for u in d['u']],c,lw=.8)
    ax[2].plot(d['t'],d['state'],c,lw=1)
ax[0].set_ylim(-8,8); ax[0].axhline(.5,color=G,ls=':'); ax[0].axhline(-.5,color=G,ls=':')
ax[0].set_ylabel('实际 − 目标 (°)'); ax[0].legend(loc='upper right'); ax[0].set_title('30kg，负载惯量=4倍电机，+90°后−180°(T=0)：末端过冲与来回摆')
ax[1].set_ylabel('输出电压 (V)'); ax[2].set_ylabel('控制器状态\n0运动 1收尾 2保持'); ax[2].set_xlabel('时间 (ms)')
ax[0].set_xlim(2400,5000)
plt.tight_layout(); plt.savefig(OUT+'/01_惯量负载_过冲振荡.png',dpi=110); plt.close()
# F2 重载换向 过流卸力
a=rd('fig/f2_cur.csv'); b=rd('fig/f2_pro.csv')
fig,ax=plt.subplots(4,1,figsize=(10,9),sharex=True)
for d,c,l in [(a,C1,'现状'),(b,C2,'方案原型(能力规划+运动中可行性重规划+模型电流钳位)')]:
    ax[0].plot(d['t'],[x/100 for x in d['x']],c,lw=1,label=l)
    ax[1].plot(d['t'],[i/1000 for i in d['i']],c,lw=.8)
    ax[2].plot(d['t'],d['lim'],c,lw=1)
    ax[3].plot(d['t'],[1 if tq>0 else 0 for tq in d['torque']],c,lw=1.2)
ax[0].plot(a['t'],[x/100 for x in a['target']],G,lw=.8,ls='--',label='目标')
ax[0].set_ylabel('角度 (°)'); ax[0].legend(loc='upper right',fontsize=8)
ax[0].set_title('20kg，重力40%标定电压+惯量6倍，180°每400ms换向：冲过目标→反接制动电流>3A→过流保护卸力')
ax[1].axhline(3,color='r',ls=':'); ax[1].axhline(2,color=G,ls=':'); ax[1].set_ylabel('绕组电流 (A)\n(假设R=3.2Ω)')
ax[2].set_ylabel('保护给的PWM上限'); ax[3].set_ylabel('卸力(1=已卸力)'); ax[3].set_xlabel('时间 (ms)'); ax[0].set_xlim(0,5500)
plt.tight_layout(); plt.savefig(OUT+'/02_重载换向_过流卸力.png',dpi=110); plt.close()
# F3 流式 200ms T200
a=rd('fig/f3_cur.csv'); b=rd('fig/f3_pro.csv')
fig,ax=plt.subplots(2,1,figsize=(10,6),sharex=True)
ax[0].plot(a['t'],[v/100 for v in a['refv']],C1,lw=1,label='现状 参考速度'); ax[0].plot(a['t'],[v/100 for v in a['vf']],C1,lw=.6,alpha=.5,label='现状 实际速度')
ax[1].plot(b['t'],[v/100 for v in b['refv']],C2,lw=1,label='方案 参考速度(规划时间≥2×指令间隔)'); ax[1].plot(b['t'],[v/100 for v in b['vf']],C2,lw=.6,alpha=.5,label='方案 实际速度')
for x in ax: x.axhline(20,color=G,ls=':'); x.set_ylabel('速度 (°/s)'); x.legend(loc='upper right',fontsize=8); x.set_xlim(1000,2600)
ax[0].set_title('30kg 空载，总线每200ms发一个+4°目标、T=200(期望20°/s匀速)：每个目标都停一下')
ax[1].set_xlabel('时间 (ms)')
plt.tight_layout(); plt.savefig(OUT+'/03_流式指令_一段一段.png',dpi=110); plt.close()
# F4 规划器跟正弦流
d=rd('fig/f4.csv'); iv=20
zoh=[];last=13500
for t,c in zip(d['t'],d['cmd']):
    if int(t)%iv==0: last=c
    zoh.append(last)
def box(x,w):
    out=[];s=0
    for i in range(len(x)):
        s+=x[i]; 
        if i>=w: s-=x[i-w]
        out.append(s/min(i+1,w))
    return out
ip=box(box(zoh,iv),iv)
fig,ax=plt.subplots(1,1,figsize=(10,4))
ax.plot(d['t'],[c/100 for c in d['cmd']],G,lw=1,label='上位机指令(±30°，1Hz，每20ms一帧，T=20)')
ax.plot(d['t'],[r/100 for r in d['ref']],C1,lw=1.2,label='现状规划器参考(逐点停到目标+最短运动时间150ms)')
ax.plot(d['t'],[r/100 for r in ip],C2,lw=1.2,label='插补通道参考(相邻目标插补，约1.5帧延迟)')
ax.set_xlim(1000,4000); ax.set_ylabel('角度 (°)'); ax.set_xlabel('时间 (ms)'); ax.legend(fontsize=8,loc='upper right')
ax.set_title('规划器单独测试(30kg 默认 V/A/D/平滑/Tm)：连续目标流的滞后与幅值失真')
plt.tight_layout(); plt.savefig(OUT+'/04_连续指令_规划器滞后.png',dpi=110); plt.close()
# F5 假托举力蠕出窗口
a=rd('fig/f5_cur.csv')
fig,ax=plt.subplots(3,1,figsize=(10,7),sharex=True)
ax[0].plot(a['t'],[(x-tg)/100 for x,tg in zip(a['x'],a['target'])],C1,lw=1); ax[0].axhline(.5,color=G,ls=':'); ax[0].axhline(-.5,color=G,ls=':')
ax[0].set_ylim(-1.5,1.5); ax[0].set_ylabel('实际 − 目标 (°)')
ax[0].set_title('30kg 空载、实际反电势系数比标定小15%：运动中积分吃进模型误差，到位后被冻结成"托举力"，轴蠕出到位窗')
ax[1].plot(a['t'],a['ui'],C1,lw=1); ax[1].set_ylabel('积分 (mV)')
ax[2].plot(a['t'],a['state'],C1,lw=1); ax[2].set_ylabel('状态 0/1/2'); ax[2].set_xlabel('时间 (ms)')
ax[0].set_xlim(3600,5000)
plt.tight_layout(); plt.savefig(OUT+'/05_到位后蠕出窗口.png',dpi=110); plt.close()
# F6 低速粘滑
a=rd('fig/f6_cur.csv'); b=rd('fig/f6_dit.csv')
fig,ax=plt.subplots(2,1,figsize=(10,6),sharex=True)
ax[0].plot(a['t'],[v/100 for v in a['v']],C1,lw=.6,label='现状 实际速度'); ax[0].plot(a['t'],[v/100 for v in a['refv']],'k',lw=1,label='参考速度')
ax[1].plot(b['t'],[v/100 for v in b['v']],C2,lw=.6,label='方案试验：低速段叠加 ±fs 250Hz 颤振'); ax[1].plot(b['t'],[v/100 for v in b['refv']],'k',lw=1)
for x in ax: x.set_ylim(-2,6); x.set_ylabel('速度 (°/s)'); x.legend(loc='upper right',fontsize=8); x.set_xlim(4000,9000)
ax[0].set_title('15kg 空载，20°用20s(约1°/s)：低于约3°/s时粘滑，一走一停'); ax[1].set_xlabel('时间 (ms)')
plt.tight_layout(); plt.savefig(OUT+'/06_低速粘滑.png',dpi=110); plt.close()
print('done')
