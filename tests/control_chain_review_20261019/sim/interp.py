import math
# 插补通道对照：目标零阶保持 -> 两级宽度=Δ 的滑动平均(三角核，等价线性插补再平滑一次)，看与理想指令的最大偏差和参考加速度
for amp,hz,iv in [(3000,0.5,20),(3000,1,20),(3000,2,20),(1500,3,20),(3000,1,10),(3000,0.5,100)]:
    N=8000; cmd=[0]*N; zoh=[0]*N; last=13500
    for t in range(N):
        c=13500+amp*math.sin(2*math.pi*hz*t/1000); cmd[t]=c
        if t%iv==0: last=c
        zoh[t]=last
    def box(x,w):
        out=[0]*len(x); s=0
        for i in range(len(x)):
            s+=x[i]; 
            if i>=w: s-=x[i-w]
            out[i]=s/min(i+1,w)
        return out
    r=box(box(zoh,iv),iv)
    lag=max(abs(r[t]-cmd[t]) for t in range(1000,6000))
    print(f"sine {amp/100:.0f}deg {hz}Hz iv={iv}ms : 插补参考与指令最大偏差 {lag/100:.2f} deg (纯滞后约 {iv} ms)")
