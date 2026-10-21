from pathlib import Path
import importlib.util,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=HERE/'output'
plt.rcParams['font.sans-serif']=['Microsoft YaHei'];plt.rcParams['axes.unicode_minus']=False
SELECT='m1_r0'
data=json.loads((OUT/'pwm.json').read_text())
matrix=json.loads((OUT/'matrix.json').read_text())
verification=json.loads((OUT/'verification.json').read_text())
def load(name,mode,T,noise=1):return np.genfromtxt(OUT/f'{name}_m{mode}_T{T}_n{noise}.csv',delimiter=',',names=True)
summary={'noise_sigma_deg':.01,'motion_max_step_unit':'percentage points of full PWM duty, abs(PWM[t]-PWM[t-1])/2400*100','comparisons':[],'scope':'20kg C_Pos_Ctrl.c and header only; V/A/D/J, planner, observer and protocol unchanged.'}
for mode,T,label in [(9,0,'0.1deg request T0'),(8,0,'0.3deg request T0'),(0,0,'90deg T0'),(0,1000,'90deg T1000'),(3,1000,'slider every50ms T1000'),(4,0,'reverse every400ms T0'),(5,0,'20deg T10000 first4.5s')]:
    old=data['base'][f'base_m{mode}_T{T}_n1'];new=data[SELECT][f'{SELECT}_m{mode}_T{T}_n1']
    a=load('base',mode,T);b=load(SELECT,mode,T)
    metrics={'scenario':label,'before_move_max_step':old['move']['max_step_pct'],'after_move_max_step':new['move']['max_step_pct'],'before_move_hf_rms':old['move']['hf_rms_pct'],'after_move_hf_rms':new['move']['hf_rms_pct'],'before_settle_max_step':old['settle'].get('max_step_pct'),'after_settle_max_step':new['settle'].get('max_step_pct')}
    for tag,x in [('before',a),('after',b)]:
        idx=np.flatnonzero(np.diff(x['target'],prepend=x['target'][0]));lastcmd=x['t'][idx[-1]] if len(idx) else 0
        arr=x[(x['t']>=lastcmd)&(x['arrived']==1)]['t']
        metrics[tag+'_arrival_after_last_cmd_ms']=float(arr[0]-lastcmd) if len(arr) else None
        metrics[tag+'_final_error_deg']=float(x['x'][-1]-x['target'][-1])
    summary['comparisons'].append(metrics)
summary['cruise250_450_ms']={k:data[p][f'{p}_m0_T0_n1']['cruise250_450'] for k,p in [('before','base'),('after',SELECT)]}
summary['matrix']={label:{'cases':len(rows),'fault_events':sum(r['fault'] for r in rows.values()),'final_outside_0_5_deg':sum(abs(r['final'])>.5 for r in rows.values())} for label,rows in [('before',matrix['base']),('after',matrix[SELECT])]}
summary['retarget']={label:{'cases':486,'fault_events':sum(r['fault'] for r in rows['jolt']),'final_outside_0_5_deg':sum(abs(r['final'])>.5 for r in rows['jolt'])} for label,rows in verification.items()}
summary['limitations']=['Final PWM duty at 1ms is simulated; PWM switching edges, EMI and task execution latency require bench validation.','Stress matrix still contains pre-existing overload/protection and accuracy failures; no new faulty cases or newly outside 0.5deg cases relative to baseline.','SETTLE individual max steps may increase; damping and integral retain raw observer feedback to avoid precision regressions.','HOLD now returns to active settling after drifting outside precision window, including after external displacement.']
(OUT/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')

fig,axs=plt.subplots(2,2,figsize=(12,8))
colors={'base':'C1',SELECT:'C0'}
for name,label in [('base','优化前'),(SELECT,'优化后')]:
    x=load(name,0,0);w=(x['t']>=250)&(x['t']<=450)
    axs[0,0].plot(x['t'][w],x['pwm'][w]/24,label=label,color=colors[name],alpha=.8)
    x=load(name,9,0);w=(x['t']>=30)&(x['t']<=130)
    axs[0,1].plot(x['t'][w],x['pwm'][w]/24,label=label,color=colors[name],alpha=.85)
    x=load(name,3,1000);w=x['t']<=3000
    axs[1,0].plot(x['t'][w],x['pwm'][w]/24,label=label,color=colors[name],alpha=.75,lw=.8)
    x=load(name,0,0);w=x['t']<=750
    axs[1,1].plot(x['t'][w],x['v'][w],label=label,color=colors[name])
axs[0,0].set_title('90° T=0匀速段：相邻1ms最大变化 1.67 → 1.13个百分点')
axs[0,1].set_title('0.1°请求 T=0：运动阶段最大变化 6.21 → 2.33个百分点')
axs[1,0].set_title('每50ms新目标、每条T=1000：最大变化 4.92 → 1.83个百分点')
axs[1,1].set_title('90° T=0实际速度：到位时间均为637ms')
for ax in axs.flat:ax.grid(alpha=.25);ax.legend();ax.set_xlabel('指令后时间 (ms)')
for ax in [axs[0,0],axs[0,1],axs[1,0]]:ax.set_ylabel('最终占空比 (%)')
axs[1,1].set_ylabel('物理模型实际速度 (°/s)')
fig.suptitle('20kg空载PWM优化前后：仅速度残差轻度滤波、起转补偿升降及保持漂移退出\n当前标定参数，位置噪声σ=0.01°；轨迹最大速度与加减速度保持不变。',fontsize=11)
fig.tight_layout();fig.savefig(OUT/'comparison.png',dpi=150)
print('PASS comparison figure and numeric summary generated')
for r in summary['comparisons']:print(r['scenario'],round(r['before_move_max_step'],3),round(r['after_move_max_step'],3),r['before_arrival_after_last_cmd_ms'],r['after_arrival_after_last_cmd_ms'])
