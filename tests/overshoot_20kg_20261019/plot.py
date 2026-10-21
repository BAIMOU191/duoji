from pathlib import Path
import csv,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent
OUT=HERE/'output'
plt.rcParams['font.sans-serif']=['Microsoft YaHei']
plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(3,1,figsize=(10,8),sharex=True)
for name,label in [('after_9000_0_0_1','参数匹配'),('after_9000_0_2_1.3','实际 KA = 标定值 × 1.30'),('after_9000_0_5_1.15','实际 KV = 标定值 × 1.15')]:
    data=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(open(OUT/(name+'.csv')))];ts=[r['t'] for r in data]
    axs[0].plot(ts,[r['x']-r['target'] for r in data],label=label)
    axs[1].plot(ts,[r['v'] for r in data],label=label)
    axs[2].plot(ts,[r['ui'] for r in data],label=label)
axs[0].axhline(0,color='black',lw=.8);axs[0].set_ylim(-.4,.5);axs[0].set_ylabel('实际角度 − 目标 (°)');axs[0].legend(loc='lower right')
axs[1].set_ylim(-5,45);axs[1].set_ylabel('实际速度 (°/s)')
axs[2].set_ylabel('积分输出 (mV)');axs[2].set_xlabel('指令后时间 (ms)');axs[2].set_xlim(580,1000)
for ax in axs:ax.grid(alpha=.25)
fig.suptitle('20kg 空载，135° → 225°，T=0\n实线为当前代码；修改前代码在相同条件下逐拍重合。参数偏差为敏感性假设，尚非实机测量。',fontsize=11)
fig.tight_layout();fig.savefig(OUT/'overshoot.png',dpi=160)
rows=json.loads((OUT/'results.json').read_text())
selected=[r for r in rows if r['version']=='after' and r['step']==9000 and r['T']==0 and (r['diag']==0 or (r['diag']==2 and r['factor']==1.3) or (r['diag']==5 and r['factor']==1.15))]
metrics=[]
for r in selected:
    data=[{k:float(v) for k,v in row.items()} for row in csv.DictReader(open(r['trace']))]
    peak=int(r['peak_t']);back=next((x['t']-peak for x in data[peak:] if x['x'] <= data[peak]['x']-.1),None)
    metrics.append({'diag':r['diag'],'factor':r['factor'],'overshoot_deg':r['over'],'time_to_retract_0_1_deg_ms':back,'final_error_deg':r['final'],'reference_done_ms':r['done'],'peak_ms':peak})
(OUT/'summary.json').write_text(json.dumps({'cases':len(rows),'before_after_identical':True,'metrics':metrics},indent=2))
print(json.dumps(metrics,indent=2))
