from pathlib import Path
import csv,json,math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parent
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False})
colors={15:'#087f8c',20:'#dd6e42',30:'#6946a0'}
metrics={};traces={}
for kg in (15,20,30):
    folder=ROOT/f'CH32V00X_APP_ADC_{kg}kg'
    entries=[]
    for line in (folder/'results.txt').read_text().splitlines():
        parts=line.split(',');record={'type':parts[0]}
        for item in parts[1:]:
            if '=' in item:
                k,v=item.split('=',1)
                try:v=float(v) if '.' in v else int(v)
                except ValueError:pass
                record[k]=v
            else:record['name']=item
        entries.append(record)
    metrics[kg]=entries
    data={}
    for row in csv.DictReader((folder/'trace.csv').open()):
        name=row.pop('scenario');data.setdefault(name,[]).append({k:float(v) for k,v in row.items()})
    traces[kg]=data
(ROOT/'metrics.json').write_text(json.dumps(metrics,indent=2),encoding='utf-8')
fig,axs=plt.subplots(3,2,figsize=(13,10),constrained_layout=True)
for ri,kg in enumerate((15,20,30)):
    for ci,name in enumerate(('step_T0','reverse_T0')):
        d=traces[kg][name];ax=axs[ri,ci];t=[x['t_ms']/1000 for x in d]
        ax.plot(t,[x['target_cdeg']/100 for x in d],color='#b8c3ce',lw=1,label='Command')
        ax.plot(t,[x['ref_cdeg']/100 for x in d],color='#243447',lw=1.4,label='Reference')
        ax.plot(t,[x['pos_cdeg']/100 for x in d],color=colors[kg],lw=1.2,label='Plant')
        ax.set(xlim=(0,4.2),title=f'{kg} kg / '+('94.5 deg step, T=0' if ci==0 else '189 deg reversals every 250 ms'),ylabel='Angle (deg)',xlabel='Time (s)');ax.grid(alpha=.16)
        if ri==0 and ci==0:ax.legend(loc='lower right')
fig.savefig(ROOT/'01_control_chain.png',dpi=160);plt.close(fig)
fig,axs=plt.subplots(2,2,figsize=(13,8),constrained_layout=True)
for kg in (15,20,30):
    for ax,name in [(axs[0,0],'small_T5000'),(axs[0,1],'mismatch_step'),(axs[1,0],'load_after_arrival_KI0'),(axs[1,1],'load_after_arrival_KI80')]:
        d=traces[kg][name];ax.plot([x['t_ms']/1000 for x in d],[(x['pos_cdeg']-x['target_cdeg'])/100 for x in d],color=colors[kg],label=f'{kg} kg',lw=1.2)
for ax,title in zip(axs.flat,['1.35 deg command stretched to 5000 ms','Plant mismatch: inertia x2, Kv x1.2, Fs x1.5, delay x1.5','1200 mV equivalent load at t=2 s / Hold KI=0','Same load / Hold KI=80']):
    ax.axhline(.5,color='#888',ls='--',lw=.8);ax.axhline(-.5,color='#888',ls='--',lw=.8)
    ax.set(title=title,xlabel='Time (s)',ylabel='True error (deg)');ax.grid(alpha=.16)
axs[0,1].set(ylim=(-3,1),xlim=(.7,10));axs[0,0].legend();fig.savefig(ROOT/'02_precision_and_load.png',dpi=160);plt.close(fig)
fig,axs=plt.subplots(3,1,figsize=(12,9),constrained_layout=True)
for ax,kg in zip(axs,(15,20,30)):
    d=[x for x in traces[kg]['small_T5000'] if x['t_ms']<5200]
    t=[x['t_ms']/1000 for x in d]
    ax.plot(t,[(x['ref_cdeg']-13500)/100 for x in d],color='#243447',label='Reference',lw=1.5)
    ax.plot(t,[(x['pos_cdeg']-13500)/100 for x in d],color=colors[kg],label='Plant',lw=1.3)
    ax.set(title=f'{kg} kg: slow 1.35 deg motion / matched model + +/-0.06 deg sensor noise',ylabel='Displacement (deg)',xlabel='Time (s)');ax.grid(alpha=.16)
    if kg==15:ax.legend()
fig.savefig(ROOT/'03_slow_motion.png',dpi=160);plt.close(fig)
for kg in (15,20,30):
    print(kg)
    for e in metrics[kg]:
        if e['type'] in ('SCENARIO','TRAJECTORY'):
            print(e.get('name'),e.get('error_deg'),e.get('max_tracking_deg'),e.get('last_command_arrived_ms'))
