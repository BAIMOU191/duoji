"""Numerical evidence and rendered plots for the lever audit."""
import csv
import json
import sys
from pathlib import Path
import numpy as np
from scipy.signal import find_peaks
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

folder = Path(sys.argv[1])
def read(name):
    with (folder / name).open(newline='') as f:
        return list(csv.DictReader(f))

original = read('trace_original.csv')
candidate = read('trace_unified_rest.csv')
protected = read('trace_protected.csv')
def pick(rows, token):
    return [r for r in rows if r['case'] == token]
def arr(rows, key):
    return np.array([float(r[key]) for r in rows])

case = 'L1916_J1.724_B0.77_P1478_W50_init0_N0_T0_I5_S25'
slowcase = 'L1916_J1.724_B0.77_P1478_W50_init0_N0_T0_I1_S25'
base = pick(original,case)
slow = pick(original,slowcase)
trial = pick(candidate,case)
assert base and slow and trial
assert all(r['ref_cdeg'] == r['target_cdeg'] for r in base[5000:])
assert all(r['fault'] == '0' and r['pwm_limit'] == '2400' for r in protected)
analysis = {'protection_faults':sorted({r['fault'] for r in protected}),
            'protection_limits':sorted({r['pwm_limit'] for r in protected}),
            'reference_constant_after_5s':True,'cases':{},'sweeps':{}}
for token, data in [(case,base),(slowcase,slow)]:
    tail = data[5000:]
    pp = float(np.ptp(arr(tail,'actual_deg')))
    peaks,_ = find_peaks(arr(tail,'actual_deg'),prominence=max(.5,pp*.65),distance=200)
    times = arr(tail,'t_ms')[peaks]/1000
    zeros = [int(data[i]['t_ms']) for i in range(1,len(data))
             if int(data[i-1]['integral_mv']) > 500 and int(data[i]['integral_mv']) == 0]
    analysis['cases'][token] = {'peak_peak_deg':pp,'major_peak_times_s':times.tolist(),
        'major_peak_median_period_s':float(np.median(np.diff(times))) if len(times)>1 else None,
        'integral_clear_times_ms':zeros}
for name in ['original','no_clear','no_probe','retain_bias','continuous_hold','load_hold','unified_rest']:
    rows = read('sweep_'+name+'.csv')
    analysis['sweeps'][name] = {'n':len(rows), 'tail_peak_peak_over_1deg':sum(float(r['tail_peak_peak_deg'])>1 for r in rows),
                              'worst_tail_peak_peak_deg':max(float(r['tail_peak_peak_deg']) for r in rows)}
(folder/'analysis.json').write_text(json.dumps(analysis,indent=2),encoding='utf-8')

fig, axes = plt.subplots(3,2,figsize=(14,11),constrained_layout=True)
fig.suptitle('30kg lever load | actual C firmware and isolated controller experiments',fontsize=15)
target = float(base[-1]['target_cdeg'])/100
for data,label,color in [(base,'Current firmware','#d75b38')]:
    axes[0,0].plot(arr(data,'t_ms')/1000,arr(data,'actual_deg')-target,label=label,color=color)
axes[0,0].axhline(0,color='black',ls=':',lw=1)
axes[0,0].set(xlim=(0,12),title='Stop reference fixed; axis keeps oscillating (~2.87 deg p-p)',ylabel='Axis error (degree)')
axes[0,1].plot(arr(base,'t_ms')/1000,arr(base,'integral_mv'),label='Support integral (mV)',lw=1.3)
axes[0,1].plot(arr(base,'t_ms')/1000,arr(base,'out_mv'),label='Output (mV)',alpha=.5,lw=.8)
for t in analysis['cases'][case]['integral_clear_times_ms']:
    axes[0,1].axvline(t/1000,color='#d75b38',alpha=.22,lw=.8)
axes[0,1].set(xlim=(0,12),title='Red markers: support integral suddenly cleared',ylabel='Voltage (mV)')
axes[1,0].plot(arr(slow,'t_ms')/1000,arr(slow,'actual_deg')-target,color='#d75b38')
axes[1,0].axhline(0,color='black',ls=':',lw=1)
axes[1,0].set(title='Lower integral bandwidth: major cycle ~1.5 s, ~3.1 deg p-p',ylabel='Axis error (degree)')
for data,label in [(base,'Current firmware'),(trial,'Experimental unified rest law')]:
    axes[1,1].plot(arr(data,'t_ms')/1000,arr(data,'actual_deg')-target,label=label,alpha=.85,lw=1.2)
axes[1,1].axhline(0,color='black',ls=':',lw=1)
axes[1,1].set(title='Nominal lever load: experimental law settles (~0.40 deg error)',ylabel='Axis error (degree)')
distcase='disturb_J1.724_B0.77_N0'
for data,label in [(pick(original,distcase),'Current firmware'),(pick(candidate,distcase),'Experimental unified rest law')]:
    axes[2,0].plot(arr(data,'t_ms')/1000,arr(data,'actual_deg')-float(data[-1]['target_cdeg'])/100,label=label,lw=1.1)
for t in [7,12,18]: axes[2,0].axvline(t,color='black',ls=':',lw=1)
axes[2,0].set(title='7s remove load; 12s restore; 18s external 2 deg push',ylabel='Axis error (degree)')
labels=['Current','No clear','No probe','Keep bias','Continuous','Gentle recovery','Unified rest']
values=[analysis['sweeps'][n]['tail_peak_peak_over_1deg'] for n in analysis['sweeps']]
axes[2,1].barh(labels,values,color=['#d75b38']+['#638aa6']*6)
axes[2,1].set(title='121 conditions: tail oscillation > 1 degree\nUnified rest fails severely at 10x inertia; experimental only',xlabel='Number of conditions (lower is better)')
for ax in axes.flat:
    ax.grid(alpha=.15)
    if ax is not axes[2,1]: ax.set_xlabel('Time after command (s)')
    if ax.get_legend_handles_labels()[0]: ax.legend(fontsize=9,loc='best')
fig.savefig(folder/'lever_comparison.png',dpi=150)
fig.savefig(folder/'lever_comparison.svg')
print('Analysis and plots saved; fixed reference and no protection faults confirmed.')
