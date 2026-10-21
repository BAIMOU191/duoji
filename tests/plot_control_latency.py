"""Render deterministic motor/trajectory audit results; uses matplotlib only."""
import csv
import sys
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

folder = Path(sys.argv[1])
with (folder / 'trace.csv').open(newline='') as file:
    rows = list(csv.DictReader(file))

fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
fig.suptitle('30kg current C firmware | deterministic host simulation', fontsize=15)
for ax, case, title, horizon in [
    (axes[0, 0], 'single_1deg_fast', 'About 1 degree, T=0: 0.13 degree at 24 ms', 250),
    (axes[0, 1], 'single_1deg_1000ms', 'Same move, T=1000: 0.13 degree at 377 ms', 1100),
    (axes[1, 0], 'retarget_age20_to1600', 'Opposite target after 20 ms: automatic Follow', 450),
    (axes[1, 1], 'retarget_age20_to1600_force_plan', 'Same opposite target: force ordinary Plan', 450),
]:
    data = [r for r in rows if r['case'] == case and int(r['t_ms']) <= horizon]
    start = float(data[0]['actual_cdeg'])
    ax.plot([int(r['t_ms']) for r in data], [(float(r['ref_cdeg'])-start)/100 for r in data], label='Reference', lw=2)
    ax.plot([int(r['t_ms']) for r in data], [(float(r['actual_cdeg'])-start)/100 for r in data], label='Axis', lw=1.5)
    ax.set(title=title, xlabel='Time since complete command (ms)', ylabel='Displacement (degree)')
    ax.grid(alpha=.2)
    ax.legend(loc='best')
    if case.startswith('single'):
        ax.axhline(.13, color='#777777', ls=':', lw=1)
    else:
        ax.axhline(0, color='#777777', ls=':', lw=1)
        ax.axhline(-.13, color='#777777', ls=':', lw=1)
fig.savefig(folder / 'latency_comparison.png', dpi=150)
fig.savefig(folder / 'latency_comparison.svg')
