# 跑一组配置的全矩阵并汇总。用法: python runcfg.py name "KNOB=V KNOB=V" [models]
import subprocess, sys, os, statistics as st, json
from concurrent.futures import ThreadPoolExecutor

def run(exe, model, knobs):
    out = subprocess.run([exe, model, 'matrix', 'NUL', '*', '*'] + knobs.split(), capture_output=True, text=True).stdout
    R = {}
    for line in out.splitlines():
        if line.startswith('CAL'): continue
        p = line.strip().split(',')
        d = {}
        for kv in p[3:]:
            k, v = kv.split('='); d[k] = float(v)
        R[(p[1], p[2])] = d
    return R

LOADED = ['inertia2', 'inertia4', 'inertia8', 'grav20%_r4', 'grav40%_r6', 'grav60%_r8', 'stepload+3V', 'stepload-3V',
          'randload', 'grav30%_r4_bl0.6', 'grav30%_r4_6.4V', 'push_r4', 'push_r8', 'push_r4_bl']
MIS = ['mis_ka0.7', 'mis_ka1.5', 'mis_kv0.85', 'mis_kv1.15', 'mis_fric0.5', 'mis_fric1.6', 'mis_fs2.0', 'mis_delay0.6',
       'mis_delay1.5', 'mis_noise2.5x', 'pot_ripple', 'push_r0']
PM = ['bigstep', 'reverse400', 'reverse_arrive', 'timed90', 'hold']
STREAM = ['stream_pwm', 'stream_bus50_T0', 'stream_bus100_T100', 'stream_bus200_T200']

def summarize(R):
    s = {}
    os_ = [R[(l, c)]['os'] for l in LOADED for c in PM]
    s['os_med'] = st.median(os_); s['os_p90'] = sorted(os_)[int(.9 * len(os_))]; s['os_max'] = max(os_)
    s['ring'] = sum(R[(l, c)]['ring'] for l in LOADED for c in PM)
    s['out'] = sum(abs(R[(l, c)]['final']) > 0.5 for l in LOADED for c in PM)
    s['fault'] = sum(R[(l, c)]['fault'] for l in R for c in [0] if False) + sum(R[k]['fault'] for k in R)
    s['stall'] = sum(R[k]['stall'] for k in R)
    s['mis_out'] = sum(abs(R[(l, c)]['final']) > 0.5 for l in MIS for c in PM)
    s['mis_os_max'] = max(R[(l, c)]['os'] for l in MIS for c in PM)
    s['nl_os_max'] = max(R[('noload', c)]['os'] for c in PM)
    s['nl_final_max'] = max(abs(R[('noload', c)]['final']) for c in PM)
    s['nl_track_max'] = max(R[('noload', c)]['track'] for c in PM)
    s['st_vmin'] = min(R[(l, c)]['vmin_r'] for l in ['noload', 'inertia4', 'grav20%_r4'] for c in STREAM)
    s['st_refvmin'] = min(R[(l, c)]['refvmin_r'] for l in ['noload', 'inertia4', 'grav20%_r4'] for c in STREAM)
    s['st_stops_var'] = sum(R[(l, c)]['stops'] for l in ['stepload+3V', 'stepload-3V', 'randload'] for c in STREAM)
    s['st_track_var'] = max(R[(l, c)]['track'] for l in ['stepload+3V', 'stepload-3V', 'randload'] for c in STREAM)
    s['sine_lag'] = max(R[(l, 'sine_T20')]['lag'] for l in ['noload', 'inertia4', 'grav20%_r4'])
    s['hold_push_r0'] = R[('push_r0', 'hold')]['final']
    s['maxdu_nl'] = max(R[('noload', c)]['maxdu'] for c in PM)
    return s

if __name__ == '__main__':
    name, knobs = sys.argv[1], sys.argv[2]
    models = sys.argv[3].split(',') if len(sys.argv) > 3 else ['15', '20', '30']
    exe = 's' if name == 'base0' else 'c'
    with ThreadPoolExecutor(3) as ex:
        res = dict(zip(models, ex.map(lambda m: run(f'./{exe}{m}.exe', m + 'kg', knobs), models)))
    os.makedirs('runs', exist_ok=True)
    json.dump({m: {f'{k[0]}|{k[1]}': v for k, v in res[m].items()} for m in models}, open(f'runs/{name}.json', 'w'))
    S = {m: summarize(res[m]) for m in models}
    keys = list(S[models[0]].keys())
    print(f'{name:<14}', ' '.join(f"{k}={'/'.join(f'{S[m][k]:.2f}' if isinstance(S[m][k], float) else str(S[m][k]) for m in models)}" for k in keys))
