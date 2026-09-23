"""带载工况矩阵仿真：场景(小角度/大角度/来回转/PWM输入) x 负载(空载/惯量/重力/柔性/制动)。

用法：
    python tests/load_matrix.py                         用本工程源码跑全矩阵
    python tests/load_matrix.py --root <工程目录> --label old   用另一份源码(对象模型仍取本工程 tests/sim)
    python tests/load_matrix.py --compare a.json b.json  并排比较两次结果

对象模型见 tests/sim/load_sim.c：生产控制代码一行不改地链接进来，只替换被控对象。
简化模型，结论只用于同条件下的方案比较，不是实机保证。
"""
import argparse, json, pathlib, subprocess, sys, statistics

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = HERE / 'results' / 'load_matrix'

SCENARIOS = ['small', 'large', 'large_t', 'recip', 'recip_hold', 'pwm']
# 负载工况。grav 为重力力矩幅值(PWM当量，满量程2400)；gmode=1 为摆臂(行程中点水平)；
# inertia 为外加惯量/空载惯量；fres 为空载谐振(Hz)，开启两质量柔性模型；vs 为 Stribeck 速度。
LOADS = {
    '空载':          {},
    '空载+制动x1.54': {'brake': 1.54},
    '惯量x3':        {'inertia': 3},
    '摆臂400':       {'grav': 400, 'gmode': 1},
    '摆臂800+惯量x3': {'grav': 800, 'gmode': 1, 'inertia': 3},
    '恒载600+惯量x2': {'grav': 600, 'gmode': 0, 'inertia': 2},
    '摆臂800+柔性':   {'grav': 800, 'gmode': 1, 'inertia': 3, 'fres': 130, 'backlash': 5, 'zeta': 0.1},
    '摆臂600+Stribeck': {'grav': 600, 'gmode': 1, 'inertia': 2, 'vs': 300},
    '摆臂400+制动x1.54': {'grav': 400, 'gmode': 1, 'inertia': 2, 'brake': 1.54},
}
METRICS = ['hold_travel', 'hold_rev', 'hold_dpwm', 'hold_pwm_sw', 'hold_pp', 'hold_err',
           'over_max', 'settle_max', 'unsettled', 'move_rev', 'move_dpwm', 'hold_on']
LABELS = {'hold_travel': '保持微动cd/s', 'hold_rev': '保持换向/s', 'hold_dpwm': '保持|dPWM|',
          'hold_pwm_sw': 'PWM翻转/s', 'hold_pp': '保持峰峰', 'hold_err': '静差',
          'over_max': '超调', 'settle_max': '到位ms', 'unsettled': '未到位', 'move_rev': '运动换向',
          'move_dpwm': '全程|dPWM|', 'hold_on': '保持通电比'}


def build(src_root, label):
    OUT.mkdir(parents=True, exist_ok=True)
    exe = OUT / f'load_sim_{label}.exe'
    inc = [HERE / 'sim' / 'stubs', HERE / 'sim', src_root / 'Application',
           src_root / 'Common', src_root / 'Drivers']
    srcs = [HERE / 'sim' / 'load_sim.c', src_root / 'Application' / 'A_Sensor.c'] + \
           [src_root / 'Common' / f'{n}.c' for n in ('C_Pos_Ctrl', 'C_Speed_Observer', 'C_Traj_Planner')]
    cmd = ['gcc', '-std=c99', '-O2', '-w'] + ['-I' + str(p) for p in inc] + \
          [str(s) for s in srcs] + ['-lm', '-o', str(exe)]
    subprocess.run(cmd, check=True)
    return exe


def run_one(exe, scen, params, trace_to=None):
    args = [str(exe), scen] + [f'{k}={v}' for k, v in params.items()]
    if trace_to:
        with open(trace_to, 'w') as f:
            r = subprocess.run(args + ['trace=1'], stdout=f, stderr=subprocess.PIPE, text=True)
    else:
        r = subprocess.run(args, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    if r.returncode != 0:
        raise RuntimeError(r.stderr)
    return json.loads(r.stderr.strip().splitlines()[-1])


def run_matrix(exe, extra=None, seeds=(1, 7)):
    res = {}
    for ln, lp in LOADS.items():
        for sc in SCENARIOS:
            vals = []
            for sd in seeds:
                p = dict(lp); p.update(extra or {}); p['seed'] = sd
                vals.append(run_one(exe, sc, p))
            # 多个随机种子取最坏值
            res[f'{ln}|{sc}'] = {m: max(v[m] for v in vals) for m in METRICS}
    return res


def score(res):
    """把矩阵压成几项总分：保持抖动(微动+换向+电气)、运动质量(超调/到位)"""
    agg = {}
    for m in METRICS:
        agg[m + '_avg'] = round(statistics.mean(v[m] for v in res.values()), 2)
        agg[m + '_max'] = round(max(v[m] for v in res.values()), 2)
    return agg


def table(results, keys=('hold_travel', 'hold_pp', 'hold_dpwm', 'over_max', 'settle_max', 'move_rev')):
    names = list(results)
    rows = sorted(set().union(*[r['cases'].keys() for r in results.values()]),
                  key=lambda c: (list(LOADS).index(c.split('|')[0]) if c.split('|')[0] in LOADS else 99,
                                 SCENARIOS.index(c.split('|')[1]) if c.split('|')[1] in SCENARIOS else 99))
    hdr = '%-28s' % '工况|场景' + ''.join('%-*s' % (13 * len(names), LABELS[k]) for k in keys)
    print(hdr)
    print('%-28s' % '' + ''.join(''.join('%-13s' % n[:12] for n in names) for _ in keys))
    for c in rows:
        line = '%-28s' % c
        for k in keys:
            for n in names:
                v = results[n]['cases'].get(c, {}).get(k, float('nan'))
                line += '%-13s' % (('%.0f' if abs(v) >= 100 else '%.2f' if abs(v) < 10 else '%.1f') % v)
        print(line)
    print()
    for k in keys:
        print('%-14s' % LABELS[k] + '  '.join('%s avg=%-8.2f max=%-8.2f' % (n, results[n]['score'][k + '_avg'],
                                                                           results[n]['score'][k + '_max'])
                                              for n in names))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default=str(ROOT))
    ap.add_argument('--label', default='current')
    ap.add_argument('--compare', nargs='*')
    ap.add_argument('--set', nargs='*', default=[], help='额外对象参数 k=v，作用于全部工况')
    a = ap.parse_args()
    if a.compare:
        results = {pathlib.Path(p).stem: json.loads(pathlib.Path(p).read_text(encoding='utf-8'))
                   for p in a.compare}
        table(results)
        return 0
    extra = {kv.split('=')[0]: kv.split('=')[1] for kv in a.set}
    exe = build(pathlib.Path(a.root), a.label)
    res = run_matrix(exe, extra)
    out = {'root': a.root, 'extra': extra, 'cases': res, 'score': score(res)}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f'{a.label}.json').write_text(json.dumps(out, indent=1, ensure_ascii=False), encoding='utf-8')
    table({a.label: out})
    return 0


if __name__ == '__main__':
    sys.exit(main())
