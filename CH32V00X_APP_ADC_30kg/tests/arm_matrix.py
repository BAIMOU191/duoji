"""机械臂肩关节工况矩阵：手臂参数网格 x 场景(保持/慢动/主机插补/PWM输入/其它关节扰动)。

用法：
    python tests/arm_matrix.py                              本工程源码
    python tests/arm_matrix.py --root <源码目录> --label old  另一份源码(对象模型取本工程 tests/sim)
    python tests/arm_matrix.py --compare a b ...             比较 results/arm_matrix/<label>.json
对象模型见 tests/sim/arm_sim.c。简化模型，只用于同条件下比较方案。
"""
import argparse, itertools, json, pathlib, subprocess, sys, statistics
import concurrent.futures as cf

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = HERE / 'results' / 'arm_matrix'
SCENARIOS = ['hold', 'slow', 'stream', 'pwm', 'disturb']
GRID = {
    'inertia': [1, 3, 8],
    'farm': [0, 4, 8, 15],
    'zarm': [0.03, 0.1],
    'gap': [0, 60],
    'grav': [500, 1200],
}
METRICS = ['hold_pp', 'hold_swing', 'hold_jit', 'hold_pwm_std', 'hold_dpwm', 'move_jit', 'move_dpwm', 'end_pp', 'growth']


def build(src_root, label):
    OUT.mkdir(parents=True, exist_ok=True)
    exe = OUT / f'arm_sim_{label}.exe'
    inc = [HERE / 'sim' / 'stubs', HERE / 'sim', src_root / 'Application', src_root / 'Common', src_root / 'Drivers']
    srcs = [HERE / 'sim' / 'arm_sim.c', src_root / 'Application' / 'A_Sensor.c'] + \
           [src_root / 'Common' / f'{n}.c' for n in ('C_Pos_Ctrl', 'C_Speed_Observer', 'C_Traj_Planner')]
    subprocess.run(['gcc', '-std=c99', '-O2', '-w'] + ['-I' + str(p) for p in inc] +
                   [str(s) for s in srcs] + ['-lm', '-o', str(exe)], check=True)
    return exe


def run_one(exe, scen, params, trace_to=None):
    args = [str(exe), scen] + [f'{k}={v}' for k, v in params.items()]
    if trace_to:
        with open(trace_to, 'w') as f:
            r = subprocess.run(args + ['trace=1'], stdout=f, stderr=subprocess.PIPE, text=True)
    else:
        r = subprocess.run(args, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    return json.loads(r.stderr.strip().splitlines()[-1])


def combos(grid=GRID):
    keys = list(grid)
    for vals in itertools.product(*[grid[k] for k in keys]):
        d = dict(zip(keys, vals))
        if d['farm'] == 0 and (d['zarm'] != grid['zarm'][0] or d['gap'] != grid['gap'][0]):
            continue                      # 刚性连接时阻尼/间隙无意义
        yield d


def run_matrix(exe, extra=None, grid=GRID):
    jobs = [(c, sc) for c in combos(grid) for sc in SCENARIOS]

    def one(j):
        c, sc = j
        p = dict(c); p.update(extra or {})
        return ('|'.join(f'{k}={v}' for k, v in c.items()) + '|' + sc, run_one(exe, sc, p))
    with cf.ThreadPoolExecutor(max_workers=8) as ex:
        return dict(ex.map(one, jobs))


def summary(res):
    s = {}
    for sc in SCENARIOS:
        rows = [v for k, v in res.items() if k.endswith('|' + sc)]
        for m in METRICS:
            s[f'{sc}.{m}.avg'] = statistics.mean(r[m] for r in rows)
            s[f'{sc}.{m}.max'] = max(r[m] for r in rows)
    return s


def show(labels):
    data = {l: json.loads((OUT / f'{l}.json').read_text(encoding='utf-8')) for l in labels}
    cols = [('hold', 'hold_pp'), ('hold', 'hold_pwm_std'), ('hold', 'hold_swing'), ('slow', 'move_jit'),
            ('slow', 'hold_pwm_std'), ('stream', 'move_jit'), ('stream', 'hold_pwm_std'), ('pwm', 'move_jit'),
            ('disturb', 'hold_pp'), ('disturb', 'hold_swing')]
    print('%-18s' % 'label' + ''.join('%-22s' % f'{a}.{b}' for a, b in cols))
    for l in labels:
        s = data[l]['summary']
        print('%-18s' % l + ''.join('%-22s' % ('%.1f/%.1f' % (s[f'{a}.{b}.avg'], s[f'{a}.{b}.max'])) for a, b in cols))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default=str(ROOT))
    ap.add_argument('--label', default='current')
    ap.add_argument('--compare', nargs='*')
    a = ap.parse_args()
    if a.compare:
        show(a.compare); return 0
    exe = build(pathlib.Path(a.root), a.label)
    res = run_matrix(exe)
    (OUT / f'{a.label}.json').write_text(json.dumps({'cases': res, 'summary': summary(res)}, ensure_ascii=False),
                                         encoding='utf-8')
    show([a.label])
    return 0


if __name__ == '__main__':
    sys.exit(main())
