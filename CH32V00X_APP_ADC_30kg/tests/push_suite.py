"""到位保持刚度的手推测试集：刚性/柔性手臂 x 有无重力 x 推力大小与方向。
    python tests/push_suite.py <arm_sim.exe>   打印汇总
"""
import json, pathlib, subprocess, sys, statistics, concurrent.futures as cf
ARMS = {'刚性': {'farm': 0, 'inertia': 3}, '柔性4Hz': {'farm': 4, 'inertia': 3, 'zarm': 0.08, 'gap': 40},
        '柔性6Hz': {'farm': 6, 'inertia': 2, 'zarm': 0.05, 'gap': 30}}
GRAVS = {'无重力': 0, '重力900': 900}
FORCES = [400, 800, 1500, -800]


def run(exe, extra=None):
    exe = pathlib.Path(exe).resolve()
    jobs = [(a, g, f) for a in ARMS for g in GRAVS for f in FORCES]
    def one(j):
        a, g, f = j
        p = dict(ARMS[a], grav=GRAVS[g], pushf=f); p.update(extra or {})
        r = subprocess.run([str(exe), 'push'] + [f'{k}={v}' for k, v in p.items()], capture_output=True, text=True)
        return (f'{a}|{g}|F{f}', json.loads(r.stderr.strip().splitlines()[-1]))
    with cf.ThreadPoolExecutor(max_workers=8) as ex:
        return dict(ex.map(one, jobs))


def summary(res):
    m = lambda key, flt: statistics.mean(v[key] for k, v in res.items() if flt(k))
    mx = lambda key, flt: max(v[key] for k, v in res.items() if flt(k))
    return {
        'loose400': m('shaft_max', lambda k: 'F400' in k),            # 小推力下的让位(松动区)
        'defl800': m('shaft_max', lambda k: 'F800' in k or 'F-800' in k),
        'end800': m('shaft_end', lambda k: 'F800' in k or 'F-800' in k),
        'defl1500': m('shaft_max', lambda k: 'F1500' in k),
        'recoil': m('shaft_recoil', lambda k: True), 'recoil_max': mx('shaft_recoil', lambda k: True),
        'arm_recoil': m('push_recoil', lambda k: True),
        'swing_flex': m('push_swing', lambda k: '柔性' in k), 'final': m('push_final', lambda k: True),
    }


if __name__ == '__main__':
    s = summary(run(sys.argv[1]))
    print('  '.join(f'{k}={v:.1f}' for k, v in s.items()))
