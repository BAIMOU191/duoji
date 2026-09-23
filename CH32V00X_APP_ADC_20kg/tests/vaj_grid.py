"""轨迹三参数(V/A/J)网格扫描：验证"随便改参数，舵机仍能完整走完一次运动，只是快慢/柔和不同"。

用法：
    python tests/vaj_grid.py                      默认网格 x 对象失配，打印每种对象下的最坏指标和需要留意的组合
    python tests/vaj_grid.py 265 8000 1000000      只看一组参数，逐个位移打印(标称对象，减速跟随加速)
    python tests/vaj_grid.py 265 8000 1000000 2000 同上但指定最大减速度

仿真链接生产代码本体(见 tests/sim/vaj_sim.c)，V/A/J 在运行时由 C_Traj_Set_Limits 设置，不用改面板重编译。
判据：终点误差不出到位窗、不出现两段、超调不超过到位窗。仿真对象按 2026-09-22 实机遥测(tau42、制动不偏弱)；
控制器 MODEL_TAU_MS 42、MODEL_BRAKE_GAIN_Q8 290、制动放行 DSGC_BRAKE_RELAX_X 2。标称对象下各组末端都与参考同时到位；
惯量比模型大30%时大位移超调1~1.5度(带重负载应把 MODEL_BRAKE_GAIN_Q8 往330调或带载重新标定)；
J=2000 这种极软设置小位移会爬行出两段；电机增益比标定低30%时中速大位移超调约1.5~2度(速度环积分补偿增益误差的老问题)。
"""
import itertools, pathlib, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = HERE / 'results'
INC = ['-I' + str(p) for p in [HERE / 'sim' / 'stubs', HERE / 'sim', ROOT / 'Application',
                               ROOT / 'Common', ROOT / 'Drivers']]
SRCS = [str(HERE / 'sim' / 'vaj_sim.c'), str(ROOT / 'Application' / 'A_Sensor.c')] + \
       [str(ROOT / 'Common' / f'{n}.c') for n in ('C_Pos_Ctrl', 'C_Speed_Observer', 'C_Traj_Planner')]

V = [30, 100, 265]                               # 度/秒
A = [300, 4000, 20000]                           # 度/秒^2
D = [500, 2000, 6000]                            # 度/秒^2，最大减速度
J = [10000, 120000, 1000000]                     # 度/秒^3
PLANTS = {'标称': (1, 1, 1), 'tau+30%': (1, 1.3, 1), 'tau-30%': (1, 0.7, 1), '增益+30%': (1.3, 1, 1),
          '增益-30%': (0.7, 1, 1), '摩擦+30%': (1, 1, 1.3), '摩擦-30%': (1, 1, 0.7)}
IN_WIN = 67   # CTRL_IN_WIN_CDEG，本机 2.5 倍最小步长


def build():
    OUT.mkdir(exist_ok=True)
    exe = OUT / 'vaj_sim.exe'
    subprocess.run(['gcc', '-std=c99', '-O2', '-w'] + INC + SRCS + ['-lm', '-o', str(exe)], check=True)
    return exe


def run(exe, v, a, j, plant=(1, 1, 1), verbose=False, d=0):
    args = [str(exe), str(v), str(a), str(j), *map(str, plant)] + [('v' if verbose else 'q'), str(d)]
    out = subprocess.run(args, capture_output=True, text=True, check=True).stdout
    last = out.strip().splitlines()[-1]
    return out, dict((k, float(x)) for k, x in (kv.split('=') for kv in last.split()))


def main():
    exe = build()
    if len(sys.argv) in (4, 5):
        d = sys.argv[4] if len(sys.argv) == 5 else 0
        print(run(exe, *sys.argv[1:4], verbose=True, d=d)[0])
        return
    for name, plant in PLANTS.items():
        combos = list(itertools.product(V, A, D, J))
        with ThreadPoolExecutor(8) as ex:
            res = list(ex.map(lambda c: (c, run(exe, c[0], c[1], c[3], plant, d=c[2])[1]), combos))
        note = [(c, r) for c, r in res if r['err'] > IN_WIN or r['segs'] > 1 or r['over'] > IN_WIN]
        worst = {k: max(r[k] for _, r in res) for k in res[0][1]}
        print(f'== {name}: {len(res)} 组，需留意 {len(note)} 组；最坏 ' +
              ' '.join(f'{k}={v:.0f}' for k, v in worst.items()))
        for c, r in note[:6]:
            print(f'     V={c[0]} A={c[1]} D={c[2]} J={c[3]}: over={r["over"]:.0f} err={r["err"]:.0f} '
                  f'segs={r["segs"]:.0f} rev={r["rev"]:.0f} tail={r["tail"]:.0f}')


if __name__ == '__main__':
    main()
