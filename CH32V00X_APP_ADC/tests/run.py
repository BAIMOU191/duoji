"""本工程的全套主机端验证：编译生产代码 -> 跑回归/审计/仿真场景 -> 汇总指标。

用法：
    python tests/run.py            回归 + 审计 + 全部仿真场景 + 输入分辨率 + 鲁棒性扫描
    python tests/run.py quick      只跑回归和审计(几秒)

仿真链接的是生产代码本体(A_Servo.c / A_Sensor.c / A_Protect.c / C_*.c)，只有
ADC、PWM 捕获和 H桥被替换成桩，被控对象按 A_Parameter.h 里本机的标定值建模。
编码器模式由 D_adc.h 的 D_ADC_ENCODER_ENABLE 决定，本脚本自动识别并跳过不适用
的场景(例如整圈可测的编码器没有"脱困"这回事)。
"""
import csv, json, pathlib, re, subprocess, sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = HERE / 'results'
CC = ['gcc', '-std=c99', '-O2', '-Wall', '-Wextra', '-Wno-unused-parameter']
INC = ['-I' + str(p) for p in [HERE / 'sim' / 'stubs', ROOT / 'Application',
                               ROOT / 'Common', ROOT / 'Drivers']]
SYSINC = ['-I' + str(ROOT / 'System')]   # A_Config.c 要 flash.h
COMMON = [str(ROOT / 'Common' / f'{n}.c') for n in
          ('C_Pos_Ctrl', 'C_Speed_Observer', 'C_Traj_Planner')]
SENSOR = str(ROOT / 'Application' / 'A_Sensor.c')

CYCLE_PERIOD_MS = 3000     # servo_sim 的 cycle 场景每3秒换一次目标
MICRO_PERIOD_MS = 600      # micro 场景每级停留时间，与 servo_sim 的 SIM_MICRO_MS 一致
MICRO_STEP_US = 5          # micro 场景每级的脉宽增量，与 SIM_MICRO_US 一致
SPAN_CDEG = 27000          # 模式1的270度行程


def has_deadzone():
    """本工程的编码器有没有"测不到角度"的区间，取自 D_adc.h 的板级开关。"""
    text = (ROOT / 'Drivers' / 'D_adc.h').read_text(encoding='utf-8')
    m = re.search(r'#define\s+D_ADC_ENCODER_ENABLE\s+(\d)', text)
    return bool(m and m.group(1) == '1')


def dir_invert():
    """整机旋向有没有被翻过来，取自 A_Parameter.h 的 CFG_DIR_INVERT。"""
    text = (ROOT / 'Application' / 'A_Parameter.h').read_text(encoding='utf-8')
    m = re.search(r'#define\s+CFG_DIR_INVERT\s+(\d)', text)
    return bool(m and m.group(1) == '1')


DEADZONE = has_deadzone()
DIR_INVERT = dir_invert()


def pwm_to_cdeg(pwm):
    """协议脉宽 -> 行程坐标，与 Servo_PulseToAngle 同式。
    CFG_DIR_INVERT=1 时同一脉宽落在行程另一头，不镜像就整整差一个量程。行程两端可达，不再夹。
    """
    a = (pwm - 500) * SPAN_CDEG / 2000.0
    if DIR_INVERT:
        a = SPAN_CDEG - a
    return a


# cycle 场景交替发 500us / 2000us
CYCLE_TARGETS = (pwm_to_cdeg(500), pwm_to_cdeg(2000))


def build(name, sources, extra=()):
    OUT.mkdir(exist_ok=True)
    exe = OUT / f'{name}.exe'
    subprocess.run(CC + list(extra) + INC + sources + ['-lm', '-o', str(exe)], check=True)
    return exe


def run_case(exe, *args):
    r = subprocess.run([str(exe), *args], capture_output=True, text=True)
    return r.returncode, (r.stdout + r.stderr).strip()


def scenario(exe, mode, params=(), suffix=''):
    """跑一个时序场景，回收控制质量指标。params 依次是
    对象增益 / tau 倍率 / 摩擦倍率 / 噪声(厘度) / 纯延迟(拍)。"""
    dest = OUT / f'{mode}{suffix}.csv'
    with dest.open('w') as f:
        subprocess.run([str(exe), mode, *map(str, params)], stdout=f, check=True)
    with dest.open() as f:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]
    tail = rows[-1000:]
    res = {
        'final_pos': round(rows[-1]['pos'], 1),
        'tail_motion_pp': round(max(r['pos'] for r in tail) - min(r['pos'] for r in tail), 2),
        'tail_nonzero_pwm': sum(r['pwm'] != 0 for r in tail),
        'tail_max_pwm': max(abs(r['pwm']) for r in tail),
        'max_abs_pwm': max(abs(r['pwm']) for r in rows),
        'escape_entries': sum(r['enc_state'] == 1 and (i == 0 or rows[i - 1]['enc_state'] != 1)
                              for i, r in enumerate(rows)),
    }
    if mode == 'flip':    # 往复换向：换向段PWM单拍跳变，越小冲击越轻
        body = rows[1500:9000]
        res['max_dpwm'] = max(abs(b['pwm'] - a['pwm']) for a, b in zip(body, body[1:]))
    if mode == 'micro':   # 微步进：每级都要走完并停稳，看的是抖动而不是速度
        res['steps'] = []
        for start in range(0, len(rows) - MICRO_PERIOD_MS + 1, MICRO_PERIOD_MS):
            b = rows[start:start + MICRO_PERIOD_MS]
            target = pwm_to_cdeg(b[0]['pwm_target'])
            direction = 1 if target > b[0]['pos'] else -1 if target < b[0]['pos'] else 0
            if direction == 0:
                continue
            revs, prev = 0, 0
            for r in b:                      # 轴真的来回动了几次(忽略停住的拍)
                if abs(r['vel']) < 20:
                    continue
                sgn = 1 if r['vel'] > 0 else -1
                if prev and sgn != prev:
                    revs += 1
                prev = sgn
            vref = max(abs(r['ref_vel']) for r in b)
            res['steps'].append({
                'overshoot': round(max(0.0, max(direction * (r['pos'] - target) for r in b)), 1),
                'reversals': revs,
                'ratio': round(max(abs(r['vel']) for r in b) / vref, 2) if vref > 1 else 0,
                'err': round(b[-1]['pos'] - target, 1),
                'tail_pwm': sum(r['pwm'] != 0 for r in b[-200:]),
            })
    if mode == 'cycle':   # 只有 cycle 会走完整的"发指令->到位->静止"过程
        res['moves'] = []
        for start in range(0, len(rows) - CYCLE_PERIOD_MS + 1, CYCLE_PERIOD_MS):
            block = rows[start:start + CYCLE_PERIOD_MS]
            target = CYCLE_TARGETS[(start // CYCLE_PERIOD_MS) % 2]
            direction = 1 if target > block[0]['pos'] else -1
            done = next((i for i, r in enumerate(block) if r['done']), CYCLE_PERIOD_MS - 1)
            res['moves'].append({
                'done_ms': done,
                'overshoot': round(max(0.0, max(direction * (r['pos'] - target) for r in block)), 1),
                'final_error': round(block[-1]['pos'] - target, 1),
                'post_done_travel': round(sum(abs(block[i]['pos'] - block[i - 1]['pos'])
                                              for i in range(done + 1, len(block))), 1),
                'tail_pwm': sum(r['pwm'] != 0 for r in block[-1000:]),
            })
    return res


def worst(res, key):
    return max(abs(m[key]) for m in res['moves'])


# 输入分辨率：1us 的阶梯里有几级真的被目标死区放行，放行时步距多大。
# 总线和 PWM 输入走同一个内核，差别只在入口分辨率和 PWM 专属的额外死区，
# 所以这三行放在一起看就是"PWM 模式到底比总线粗多少"。
STEP_MODES = ('bus-micro', 'pwm-micro', 'pwm-micro-jit')
STEP_LEVEL_MS = 600     # 与 servo_sim 的 SIM_PWM_MICRO_MS 一致
STEP_INPUT_US = 1       # 与 servo_sim 的 SIM_PWM_MICRO_US 一致


def input_resolution(exe, mode):
    """跑 1us 阶梯，回收"生效级数"和"每次生效的目标步距(厘度)"。"""
    dest = OUT / f'{mode}.csv'
    with dest.open('w') as f:
        subprocess.run([str(exe), mode], stdout=f, check=True)
    with dest.open() as f:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
    levels, steps = [], []
    for start in range(0, len(rows) - STEP_LEVEL_MS + 1, STEP_LEVEL_MS):
        tail = rows[start + STEP_LEVEL_MS - 100:start + STEP_LEVEL_MS]
        levels.append(sum(r['ref_pos'] for r in tail) / len(tail))
    for a, b in zip(levels, levels[1:]):
        if abs(b - a) > 1:
            steps.append(abs(b - a))
    return {'levels': len(levels), 'accepted': len(steps),
            'step_min': round(min(steps), 1) if steps else 0,
            'step_max': round(max(steps), 1) if steps else 0}


def main():
    quick = len(sys.argv) > 1 and sys.argv[1] == 'quick'
    report, failures = {'encoder': '电位器ADC' if DEADZONE else 'MT6701磁编码'}, 0

    sim = build('sim', [str(HERE / 'sim' / 'servo_sim.c'), SENSOR] + COMMON)
    feature = build('feature_audit', [str(HERE / 'audit' / 'feature_audit.c'), SENSOR] + COMMON)
    # 协议层和参数层各自独立成可执行文件：它们把舵机换成探针/把Flash换成内存，
    # 查的是"指令分发对不对""旧参数升级后还在不在"，掺进整机仿真反而看不清。
    proto = build('proto_audit', [str(HERE / 'audit' / 'proto_audit.c')], extra=SYSINC)
    config = build('config_audit', [str(HERE / 'audit' / 'config_audit.c')], extra=SYSINC)
    planner = build('planner_audit', [str(HERE / 'audit' / 'planner_audit.c')])
    sched = build('sched_audit', [str(HERE / 'audit' / 'sched_audit.c')])
    protect = build('protect_audit', [str(HERE / 'audit' / 'protect_audit.c')])
    servo = build('servo_audit', [str(HERE / 'audit' / 'servo_audit.c'), SENSOR] + COMMON)
    stall = build('stall_audit', [str(HERE / 'audit' / 'stall_audit.c'), SENSOR] + COMMON,
                  extra=['-DSIM_REAL_PROTECT'])

    print('===== 回归与审计 (%s) =====' % report['encoder'])
    servo_cases = ['range', 'save', 'protection', 'invalid_range']
    if DEADZONE:
        servo_cases += ['stop', 'release', 'pause']
    # 新增协议功能的专项审计。多圈只在整圈可测的编码器上存在，量程收缩只在
    # 有物理死区的编码器上存在，两边各跑各的。
    feature_cases = ['pulse_range', 'pulse_limit', 'boot_window', 'endpoint', 'hold_stiff',
                     'cal_mid', 'cal_zero', 'cal_repeat', 'cal_default', 'direction']
    if DEADZONE:
        feature_cases += ['cal_shrink']
    else:
        feature_cases += ['mt_distance', 'mt_timing', 'mt_pause',
                          'mt_interrupt', 'mt_reject', 'mt_longest']

    suites = [(sim, ['regression']), (proto, []), (config, []),
              (planner, []), (protect, []), (sched, [])]
    suites += [(feature, [c]) for c in feature_cases]
    suites += [(servo, [c]) for c in servo_cases]
    suites += [(stall, [c]) for c in
               ['stall', 'free', 'slow', 'recover', 'hold_load', 'jam', 'released']]
    for exe, args in suites:
        code, out = run_case(exe, *args)
        label = (exe.stem + ' ' + ' '.join(args)).strip()
        print(('  OK   ' if code == 0 else '  FAIL ') + label.ljust(24) + out.replace('\n', ' | '))
        failures += code != 0
    report['audit_failures'] = failures
    if quick:
        print('\naudit failures =', failures)
        return 1 if failures else 0

    print('\n===== 控制质量场景 =====')
    modes = ['cycle', 'micro', 'rapid', 'flip', 'jitter', 'disturb', 'load', 'unload']
    if DEADZONE:
        modes += ['boot-low', 'boot-high']
    report['scenarios'] = {m: scenario(sim, m) for m in modes}
    for m, r in report['scenarios'].items():
        extra = ''
        if r.get('moves'):
            extra = '  到位%dms 超调%.1f 终点误差%.1f' % (
                worst(r, 'done_ms'), worst(r, 'overshoot'), worst(r, 'final_error'))
        if 'max_dpwm' in r:
            extra = '  换向PWM单拍跳变max=%d' % r['max_dpwm']
        if r.get('steps'):
            st = r['steps']
            n = len(st)
            extra = ('  %d级x%dus: 过冲avg%.1f 反转avg%.1f 超速avg%.2f 误差max%.1f 尾通电%d'
                     % (n, MICRO_STEP_US, sum(s['overshoot'] for s in st) / n,
                        sum(s['reversals'] for s in st) / n,
                        sum(s['ratio'] for s in st) / n,
                        max(abs(s['err']) for s in st),
                        sum(s['tail_pwm'] for s in st)))
        if r.get('steps'):
            print('  %-10s%s' % (m, extra))   # micro 全程都在动，尾段指标没有意义
        else:
            print('  %-10s 静止段非零PWM=%-5d 静止摆动=%-7s%s'
                  % (m, r['tail_nonzero_pwm'], r['tail_motion_pp'], extra))

    print('\n===== 输入分辨率(1us 阶梯，看目标死区放行几级) =====')
    report['resolution'] = {m: input_resolution(sim, m) for m in STEP_MODES}
    for m, r in report['resolution'].items():
        print('  %-14s %2d级x%dus -> 生效%2d次，步距 %.0f~%.0f 厘度'
              % (m, r['levels'], STEP_INPUT_US, r['accepted'],
                 r['step_min'], r['step_max']))

    print('\n===== 鲁棒性扫描(对象失配) =====')
    nominal_delay = 2 if DEADZONE else 3
    sweep = {}
    cases = [
        ('标称',     1.0, 1.0, 1.0, 0),
        ('增益+30%', 1.3, 1.0, 1.0, 0),
        ('增益-30%', 0.7, 1.0, 1.0, 0),
        ('tau+30%',  1.0, 1.3, 1.0, 0),
        ('tau-30%',  1.0, 0.7, 1.0, 0),
        ('摩擦+30%', 1.0, 1.0, 1.3, 0),
        ('摩擦-30%', 1.0, 1.0, 0.7, 0),
        ('延迟+1拍', 1.0, 1.0, 1.0, +1),
        ('延迟-1拍', 1.0, 1.0, 1.0, -1),
    ]
    noise = 4.5 if DEADZONE else 0.8
    for label, g, t, f, d in cases:
        r = scenario(sim, 'cycle', (g, t, f, noise, max(0, nominal_delay + d)),
                     suffix='_' + label)
        sweep[label] = {
            'done_ms': worst(r, 'done_ms'), 'overshoot': worst(r, 'overshoot'),
            'final_error': worst(r, 'final_error'),
            'tail_nonzero_pwm': r['tail_nonzero_pwm'], 'tail_motion_pp': r['tail_motion_pp'],
        }
        print('  %-9s 到位%5dms  超调%7.1f  终点误差%7.1f  静止非零PWM=%d'
              % (label, sweep[label]['done_ms'], sweep[label]['overshoot'],
                 sweep[label]['final_error'], sweep[label]['tail_nonzero_pwm']))
    # 噪声加倍单独一档，看的是静止段而不是到位时间
    r = scenario(sim, 'cycle', (1.0, 1.0, 1.0, noise * 2, nominal_delay), suffix='_噪声x2')
    sweep['噪声x2'] = {
        'done_ms': worst(r, 'done_ms'), 'overshoot': worst(r, 'overshoot'),
        'final_error': worst(r, 'final_error'),
        'tail_nonzero_pwm': r['tail_nonzero_pwm'], 'tail_motion_pp': r['tail_motion_pp'],
    }
    print('  %-9s 到位%5dms  超调%7.1f  终点误差%7.1f  静止非零PWM=%d'
          % ('噪声x2', sweep['噪声x2']['done_ms'], sweep['噪声x2']['overshoot'],
             sweep['噪声x2']['final_error'], sweep['噪声x2']['tail_nonzero_pwm']))
    report['sweep'] = sweep

    (OUT / 'summary.json').write_text(json.dumps(report, indent=2, ensure_ascii=False),
                                      encoding='utf-8')
    print('\naudit failures =', failures, '  详细指标见 tests/results/summary.json')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
