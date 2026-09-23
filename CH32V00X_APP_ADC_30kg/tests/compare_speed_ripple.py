"""Compare tuning candidates in isolated copies; never edit production parameters.

Run with Python and host gcc. Outputs: tests/results/ripple_comparison/.
The simulated plant is only a first-order approximation. Injected periodic sensor
error and torque disturbance are sensitivity tests, not identified hardware data.
"""
import argparse
import csv
import io
import json
import pathlib
import re
import shutil
import statistics
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
OUT = ROOT / 'tests/results/ripple_comparison'
CANDIDATES = {
    'baseline': {},
    'speed88': {'TUNE_SPEED_PCT': 88},
    'speed92': {'TUNE_SPEED_PCT': 92, 'DSGC_FF_MARGIN_PWM': 180},
    'speed95': {'TUNE_SPEED_PCT': 95, 'DSGC_FF_MARGIN_PWM': 110},
    'quiet85': {'TUNE_OBS_BW_HZ': 9, 'TUNE_DAMPING': 20},
    'quiet92': {'TUNE_SPEED_PCT': 92, 'DSGC_FF_MARGIN_PWM': 180,
                'TUNE_OBS_BW_HZ': 9, 'TUNE_DAMPING': 20},
    'soft92': {'TUNE_SPEED_PCT': 92, 'DSGC_FF_MARGIN_PWM': 180,
               'TUNE_OBS_BW_HZ': 9, 'TUNE_STIFFNESS_RADS': 7,
               'TUNE_INTEGRAL_RADS': 7},
    'middle92': {'TUNE_SPEED_PCT': 92, 'DSGC_FF_MARGIN_PWM': 180,
                 'TUNE_OBS_BW_HZ': 10, 'TUNE_STIFFNESS_RADS': 8},
}
# name, mode, gain, tau ratio, friction ratio, uniform noise amplitude (cdeg), delay, injection
CASES = [
    ('nominal', 'cruise', 1, 1, 1, 5.21, 0, 0),
    ('sensor', 'cruise', 1, 1, 1, 5.21, 0, 1),
    ('torque', 'cruise', 1, 1, 1, 5.21, 0, 2),
    ('mixed', 'cruise', 1, 1, 1, 5.21, 0, 3),
    ('slow_tau', 'cruise', 1, 1.27, 1, 5.21, 0, 3),
    ('fast_tau', 'cruise', 1, .64, 1, 5.21, 0, 3),
    ('gain_low', 'cruise', .85, 1, 1, 5.21, 0, 3),
    ('delay2', 'cruise', 1, 1, 1, 5.21, 2, 3),
    ('micro', 'micro', 1, 1, 1, 5.21, 0, 0),
    ('flip', 'flip', 1, 1, 1, 5.21, 0, 0),
    ('load', 'load', 1, 1, 1, 5.21, 0, 0),
]


def metrics(rows, mode):
    result = {'max_pwm': max(abs(r['pwm']) for r in rows)}
    if mode == 'cruise':
        windows, moves = [], []
        for start, end, target in [(0, 3000, 27000), (3000, 6000, 0)]:
            a = rows[start:end]
            sign = 1 if target > a[0]['pos'] else -1
            vmax = max(abs(r['ref_vel']) for r in a)
            flat = [r for r in a if abs(r['ref_vel']) >= vmax-1 and r['ref_acc'] == 0]
            if flat:
                flat = [r for r in flat if r['ms'] >= flat[0]['ms']+200
                        and r['ms'] <= flat[-1]['ms']-100]
            windows.extend(flat)
            last_bad = max((i for i, r in enumerate(a) if abs(r['pos']-target) > 50), default=-1)
            moves.append({
                'reference_ms': next((i for i, r in enumerate(a) if r['done']), None),
                'settle_0_5deg_ms': last_bad+1 if last_bad < len(a)-1 else None,
                'overshoot_deg': round(max(0, max(sign*(r['pos']-target) for r in a))/100, 3),
                'tail_error_deg': round((a[-1]['pos']-target)/100, 3),
                'tail_pwm_samples': sum(r['pwm'] != 0 for r in a[-200:]),
            })
        result['moves'] = moves
        if windows:
            # Fold direction so both cruises share the same positive-speed axis.
            actual = [r['vel']*(1 if r['ref_vel'] > 0 else -1)/100 for r in windows]
            observed = [r['obs_vel']*(1 if r['ref_vel'] > 0 else -1)/100 for r in windows]
            result.update(cruise_actual_mean=round(statistics.mean(actual), 3),
                          actual_ripple_std=round(statistics.pstdev(actual), 3),
                          observer_ripple_std=round(statistics.pstdev(observed), 3),
                          pwm_std=round(statistics.pstdev(abs(r['pwm']) for r in windows), 2),
                          cruise_saturation_pct=round(100*sum(r['sat'] != 0 for r in windows)/len(windows), 2))
    else:
        result['tail_motion_deg'] = round((max(r['pos'] for r in rows[-200:])-min(r['pos'] for r in rows[-200:]))/100, 4)
        result['tail_pwm_samples'] = sum(r['pwm'] != 0 for r in rows[-200:])
        if mode == 'micro':
            result['max_step_error_deg'] = round(max(abs(rows[k]['pos']-(rows[k]['pwm_target']-500)*13.5) for k in range(599, len(rows), 600))/100, 3)
        if mode == 'flip':
            result['max_pwm_jump'] = max(abs(a['pwm']-b['pwm']) for a,b in zip(rows[1500:8999],rows[1501:9000]))
        if mode == 'load':
            result['tail_error_deg'] = round((rows[-1]['pos']-13500)/100,3)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-report', type=pathlib.Path,
                        help='Reuse the saved baseline_header from an earlier comparison.json')
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    report = {'notes': 'Sensor injection: 20 cdeg sine at 25 Hz while moving. Torque injection: 100 PWM sine at 15 Hz while moving. Not measured hardware identification.', 'candidates': {}}
    report['baseline_header'] = (json.loads(args.baseline_report.read_text(encoding='utf-8'))['baseline_header']
                                 if args.baseline_report else (ROOT/'Application/A_Parameter.h').read_text(encoding='utf-8'))
    with tempfile.TemporaryDirectory(prefix='servo30_ripple_') as temp:
        work = pathlib.Path(temp)
        for folder in ['Application', 'Common', 'Drivers', 'tests/sim']:
            shutil.copytree(ROOT/folder, work/folder)
        simfile = work/'tests/sim/servo_sim.c'
        sim = simfile.read_text(encoding='utf-8')
        sim = sim.replace('static unsigned rng = 1;', 'static unsigned rng = 1;\nstatic int injection;')
        sim = sim.replace('double p = plant_pos + noise * (2 * random_unit() - 1);', 'double p = plant_pos + noise * (2 * random_unit() - 1);\n    if ((injection & 1) && fabs(plant_vel) > 100) p += 20 * sin(2*3.141592653589793*25*tick*.001);')
        sim = sim.replace('double force = u - load_pwm;', 'double force = u - load_pwm;\n    if ((injection & 2) && fabs(plant_vel) > 100) force -= 100 * sin(2*3.141592653589793*15*tick*.001);')
        # Keep measured breakaway directions in the model; no change to production simulator.
        sim = sim.replace('(force >= 0 ? CAL_BREAKAWAY_PWM : CAL_BREAKAWAY_PWM * 0.77)', '(force >= 0 ? 219 : 235)')
        sim = sim.replace('    setup(strcmp(mode,', '    if (argc > 9) injection = atoi(argv[9]);\n    setup(strcmp(mode,')
        sim = sim.replace(': strcmp(mode, "boot-high") == 0 ? SIM_HIGH : SIM_MID,', ': strcmp(mode, "boot-high") == 0 ? SIM_HIGH : strcmp(mode, "cruise") == 0 ? 0 : SIM_MID,')
        sim = sim.replace('        /* micro：', '        if (strcmp(mode, "cruise") == 0 && k % 3000 == 0) { cmd = (k/3000)%2 ? 500 : 2500; A_Servo_Submit(cmd, 0); }\n        /* micro：')
        simfile.write_text(sim, encoding='utf-8')
        includes = ['-I'+str(work/p) for p in ['tests/sim/stubs','Application','Common','Drivers']]
        sources = [simfile,work/'Application/A_Sensor.c']+[work/'Common'/f'{n}.c' for n in ['C_Pos_Ctrl','C_Speed_Observer','C_Traj_Planner']]
        for name, changes in CANDIDATES.items():
            header = report['baseline_header']
            for key, value in changes.items():
                header, n = re.subn(r'(#define\s+'+key+r'\s+)\d+', lambda m:m[1]+str(value), header)
                assert n == 1, key
            (work/'Application/A_Parameter.h').write_text(header, encoding='utf-8')
            exe = work/(name+'.exe')
            subprocess.run(['gcc','-std=c99','-O2',*includes,*map(str,sources),'-lm','-o',str(exe)], check=True, capture_output=True)
            results = {}
            for case, mode, g, tau, fric, noise, delay, injection in CASES:
                data = subprocess.check_output([str(exe),mode,*map(str,[g,tau,fric,noise,delay,1,0,injection])],text=True)
                rows = [{k:float(v) for k,v in row.items()} for row in csv.DictReader(io.StringIO(data))]
                results[case] = metrics(rows, mode)
                if case in ('mixed', 'sensor'):
                    (OUT/f'{name}_{case}.csv').write_text(data,encoding='utf-8')
            report['candidates'][name] = {'changes':changes,'results':results}
            print(name, json.dumps({k:results[k] for k in ['mixed','gain_low','micro']}, ensure_ascii=False), flush=True)
    (OUT/'comparison.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
    print('Saved', OUT/'comparison.json')


if __name__ == '__main__':
    main()
