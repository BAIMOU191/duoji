"""Fit equal-weight 123123 calibration samples and compare real control code.

Plant profiles stay fixed to each calibration, independently of candidate headers.
The directional first-order dynamics are approximations, not hardware validation.
Run: python tests/fit_batch15.py [--simulate] [--baseline-report fit.json]
"""
import argparse
import csv
import io
import json
import math
from pathlib import Path
import re
import shutil
import statistics as stats
import subprocess
import tempfile

from compare_arrival_bounce import HARNESS, score
from compare_speed_ripple import metrics

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'tests/results/batch15'


def fit(points):
    # PWM is the dependent variable, matching CTRL_KVFF_Q16's units.
    x = stats.mean(v for v, u in points)
    y = stats.mean(u for v, u in points)
    k = sum((v-x)*(u-y) for v, u in points) / sum((v-x)**2 for v, u in points)
    b = y-k*x
    return dict(slope_q16=k*65536, friction_pwm=b,
                max_residual_pwm=max(abs(u-k*v-b) for v, u in points))


def parse(text):
    blocks = text.split('===== SERVO MINIMAL CALIBRATION =====')[1:]
    assert len(blocks) == 6, 'Expected six complete calibrations in 123123 order'
    profiles = []
    for i, b in enumerate(blocks):
        assert '===== END =====' in b
        macros = {k: int(v) for k, v in re.findall(r'#define\s+(\w+)\s+(\d+)', b)}
        points = [list(map(int, m)) for m in re.findall(
            r'\]\s+(700|1120|1540|1960|2380)\s+(\d+)\s+(\d+)', b)]
        assert len(points) == 5
        directional = [fit([(p[d+1], p[0]) for p in points]) for d in range(2)]
        raw_delay = float(re.search(r'pure delay L=([\d.]+)ms', b)[1])
        lt = [float(x) for x in re.findall(r'\[T2\] dir[01].*?L\+tau=([\d.]+)ms', b)]
        fs = [int(x) for x in re.findall(r'dir[01]\s+breakaway=(\d+) PWM', b)]
        assert len(lt) == len(fs) == 2
        profiles.append(dict(name=f'unit{i%3+1}_round{i//3+1}', unit=i%3+1,
                             points=points, directions=directional, static_pwm=fs,
                             tau_ms=[max(1, t-raw_delay) for t in lt],
                             delay=macros['MODEL_DELAY_TICKS'], macros=macros,
                             noise_pp=float(re.search(r'peak-to-peak = ([\d.]+)', b)[1])))
    return profiles


def change_header(header, values):
    for k, v in values.items():
        header, n = re.subn(r'(#define\s+'+k+r'\s+)\d+', lambda m: m[1]+str(v), header)
        assert n == 1, k
    return header


def plant_source(source, p):
    """Replace only hardware plant constants in an isolated simulator copy."""
    d = p['directions']
    source, n = re.subn(r'double fs = .*?;',
        f"double fs = (force >= 0 ? {p['static_pwm'][0]} : {p['static_pwm'][1]}) * friction_scale;", source)
    assert n == 1
    source, n = re.subn(r'double fc = .*?;',
        f"double fc = (plant_sign >= 0 ? {d[0]['friction_pwm']} : {d[1]['friction_pwm']}) * friction_scale;", source)
    assert n == 1
    source, n = re.subn(r'double plant_slope = .*?;',
        f"double plant_slope = plant_sign >= 0 ? {d[0]['slope_q16']} : {d[1]['slope_q16']};", source)
    assert n == 1
    source, n = re.subn(r'/ \(34\.0 / 1000\.0 \* tau_scale\)',
        f"/ ((plant_sign >= 0 ? {p['tau_ms'][0]} : {p['tau_ms'][1]}) / 1000.0 * tau_scale)", source)
    assert n == 1
    return source


def cruise_source(s):
    s = s.replace('static unsigned rng = 1;', 'static unsigned rng = 1;\nstatic int injection;')
    s = s.replace('double p = plant_pos + noise * (2 * random_unit() - 1);',
        'double p = plant_pos + noise * (2 * random_unit() - 1);\n'
        '    if (injection && fabs(plant_vel)>100) p += 20*sin(2*3.141592653589793*25*tick*.001);')
    s = s.replace('double force = u - load_pwm;', 'double force = u - load_pwm;\n'
        '    if (injection && fabs(plant_vel)>100) force -= 100*sin(2*3.141592653589793*15*tick*.001);')
    s = s.replace('    setup(strcmp(mode,', '    if (argc>9) injection=atoi(argv[9]);\n    setup(strcmp(mode,')
    s = s.replace(': strcmp(mode, "boot-high") == 0 ? SIM_HIGH : SIM_MID,',
        ': strcmp(mode, "boot-high") == 0 ? SIM_HIGH : strcmp(mode,"cruise")==0 ? 0 : SIM_MID,')
    s = s.replace('        /* micro：',
        '        if (!strcmp(mode,"cruise") && k%3000==0) { cmd=(k/3000)%2 ? 500 : 2500; A_Servo_Submit(cmd,0); }\n        /* micro：')
    return s


def rows(data):
    return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(io.StringIO(data))]


def simulate(report):
    candidates = {'previous': {}, 'common': report['selected']}
    cases = [(f'd{d}', d, 0, 1, 1, 0, 0) for d in [-500,-100,-20,-10,-5,5,10,20,100,500]]
    cases += [(f'slow_d{d}',d,0,1,1.5,2,0) for d in [-100,100]]
    cases += [(f'load_d{d}',d,0,1,1,0,150) for d in [-100,100]]
    cases += [(f'timed_d{d}',d,1000,1,1,0,0) for d in [-20,20]]
    report['cases'] = cases
    report['results'] = {}
    with tempfile.TemporaryDirectory(prefix='batch15_') as td:
        w = Path(td)
        for folder in ['Application','Common','Drivers','tests/sim']:
            shutil.copytree(ROOT/folder, w/folder)
        (w/'Common/C_Pos_Ctrl.c').write_text(report['controller'], encoding='utf-8')
        hf = w/'tests/sim/arrival.c'
        hf.write_text(HARNESS, encoding='utf-8')
        sf = w/'tests/sim/servo_sim.c'
        inc = ['-I'+str(w/p) for p in ['tests/sim/stubs','Application','Common','Drivers']]
        common = [w/'Application/A_Sensor.c']+[w/'Common'/f'{n}.c' for n in ['C_Pos_Ctrl','C_Speed_Observer','C_Traj_Planner']]
        for name, changes in candidates.items():
            (w/'Application/A_Parameter.h').write_text(change_header(report['baseline_header'], changes), encoding='utf-8')
            result = {}
            for p in report['profiles']:
                source = plant_source(report['plant_source'], p)
                sf.write_text(source, encoding='utf-8')
                ae, ce = w/'arrival.exe', w/'cruise.exe'
                def compile_c(src, exe):
                    subprocess.run(['gcc','-std=c99','-O2',*inc,str(src),*map(str,common),'-lm','-o',str(exe)],check=True,capture_output=True)
                compile_c(hf, ae)
                ar = {}
                for case, delta, duration, gain, tau, delay, load in cases:
                    data = subprocess.check_output([str(ae),*map(str,[delta,duration,gain,tau,1,p['noise_pp']/2,p['delay']+delay,load])], text=True)
                    ar[case] = score(rows(data), delta)
                sf.write_text(cruise_source(source), encoding='utf-8')
                compile_c(sf, ce)
                cr = {}
                for case, gain, injection in [('nominal',1,0),('mixed',1,1),('gain_low',.85,1)]:
                    data = subprocess.check_output([str(ce),'cruise',*map(str,[gain,1,1,p['noise_pp']/2,p['delay'],1,0,injection])],text=True)
                    cr[case] = metrics(rows(data), 'cruise')
                    if case == 'nominal': (OUT/f'{name}_{p["name"]}_cruise.csv').write_text(data,encoding='utf-8')
                result[p['name']] = dict(arrival=ar, cruise=cr)
            report['results'][name] = result
            summary = dict(max_tail_motion=max(a['tail_pp_deg'] for p in result.values() for a in p['arrival'].values()),
                max_final_error=max(abs(a['final_error_deg']) for p in result.values() for a in p['arrival'].values()),
                speeds=[p['cruise']['nominal']['cruise_actual_mean'] for p in result.values()])
            print(name, json.dumps(summary), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--simulate', action='store_true')
    ap.add_argument('--baseline-report', type=Path)
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    profiles = parse((ROOT/'tests/fixtures/calibration_15kg_123123.log').read_text(encoding='utf-8'))
    def points_for(ps): return [(v, p[0]) for q in ps for p in q['points'] for v in p[1:]]
    pooled = fit(points_for(profiles))
    rounded = lambda x: int(math.floor(x+.5))
    selected = dict(CAL_SPEED_SLOPE_Q16=rounded(pooled['slope_q16']), CAL_FRICTION_PWM=rounded(pooled['friction_pwm']),
        CAL_BREAKAWAY_PWM=max(max(p['static_pwm']) for p in profiles),
        MODEL_TAU_MS=rounded(stats.mean(p['macros']['MODEL_TAU_MS'] for p in profiles)),
        MODEL_DELAY_TICKS=max(p['delay'] for p in profiles),
        MODEL_BRAKE_GAIN_Q8=rounded(stats.mean(p['macros']['MODEL_BRAKE_GAIN_Q8'] for p in profiles)),
        TUNE_RESOLUTION_CDEG=max(p['macros']['TUNE_RESOLUTION_CDEG'] for p in profiles))
    prior = json.loads(args.baseline_report.read_text(encoding='utf-8')) if args.baseline_report else {}
    report = dict(profiles=profiles, pooled=pooled, selected=selected,
        leave_one_unit_out={str(i): fit(points_for([p for p in profiles if p['unit'] != i])) for i in [1,2,3]},
        baseline_header=prior.get('baseline_header') or (ROOT/'Application/A_Parameter.h').read_text(encoding='utf-8'),
        controller=prior.get('controller') or (ROOT/'Common/C_Pos_Ctrl.c').read_text(encoding='utf-8'),
        plant_source=prior.get('plant_source') or (ROOT/'tests/sim/servo_sim.c').read_text(encoding='utf-8'))
    print(json.dumps({'fit':pooled,'selected':selected},indent=2), flush=True)
    if args.simulate: simulate(report)
    (OUT/'fit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')


if __name__ == '__main__':
    main()
