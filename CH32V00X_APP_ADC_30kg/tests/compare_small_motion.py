"""Isolated small-motion sensitivity comparison using the production control code.

The first-order plant cannot represent gearbox compliance or prove a hardware fix.
Output contains the baseline source snapshots and actual simulated shaft motion.
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
OUT = ROOT/'tests/results/small_motion'
VARIANTS = {
    'baseline': ({}, None),
    'gentle_start': ({'TUNE_MOVE_MIN_MS':80,'TUNE_ACCEL_MIN_MS':50,'TUNE_SMOOTH_ACC':40},None),
    'less_kick': ({'DSGC_KICK_MAX_PCT':20,'DSGC_KICK_RISE_MS':100},None),
    'less_integral': ({'TUNE_INTEGRAL_RADS':4},None),
    'soft_hold': ({'TUNE_HOLD_STIFF':6},None),
    'earlier_slide': ({'DSGC_FRIC_MOVE_X':2,'DSGC_VEL_DB_X':1},None),
    'stiction_gate': ({'DSGC_FRIC_MOVE_X':2}, 'gate'),
    'friction_only': ({}, 'friction_only'),
    'load_guard': ({}, 'load_guard'),
}
HARNESS = r'''
#define main previous_main
#include "servo_sim.c"
#undef main
int main(int argc, char **argv) {
    int delta = atoi(argv[1]), duration = atoi(argv[2]);
    gain = atof(argv[3]); tau_scale = atof(argv[4]);
    friction_scale = atof(argv[5]); noise = atof(argv[6]);
    delay_ticks = atoi(argv[7]);
    setup(SIM_MID, 0); load_pwm = atof(argv[8]);
    puts("ms,pos,vel,ref,ref_vel,pwm,done,state,integral,fric,acc_ff,fb,obs_vel");
    for (int k = 0; k < 4000; ++k) {
        if (k == 500) A_Servo_Submit((uint16_t)(1500+delta), (uint16_t)duration);
        step();
        printf("%d,%.5f,%.4f,%ld,%ld,%d,%u,%u,%d,%d,%d,%d,%ld\n", k,
            plant_pos, plant_vel, (long)s_servo.ref.pos,(long)s_servo.ref.vel,
            applied,C_Traj_Is_Done(),s_servo.dbg.state,s_servo.dbg.u_i,
            s_servo.dbg.u_fric,s_servo.dbg.u_acc,s_servo.dbg.u_fb,(long)s_servo.obs.vel);
    }
    return 0;
}
'''


def score(rows, delta, duration):
    a = rows[500:]; target = 13500+delta*13.5; sign = 1 if delta > 0 else -1
    moving = [r for r in a if not r['done']]
    cruise = [r for r in moving if r['ms'] > 600 and r['ms'] < 500+duration-100]
    reversals = 0; prev = 0
    for r in a:
        s = 1 if r['vel'] > 20 else -1 if r['vel'] < -20 else 0
        if s:
            if prev and prev != s: reversals += 1
            prev = s
    tail = a[-1000:]
    result = {
        'reversals':reversals,
        'peak_speed_dps':round(max(abs(r['vel']) for r in a)/100,3),
        'overshoot_deg':round(max(0,max(sign*(r['pos']-target) for r in a))/100,4),
        'final_error_deg':round((a[-1]['pos']-target)/100,4),
        'tail_motion_deg':round((max(r['pos'] for r in tail)-min(r['pos'] for r in tail))/100,4),
        'tail_pwm_samples':sum(r['pwm']!=0 for r in tail),
        'max_tracking_error_deg':round(max(abs(r['pos']-r['ref']) for r in a)/100,4),
    }
    if cruise:
        result.update(speed_std_dps=round(statistics.pstdev(r['vel']/100 for r in cruise),4),
                      stopped_pct=round(100*sum(abs(r['vel'])<20 for r in cruise)/len(cruise),2),
                      cruise_mean_dps=round(statistics.mean(r['vel']/100 for r in cruise),4))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variants',nargs='+',choices=list(VARIANTS))
    parser.add_argument('--baseline-report',type=pathlib.Path)
    args=parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)
    previous=json.loads(args.baseline_report.read_text(encoding='utf-8')) if args.baseline_report else None
    header=previous['baseline_header'] if previous else (ROOT/'Application/A_Parameter.h').read_text(encoding='utf-8')
    controller=previous['baseline_controller'] if previous else (ROOT/'Common/C_Pos_Ctrl.c').read_text(encoding='utf-8')
    cases=[]
    for delta in [3,5,10,20,100,-5,-10,-20]:
        for duration in [0,200,1000]:
            cases.append((f'd{delta}_t{duration}',delta,duration,1,1,1,5.21,0,0))
    for name,g,t,f,n,d,load in [('slow_tau',1,1.27,1,5.21,0,0),('fast_tau',1,.64,1,5.21,0,0),
            ('friction_high',1,1,1.3,5.21,0,0),('noise2',1,1,1,10.42,0,0),
            ('delay2',1,1,1,5.21,2,0),('loaded',1,1,1,5.21,0,150)]:
        for delta,duration in [(5,1000),(10,1000),(10,0),(-10,1000)]:
            cases.append((f'{name}_d{delta}_t{duration}',delta,duration,g,t,f,n,d,load))
    report={'baseline_header':header,'baseline_controller':controller,'cases':cases,'variants':{}}
    with tempfile.TemporaryDirectory(prefix='servo30_small_') as temp:
        work=pathlib.Path(temp)
        for folder in ['Application','Common','Drivers','tests/sim']:
            shutil.copytree(ROOT/folder,work/folder)
        simfile=work/'tests/sim/servo_sim.c'
        sim=simfile.read_text(encoding='utf-8').replace('(force >= 0 ? CAL_BREAKAWAY_PWM : CAL_BREAKAWAY_PWM * 0.77)', '(force >= 0 ? 219 : 235)')
        simfile.write_text(sim,encoding='utf-8')
        harness=work/'tests/sim/small_motion.c'; harness.write_text(HARNESS,encoding='utf-8')
        inc=['-I'+str(work/p) for p in ['tests/sim/stubs','Application','Common','Drivers']]
        sources=[harness,work/'Application/A_Sensor.c']+[work/'Common'/f'{n}.c' for n in ['C_Pos_Ctrl','C_Speed_Observer','C_Traj_Planner']]
        for name,(settings,algorithm) in VARIANTS.items():
            if args.variants and name not in args.variants:continue
            text=header
            for key,value in settings.items():
                text,n=re.subn(r'(#define\s+'+key+r'\s+)\d+',lambda m:m[1]+str(value),text)
                assert n==1,key
            (work/'Application/A_Parameter.h').write_text(text,encoding='utf-8')
            c=controller
            if algorithm=='gate':
                c=c.replace('else if (av_obs <= CTRL_VEL_DB_CDPS)\n', 'else if (av_obs <= CTRL_VEL_DB_CDPS / 4)\n',1)
            elif algorithm=='friction_only':
                c=c.replace('if (av_obs >= CTRL_FRIC_MOVE_CDPS)\n', 'if (av_obs >= DSG_VEL_NOISE_CDPS * 2)\n',1)
                c=c.replace('else if (av_obs <= CTRL_VEL_DB_CDPS)\n', 'else if (av_obs <= DSG_VEL_NOISE_CDPS)\n',1)
            elif algorithm=='load_guard':
                c=c.replace('int32_t av_obs, int32_t u_rest)', 'int32_t av_obs, int32_t u_rest, int32_t load)')
                c=c.replace('if (av_obs >= CTRL_FRIC_MOVE_CDPS)\n', 'if (av_obs >= ((PosCtrl_Abs32(load) >= CTRL_FRIC_STATIC / 2) ? CTRL_FRIC_MOVE_CDPS : DSG_VEL_NOISE_CDPS * 2))\n',1)
                c=c.replace('else if (av_obs <= CTRL_VEL_DB_CDPS)\n', 'else if (av_obs <= ((PosCtrl_Abs32(load) >= CTRL_FRIC_STATIC / 2) ? CTRL_VEL_DB_CDPS : DSG_VEL_NOISE_CDPS))\n',1)
                c=c.replace('u_vel + u_acc + u_fb + u_i + u_spring);', 'u_vel + u_acc + u_fb + u_i + u_spring, obs->load);')
            (work/'Common/C_Pos_Ctrl.c').write_text(c,encoding='utf-8')
            exe=work/(name+'.exe')
            subprocess.run(['gcc','-std=c99','-O2',*inc,*map(str,sources),'-lm','-o',str(exe)],check=True,capture_output=True)
            results={}
            for case,*case_args in cases:
                data=subprocess.check_output([str(exe),*map(str,case_args)],text=True)
                rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(io.StringIO(data))]
                results[case]=score(rows,case_args[0],case_args[1])
                if case in ['d5_t1000','d10_t1000','d10_t0','friction_high_d10_t1000','loaded_d10_t1000']:
                    (OUT/f'{name}_{case}.csv').write_text(data,encoding='utf-8')
            report['variants'][name]={'settings':settings,'algorithm':algorithm,'results':results}
            print(name,json.dumps({k:results[k] for k in ['d5_t1000','d10_t1000','d10_t0','loaded_d10_t1000']},ensure_ascii=False),flush=True)
    dest=OUT/('comparison_selected.json' if args.variants else 'comparison.json')
    dest.write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
    print('Saved',dest)


if __name__=='__main__':main()
