"""Compare terminal rebound after small angle commands, using isolated production copies."""
import argparse, csv, io, json, pathlib, re, shutil, subprocess, tempfile
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

ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=ROOT/'tests/results/arrival_bounce'
VARIANTS={
    'baseline':({},False),
    'arm_after_quiet':({},True),
    'hold8':({'TUNE_HOLD_STIFF':8},False),
    'decel60':({'TUNE_SMOOTH_DEC':60},False),
    'gentle':({'TUNE_MOVE_MIN_MS':80,'TUNE_ACCEL_MIN_MS':50,'TUNE_SMOOTH_ACC':40},False),
    'decel80':({'TUNE_SMOOTH_DEC':80},False),
    'arm_settled':({},'settled'),
    'dec96':({'DSGC_DEC_OF_ACC_Q8':96},'settled'),
    'dec64':({'DSGC_DEC_OF_ACC_Q8':64},'settled'),
    'dec96_speed88':({'DSGC_DEC_OF_ACC_Q8':96,'TUNE_SPEED_PCT':88},'settled'),
    'dec64_speed88':({'DSGC_DEC_OF_ACC_Q8':64,'TUNE_SPEED_PCT':88},'settled'),
    'settled_hold8':({'TUNE_HOLD_STIFF':8},'settled'),
    'settled_gentle':({'TUNE_MOVE_MIN_MS':80,'TUNE_ACCEL_MIN_MS':50,'TUNE_SMOOTH_ACC':40},'settled'),
}

def score(rows, delta):
    a=rows[500:]; target=13500+delta*13.5;sign=1 if delta>0 else -1
    finish=next((i for i,r in enumerate(a) if r['done']),len(a)-1)
    after=a[finish:]
    reverse=sum(max(0,-sign*(b['pos']-c['pos'])) for c,b in zip(a,a[1:]))/100
    last_motion=max((i for i,r in enumerate(a) if abs(r['vel'])>20),default=-1)
    last_bad=max((i for i,r in enumerate(a) if abs(r['pos']-target)>50),default=-1)
    return dict(ref_ms=finish,overshoot_deg=round(max(0,max(sign*(r['pos']-target) for r in a))/100,4),
                reverse_travel_deg=round(reverse,4),
                reverse_peak_dps=round(max(0,max(-sign*r['vel'] for r in after))/100,3),
                final_error_deg=round((a[-1]['pos']-target)/100,4),
                last_motion_ms=last_motion,
                settle_ms=last_bad+1 if last_bad<len(a)-1 else None,
                tail_pp_deg=round((max(r['pos'] for r in a[-1000:])-min(r['pos'] for r in a[-1000:]))/100,4))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variants',nargs='+',choices=list(VARIANTS))
    parser.add_argument('--baseline-report',type=pathlib.Path)
    parser.add_argument('--output', type=pathlib.Path, default=OUT)
    opts=parser.parse_args()
    out=opts.output
    out.mkdir(parents=True,exist_ok=True)
    prior=json.loads(opts.baseline_report.read_text(encoding='utf-8')) if opts.baseline_report else None
    header=prior['baseline_header'] if prior else (ROOT/'Application/A_Parameter.h').read_text(encoding='utf-8')
    controller=prior['baseline_controller'] if prior else (ROOT/'Common/C_Pos_Ctrl.c').read_text(encoding='utf-8')
    plant = prior.get('plant_source') if prior else None
    if not plant: plant = (ROOT/'tests/sim/servo_sim.c').read_text(encoding='utf-8')
    cases=[]
    for delta in [5,10,20,50,100,150,-10,-50,-100,-150,500]:
        for name,tau,delay,load in [('nominal',1,3,0),('slow',(63/34),3,0),('fast',(7/34),3,0),('load',1,3,150),('delay',1,8,0),('slow_delay',(63/34),8,0),('no_delay',1,0,0)]:
            cases.append((f'{name}_d{delta}',delta,0,1,tau,1,7.69,delay,load))
    report={'baseline_header':header,'baseline_controller':controller,'plant_source':plant,'cases':cases,'variants':{}}
    with tempfile.TemporaryDirectory(prefix='servo15_arrival_') as temp:
        work=pathlib.Path(temp)
        for folder in ['Application','Common','Drivers','tests/sim']:shutil.copytree(ROOT/folder,work/folder)
        (work/'tests/sim/servo_sim.c').write_text(plant,encoding='utf-8')
        hf=work/'tests/sim/arrival.c';hf.write_text(HARNESS,encoding='utf-8')
        inc=['-I'+str(work/p) for p in ['tests/sim/stubs','Application','Common','Drivers']]
        sources=[hf,work/'Application/A_Sensor.c']+[work/'Common'/f'{n}.c' for n in ['C_Pos_Ctrl','C_Speed_Observer','C_Traj_Planner']]
        for name,(settings,arm) in VARIANTS.items():
            if opts.variants and name not in opts.variants:continue
            h=header;c=controller
            for key,value in settings.items():h=re.sub(r'(#define\s+'+key+r'\s+)\d+',lambda m:m[1]+str(value),h)
            if arm=='settled':
                c=c.replace('    else if (s_ctl.state == (uint8_t)POSCTRL_HOLD) s_ctl.held = 1U;',
                    '    else if (s_ctl.state == (uint8_t)POSCTRL_HOLD\n'
                    '             && ae_measured <= CTRL_HOLD_DEADBAND\n'
                    '             && av_obs <= CTRL_VEL_DB_CDPS) s_ctl.held = 1U;')
            elif arm:
                c=c.replace('    else if (s_ctl.state == (uint8_t)POSCTRL_HOLD) s_ctl.held = 1U;','')
                c=c.replace('        s_ctl.quiet = 1U;','        s_ctl.quiet = 1U;\n    if (s_ctl.quiet) s_ctl.held = 1U;')
            (work/'Application/A_Parameter.h').write_text(h,encoding='utf-8')
            (work/'Common/C_Pos_Ctrl.c').write_text(c,encoding='utf-8')
            exe=work/(name+'.exe')
            subprocess.run(['gcc','-std=c99','-O2',*inc,*map(str,sources),'-lm','-o',str(exe)],check=True,capture_output=True)
            results={}
            for case,*args in cases:
                data=subprocess.check_output([str(exe),*map(str,args)],text=True)
                rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(io.StringIO(data))]
                results[case]=score(rows,args[0])
                if case in ['slow_d100','slow_d150','nominal_d10','load_d100','slow_d-100']:
                    (out/f'{name}_{case}.csv').write_text(data,encoding='utf-8')
            report['variants'][name]={'settings':settings,'arm':arm,'results':results}
            print(name,json.dumps({k:results[k] for k in ['nominal_d10','slow_d100','slow_d150','load_d100']},ensure_ascii=False),flush=True)
    dest=out/('comparison_selected.json' if opts.variants else 'comparison.json')
    dest.write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
    print('Saved',dest)

if __name__=='__main__':main()
