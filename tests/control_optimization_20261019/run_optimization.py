"""Isolated first-stage control optimization and paired simulations.

No production source is written. All configurations are fixed at build time.
"""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
import subprocess
import shutil
import re
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OLD = ROOT / 'tests/control_chain_review_20261019'
OUT = HERE / 'output'
CC = Path(r'D:\SoftWare\mingw64\bin\gcc.exe')
PROFILES = {
    'baseline': {},
    'capture': {'OPT_STILL_MS': 200},
    'feedback': {'OPT_STILL_MS': 200, 'OPT_CORR_PCT': 100},
    'inertia1': {'OPT_STILL_MS': 200, 'OPT_CORR_PCT': 100, 'OPT_J_X10': 10},
    'inertia2': {'OPT_STILL_MS': 200, 'OPT_CORR_PCT': 100, 'OPT_J_X10': 20},
    'limits': {'OPT_STILL_MS': 200, 'OPT_CORR_PCT': 100, 'OPT_J_X10': 10, 'OPT_EMF_PCT': 100},
    'load_profile': {'OPT_STILL_MS': 200, 'OPT_CORR_PCT': 100, 'OPT_J_X10': 10, 'OPT_EMF_PCT': 100, 'OPT_ACC_PCT': 50},
    'load25': {'OPT_STILL_MS': 200, 'OPT_CORR_PCT': 100, 'OPT_J_X10': 10, 'OPT_ACC_PCT': 25},
    'load10': {'OPT_STILL_MS': 200, 'OPT_CORR_PCT': 100, 'OPT_J_X10': 10, 'OPT_ACC_PCT': 10},
    'load25v70': {'OPT_STILL_MS': 200, 'OPT_CORR_PCT': 100, 'OPT_J_X10': 10, 'OPT_ACC_PCT': 25, 'OPT_VEL_PCT': 70},
    'hold_i10': {'OPT_STILL_MS': 200, 'OPT_CORR_PCT': 100, 'OPT_J_X10': 10, 'RUN_HOLD_KI': 10},
    'stream': {'OPT_STILL_MS': 200, 'OPT_CORR_PCT': 100, 'OPT_J_X10': 10, 'OPT_STREAM': 1},
    'optimized': {'OPT_STILL_MS': 200, 'OPT_CORR_PCT': 100, 'OPT_J_X10': 10, 'OPT_ACC_PCT': 25, 'OPT_VEL_PCT': 70, 'OPT_STREAM': 1},
}
for _j in (0,20,40):
    PROFILES[f'opt_j{_j//10}']={**PROFILES['optimized'],'OPT_J_X10':_j}
for _ki in (10,30,60):
    PROFILES[f'opt_hold_i{_ki}']={**PROFILES['optimized'],'RUN_HOLD_KI':_ki}
PROFILES['handoff']={**PROFILES['optimized'],'OPT_BLEND_MS':50}


def replace(text, old, new, count=1):
    assert text.count(old) == count, (old[:120], text.count(old), count)
    return text.replace(old, new)


def run(args, log):
    log.parent.mkdir(parents=True, exist_ok=True)
    result = subprocess.run([str(x) for x in args], cwd=ROOT, capture_output=True,
                            text=True, encoding='utf-8', errors='replace')
    log.write_text(result.stdout + result.stderr, encoding='utf-8')
    if result.returncode:
        raise RuntimeError(f'{log}\n{result.stdout[-2000:]}\n{result.stderr[-4000:]}')
    return result.stdout


def parse(output):
    result = {}
    for line in output.splitlines():
        if not line or line.startswith('CAL,'):
            continue
        fields = line.split(',')
        assert len(fields) > 5, line
        row = {k: float(v) for k, v in (field.split('=', 1) for field in fields[3:])}
        assert all(__import__('math').isfinite(v) for v in row.values()), line
        key = '|'.join(fields[1:3])
        assert key not in result, key
        result[key] = row
    return result


def prepare():
    OUT.mkdir(parents=True, exist_ok=True)
    manifest = {}
    for kg in (20,30):
        for rel in ('Common/C_Pos_Ctrl.c','Common/C_Pos_Ctrl.h','Application/A_Servo.c'):
            assert (ROOT/'CH32V00X_APP_ADC_15kg'/rel).read_text(encoding='utf-8')==(ROOT/f'CH32V00X_APP_ADC_{kg}kg'/rel).read_text(encoding='utf-8'), rel
    for kg in (15, 20, 30):
        original = ROOT / f'CH32V00X_APP_ADC_{kg}kg'
        stage = HERE / 'source/baseline' / f'{kg}kg'
        manifest[original.name] = {}
        for directory in ('Application', 'Common', 'Drivers', 'System', 'User', 'SRC'):
            shutil.copytree(original / directory, stage / directory, dirs_exist_ok=True)
            for file in (original / directory).rglob('*'):
                if file.suffix in ('.c', '.h', '.S', '.ld'):
                    manifest[original.name][file.relative_to(original).as_posix()] = hashlib.sha256(file.read_bytes()).hexdigest()
    (OUT / 'production_hashes_start.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    (OUT / 'profiles.json').write_text(json.dumps(PROFILES, indent=2), encoding='utf-8')
    candidate = HERE / 'source/candidate'
    candidate.mkdir(parents=True, exist_ok=True)
    src = HERE / 'source/baseline/15kg'
    for directory, names in {'Common': ('C_Pos_Ctrl.c', 'C_Pos_Ctrl.h'), 'Application': ('A_Servo.c',)}.items():
        for name in names:
            shutil.copy2(src / directory / name, candidate / name)
    defaults = {'OPT_STILL_MS': 0, 'OPT_CORR_PCT': 0, 'OPT_J_X10': 0,
                'OPT_EMF_PCT': 0, 'OPT_ACC_PCT': 100, 'OPT_VEL_PCT': 100, 'OPT_STREAM': 0, 'OPT_BLEND_MS': 0}
    options = '#ifndef CONTROL_OPT_OPTIONS_H\n#define CONTROL_OPT_OPTIONS_H\n'
    for name, value in defaults.items():
        options += f'#ifndef {name}\n#define {name} {value}\n#endif\n'
    options += '_Static_assert(OPT_J_X10>=0 && OPT_J_X10<=40,"fixed inertia range");\n'
    options += '_Static_assert(OPT_ACC_PCT>=10 && OPT_ACC_PCT<=100,"acceleration ratio range");\n'
    options += '_Static_assert(OPT_BLEND_MS>=0 && OPT_BLEND_MS<=500,"handoff duration range");\n#endif\n'
    (candidate / 'C_Control_Options.h').write_text(options, encoding='utf-8')

    ctrl = (candidate / 'C_Pos_Ctrl.c').read_text(encoding='utf-8')
    ctrl = replace(ctrl, '#include <string.h>', '#include <string.h>\n#include "C_Control_Options.h"')
    ctrl = replace(ctrl, 'uint16_t cap_ms;', 'uint16_t cap_ms, hold_blend;\n    uint8_t qualified;')
    ctrl = replace(ctrl, '    s_gain.kp_q8  = g->pos_bw * 256;', '''#if OPT_J_X10 > 0
    /* Bound outer bandwidth using total fixed inertia; do not amplify acceleration FF. */
    {
        int64_t damping = s_gain.kv_q16 + ((int64_t)s_gain.kv_uv * 65536) / 1000;
        int64_t inertia = (int64_t)s_gain.ka_nv * (10 + OPT_J_X10) * 65536 / 10000000;
        int32_t max_pos = (int32_t)(damping / inertia) / 4;
        if (max_pos < 1) max_pos = 1;
        if (g->pos_bw > max_pos) g->pos_bw = (uint16_t)max_pos;
        if (g->int_bw > g->pos_bw) g->int_bw = g->pos_bw;
    }
#endif
    s_gain.kp_q8  = g->pos_bw * 256;''')
    ctrl = replace(ctrl, 'uint8_t C_PosCtrl_Reached(void) { return (uint8_t)(s_ctrl.state == PC_HOLD); }', '''uint8_t C_PosCtrl_Reached(void) { return (uint8_t)(s_ctrl.state == PC_HOLD); }
uint8_t C_PosCtrl_ArrivalQualified(void) { return s_ctrl.qualified; }
void C_PosCtrl_BeginMotion(void) { s_ctrl.cap_ms = 0U; s_ctrl.qualified = 0U; }''')
    ctrl = replace(ctrl, '    if (!ref->done) { s_ctrl.state = PC_MOVE; return; }', '''    if (!ref->done) {
        s_ctrl.qualified = 0U;
        s_ctrl.cap_ms = 0U;
        s_ctrl.hold_blend = 0U;
        s_ctrl.state = PC_MOVE;
        return;
    }''')
    ctrl = replace(ctrl, '        if (++s_ctrl.cap_ms >= CAPTURE_MS) s_ctrl.state = PC_HOLD;   /* 积分原样带进保持 */', '''        if (s_ctrl.cap_ms < CAPTURE_MS) s_ctrl.cap_ms++;
        if (s_ctrl.cap_ms >= CAPTURE_MS) {
            s_ctrl.qualified = 1U;
            if (s_ctrl.still_ms >= OPT_STILL_MS) {
                if(s_ctrl.hold_blend < OPT_BLEND_MS)s_ctrl.hold_blend++;
                else s_ctrl.state = PC_HOLD;
            }else if(s_ctrl.hold_blend)s_ctrl.hold_blend--;
        }''')
    ctrl = replace(ctrl, '    else s_ctrl.cap_ms = (s_ctrl.cap_ms > 2U) ? (uint16_t)(s_ctrl.cap_ms - 2U) : 0U;', '''    else {
        s_ctrl.cap_ms = (s_ctrl.cap_ms > 2U) ? (uint16_t)(s_ctrl.cap_ms - 2U) : 0U;
        if(s_ctrl.hold_blend)s_ctrl.hold_blend--;
    }''')
    ctrl = replace(ctrl, '    if (s_ctrl.state != PC_SETTLE) return;', '''    if (s_ctrl.state == PC_HOLD) {
        /* A freshly submitted zero-distance target can already be held. New MOVE still
         * clears this qualification before the servo checks arrival. */
        if(Fx_Abs32(e)<=s_gain.w_in && Fx_Abs32(fb->vel)<=s_gain.vel_noise)s_ctrl.qualified=1U;
        return;
    }
    if (s_ctrl.state != PC_SETTLE) return;''')
    # MOVE and SETTLE use their existing position deadbands, with one added kv correction.
    ctrl = replace(ctrl, '        u_fb = Fx_MulShift(s_gain.kv_q16, i_in, 16);\n        di =', '''        u_fb = Fx_MulShift(s_gain.kv_q16, i_in, 16);
#if OPT_CORR_PCT > 0
        u_fb += (int32_t)((int64_t)s_gain.kv_uv * Fx_MulShift(s_gain.kp_q8, Fx_Deadband(e, s_gain.noise), 8) * OPT_CORR_PCT / 100000);
#endif
        di =''', count=2)
    # SETTLE uses min_step instead of the MOVE noise deadband.
    settle = ctrl.index('    case PC_SETTLE:')
    ctrl = ctrl[:settle] + ctrl[settle:].replace('Fx_Deadband(e, s_gain.noise)', 'Fx_Deadband(e, s_gain.min_step)')
    start = ctrl.index('    case PC_SETTLE:')
    end = ctrl.index('    default:   /* PC_HOLD */', start)
    section=ctrl[start:end]
    section=replace(section,'        break;', '''#if OPT_BLEND_MS > 0
        /* Blend the two existing laws and their integrator rates, using the same I state.
         * Qualification may revoke the ramp gradually; no stored output offset is injected. */
        if(s_ctrl.hold_blend){
            int32_t ex=Fx_Deadband(e,s_gain.w_in);
            int32_t hold_fb=Fx_MulShift(s_gain.kh_q16,ex,16);
            int32_t integral=Fx_MulShift(s_ctrl.integral_q16,1,16);
            int64_t hold_di=((int64_t)s_gain.kih_q20*ex)>>4;
            /* Interpolate the full HOLD endpoint, including its short-brake rule. */
            if(Fx_Abs32(hold_fb+integral)<s_gain.fc_mv/2)hold_fb=-integral;
            u_fb += (int32_t)((int64_t)(hold_fb-u_fb)*s_ctrl.hold_blend/OPT_BLEND_MS);
            di += (hold_di-di)*s_ctrl.hold_blend/OPT_BLEND_MS;
        }
#endif
        break;''')
    ctrl=ctrl[:start]+section+ctrl[end:]
    ctrl = replace(ctrl, '    output = Fx_Clamp32(desired, limit_mv);', '''    /* Hard protection interval wins even when the optional soft interval is infeasible. */
    output = Fx_Clamp32(desired, limit_mv);
#if OPT_EMF_PCT > 0
    {
        int32_t emf = (int32_t)((int64_t)s_gain.kv_uv * fb->vel / 1000);
        int32_t width = (int32_t)s_gain.cal_mv * OPT_EMF_PCT / 100;
        int32_t lo = emf - width, hi = emf + width;
        if (lo > limit_mv) output = limit_mv;
        else if (hi < -limit_mv) output = -limit_mv;
        else {
            if (lo < -limit_mv) lo = -limit_mv;
            if (hi > limit_mv) hi = limit_mv;
            output = Fx_ClampRange(desired, lo, hi);
        }
    }
#endif''')
    (candidate / 'C_Pos_Ctrl.c').write_text(ctrl, encoding='utf-8')
    hdr = (candidate / 'C_Pos_Ctrl.h').read_text(encoding='utf-8')
    hdr = replace(hdr, '\n#endif', '\n/* Qualified arrival is separate from the currently holding state used by Stop. */\nuint8_t C_PosCtrl_ArrivalQualified(void);\nvoid C_PosCtrl_BeginMotion(void);\n\n#endif')
    hdr=hdr.replace('误差在到位窗内且静止约20ms → HOLD 并上报到位(到位只看到位精度)', '误差在到位窗内且速度合格约20ms → 到位事件；位置锚点稳定后才过渡到HOLD')
    (candidate / 'C_Pos_Ctrl.h').write_text(hdr, encoding='utf-8')
    servo = (candidate / 'A_Servo.c').read_text(encoding='utf-8')
    servo = '#include "C_Control_Options.h"\n' + servo
    stream = '''
#if OPT_STREAM
/* One accepted-command clock, independent of trajectory merges. Explicit T is untouched. */
static struct { uint16_t age, interval; uint8_t consistent; int8_t direction; } s_stream;
static void Servo_StreamReset(void)
{
    s_stream.age=0xFFFFU;s_stream.interval=0U;s_stream.consistent=0U;s_stream.direction=0;
}
static void Servo_StreamObserve(uint16_t requested_ms,uint8_t was_paused,int32_t target)
{
    uint16_t interval=s_stream.age;
    int8_t direction=(int8_t)Fx_Sign32(target-s_servo.target_angle);
    if(requested_ms || was_paused){Servo_StreamReset();return;}
    if(interval>=10U && interval<=300U && s_stream.interval && direction==s_stream.direction
       && interval*2U>=s_stream.interval && interval<=2U*s_stream.interval){
        if(s_stream.consistent<2U)s_stream.consistent++;
    }else s_stream.consistent=0U;
    s_stream.interval=interval;s_stream.direction=direction;s_stream.age=0U;
}
#else
#define Servo_StreamReset() ((void)0)
#define Servo_StreamObserve(t,p,a) ((void)0)
#endif
'''
    servo = replace(servo, '/* 观测器从当前实测位置重新起步', stream + '\n/* 观测器从当前实测位置重新起步')
    servo = replace(servo, 'static void Servo_Resync(uint8_t reinit)\n{', 'static void Servo_Resync(uint8_t reinit)\n{\n    Servo_StreamReset();')
    servo = replace(servo, 'static void Servo_ClearCommand(void)\n{', 'static void Servo_ClearCommand(void)\n{\n    Servo_StreamReset();')
    servo = replace(servo, 'void A_Servo_Init(void)\n{', 'void A_Servo_Init(void)\n{\n    Servo_StreamReset();')
    servo = replace(servo, '    C_Traj_Plan(s_servo.target_angle, time_ms);', '''#if OPT_STREAM
    if(!s_servo.timed_position && s_stream.consistent>=2U){
        uint16_t horizon=(uint16_t)(2U*s_stream.interval);
        if(time_ms<horizon)time_ms=horizon;
    }
#endif
    C_Traj_Plan(s_servo.target_angle, time_ms);''')
    servo = replace(servo, '    s_servo.target_angle   = target;', '    Servo_StreamObserve(value,was_paused,target);\n    s_servo.target_angle   = target;')
    servo = replace(servo, '    s_servo.paused = s_servo.resume_valid = 1U;', '    Servo_StreamReset();\n    s_servo.paused = s_servo.resume_valid = 1U;')
    servo = replace(servo, 'void A_Servo_Control(void)\n{', '''void A_Servo_Control(void)
{
#if OPT_STREAM
    if(s_stream.age<0xFFFFU)s_stream.age++;
    if(s_stream.age>300U)s_stream.consistent=0U;
#endif''')
    servo = replace(servo, '    s_servo.arrived      = 0U;\n    s_servo.move_pending = 0U;', '    C_PosCtrl_BeginMotion();\n    s_servo.arrived      = 0U;\n    s_servo.move_pending = 0U;')
    servo = replace(servo, '    C_SpeedObs_Configure(C_MotorModel_Cal(), TUNE_OBS_BW_HZ, CTRL_VEL_NOISE_CDPS);', '''    {
        MotorCal_t observer_cal = *C_MotorModel_Cal();
        observer_cal.ka_nv = (int32_t)((int64_t)observer_cal.ka_nv * (10 + OPT_J_X10) / 10);
        C_SpeedObs_Configure(&observer_cal, TUNE_OBS_BW_HZ, CTRL_VEL_NOISE_CDPS);
    }''')
    servo = replace(servo, '    int64_t peak = (acc > dec) ? acc : dec;', '''    /* Fixed load profile: A/D/J are all scaled together, never from bus voltage. */
    acc = (int32_t)((int64_t)acc * OPT_ACC_PCT / 100);
    dec = (int32_t)((int64_t)dec * OPT_ACC_PCT / 100);
    int64_t peak = (acc > dec) ? acc : dec;''')
    servo = replace(servo, 'C_Traj_SetLimits(Servo_LimitOf((uint32_t)g_runtime_config.traj_vmax_dps * 100U, (uint32_t)cal->rated_vel_dps * 100U),', 'C_Traj_SetLimits((int32_t)((int64_t)Servo_LimitOf((uint32_t)g_runtime_config.traj_vmax_dps * 100U, (uint32_t)cal->rated_vel_dps * 100U) * OPT_VEL_PCT / 100),')
    servo = replace(servo, 's_servo.ref.done && C_PosCtrl_Reached()) s_servo.arrived = 1U;', 's_servo.ref.done && C_PosCtrl_ArrivalQualified()) s_servo.arrived = 1U;')
    (candidate / 'A_Servo.c').write_text(servo, encoding='utf-8')
    prepare_sim()


def prepare_sim():
    sim = (OLD / 'sim_conservative/sim.c').read_text(encoding='utf-8')
    # Record the limit at the controller boundary, before bus/protection updates.
    sim = replace(sim, '#include "C_Pos_Ctrl.c"', '''#define C_PosCtrl_Update Audit_Real_PosCtrl_Update
#include "C_Pos_Ctrl.c"
#undef C_PosCtrl_Update
static int32_t audit_used_limit;
static int32_t C_PosCtrl_Update(const TrajRef_t *ref, const PosCtrlFeedback_t *fb, int32_t limit, PosCtrlDbg_t *dbg)
{
    audit_used_limit=limit;
    return Audit_Real_PosCtrl_Update(ref,fb,limit,dbg);
}''')
    sim = replace(sim, 'static Plant P;', 'static double assumed_rw=3.2;\nstatic Plant P;')
    sim = replace(sim, 'p.rw = 3.2;', 'p.rw = assumed_rw;')
    sim = replace(sim, '    LoadCase loads[48]; int nl = 0;', '    for(int ai=6;ai<argc;ai++) if(!strncmp(argv[ai],"RW=",3)) assumed_rw=atof(argv[ai]+3);\n    LoadCase loads[48]; int nl = 0;')
    sim = replace(sim, 'r.arr_ms = -1; last_cmd_seen = g_motion_end;', 'r.arr_ms = -1; in_win=0; r.settle_ms=-1; last_cmd_seen = g_motion_end;')
    sim = replace(sim, 'r = run(nm, cmds[ci].s, cmds[ci].dur);', 'r = run(nm, cmds[ci].s, !strcmp(cmds[ci].name,"slowv") ? (int)ceil(20000/g_slowv)+2000 : cmds[ci].dur);')
    # Low-resistance release is high impedance: no electrically generated damping/current.
    sim = replace(sim, 'static Plant P;', 'static int high_z;\nstatic Plant P;')
    sim = replace(sim, '    int ph = abs(x)', '    high_z = 0;\n    int ph = abs(x)')
    sim = replace(sim, 'void D_Motor_Release(uint8_t h) { (void)h; duty_now = 0; }', 'void D_Motor_Release(uint8_t h) { high_z = !h; duty_now = 0; }')
    sim = replace(sim, 'duty_now = 0; sim_ms = 0;', 'high_z = 0; duty_now = 0; sim_ms = 0;')
    sim = replace(sim, 'net - kv * vm - f_c * s', 'net - (high_z ? 0 : kv * vm) - f_c * s')
    sim = replace(sim, 'netm - kv * vm - fc * s', 'netm - (high_z ? 0 : kv * vm) - fc * s')
    sim = replace(sim, 'isum += (u - kv * vm) / P.rw;', 'isum += high_z ? 0 : (u - kv * vm) / P.rw;')
    sim = replace(sim, '        ext = (P.grav', '        if(high_z)u=0;\n        ext = (P.grav')
    sim = replace(sim, '    int arr_ms;', '    int state_changes, hard_violations, hold_entries;\n    double state_maxdu, capture_maxdu, current_peak;\n    int arr_ms;')
    sim = replace(sim, '    int fault_prev = 0;', '    int prior_state = -1;\n    int fault_prev = 0;')
    sim = replace(sim, '            lastu = s_servo.dbg.u_out;', '''            if(s_servo.debug_control_valid){
                if(prior_state>=0 && prior_state!=s_servo.dbg.state){r.state_changes++;if(du>r.state_maxdu)r.state_maxdu=du;
                    if(prior_state==PC_SETTLE && s_servo.dbg.state==PC_HOLD){r.hold_entries++;if(du>r.capture_maxdu)r.capture_maxdu=du;}}
                prior_state=s_servo.dbg.state;
                if(abs(s_servo.dbg.u_out)>audit_used_limit)r.hard_violations++;
            } else prior_state=-1;
            if(fabs(i_avg)>r.current_peak)r.current_peak=fabs(i_avg);
            lastu = s_servo.dbg.u_out;''')
    sim = replace(sim, 'stall=%d,fault=%d,arr=%d\\n', 'stall=%d,fault=%d,arr=%d,state_changes=%d,state_maxdu=%.0f,capture_maxdu=%.0f,hard_violations=%d,current_peak=%.0f,hold_entries=%d\\n')
    sim = replace(sim, 'r->stall_trips, r->faults, r->arr_ms);', 'r->stall_trips, r->faults, r->arr_ms,r->state_changes,r->state_maxdu,r->capture_maxdu,r->hard_violations,r->current_peak,r->hold_entries);')
    # Extra read-only trace columns expose the control stages without changing dynamics.
    sim=replace(sim,'paused,duty\\n','paused,duty,refa,uff,ufb,ustick,applied_u,ref_done,arrived,limit_used\\n')
    trace_start=sim.index('            fprintf(trace, "%s,%d,%.1f')
    trace_end=sim.index('        if (s_prot.fault',trace_start)
    block=sim[trace_start:trace_end]
    block=replace(block,'\\n", trace_name',',%d,%d,%d,%d,%d,%d,%d,%d\\n", trace_name')
    block=replace(block,'s_servo.paused, duty_now);','s_servo.paused, duty_now, s_servo.ref.acc,s_servo.dbg.u_ff,s_servo.dbg.u_fb,s_servo.dbg.u_stick,s_servo.applied_mv,s_servo.ref.done,s_servo.arrived,audit_used_limit);')
    sim=sim[:trace_start]+block+sim[trace_end:]
    (HERE / 'source/sim.c').write_text(sim, encoding='utf-8')
    extended=replace(sim, '/* ---------- 负载函数 ---------- */', '''
static void sc_ext_long2(int t){if(t<=4000 && t%100==0){cmd(13500+1000.0*t/1000,2000);g_motion_end=t;}}
static void sc_ext_long60(int t){if(t<=4000 && t%100==0){cmd(13500+1000.0*t/1000,60000);g_motion_end=t;}}
static void sc_ext_stream200(int t){if(t<=4000 && t%200==0){cmd(13500+2000.0*t/1000,0);g_motion_end=t;}}
static void sc_ext_gap(int t){if(t<=4000 && (t<1000 || t>=2000) && t%100==0){cmd(13500+1000.0*t/1000,0);g_motion_end=t;}}
static void sc_ext_pause(int t){if(!t)cmd(22500,0);if(t==150)A_Servo_Pause();if(t==650){A_Servo_Resume();g_motion_end=t;}}
static void sc_ext_stop(int t){if(!t)cmd(22500,0);if(t==150){A_Servo_Stop();g_final_target=s_servo.target_angle;g_motion_end=t;}}
static void sc_ext_release(int t){if(!t)cmd(22500,0);if(t==1500)A_Servo_Release(0);if(t==2000){A_Servo_RestoreTorque();cmd(22500,0);g_motion_end=t;}}
static void sc_ext_jitter(int t){static int next;if(!t)next=0;if(t==next && t<=4000){cmd(13500+1000.0*t/1000,0);g_motion_end=t;next+=((t/10)%3==0?40:60);}}
/* ---------- 负载函数 ---------- */''')
    extended=replace(extended,'CmdCase cmds[] = {','''CmdCase cmds[] = {
        {"ext_long2",sc_ext_long2,16000,0},
        {"ext_long60",sc_ext_long60,76000,0},
        {"ext_stream200",sc_ext_stream200,16000,0,1000,3900,1},
        {"ext_gap",sc_ext_gap,16000,0},
        {"ext_pause",sc_ext_pause,14000,0},
        {"ext_stop",sc_ext_stop,14000,0},
        {"ext_release",sc_ext_release,16000,0},
        {"ext_jitter",sc_ext_jitter,16000,0,1000,3900,1},
        {"ext_hold",sc_hold,14000,0},
        {"ext_timed90",sc_timed90,14000,0},''')
    (HERE/'source/sim_extended.c').write_text(extended,encoding='utf-8')


def compile_host(profile, kg, extended=False):
    base = HERE / 'source/baseline' / f'{kg}kg'
    candidate = HERE / 'source/candidate'
    include = ['-I'+str(ROOT / 'tests/stubs')]
    if profile != 'baseline':
        include += ['-I'+str(candidate)]
    include += ['-I'+str(base / directory) for directory in ('Application','Common','Drivers','System','SRC/Debug')]
    exe = OUT / f'{profile}_{kg}{"_extended" if extended else ""}.exe'
    run([CC,'-std=gnu99','-O2','-Wall','-Wextra','-Wl,--image-base,0x400000',*include,
         *[f'-D{k}={v}' for k,v in PROFILES[profile].items() if k.startswith('OPT_')],HERE / ('source/sim_extended.c' if extended else 'source/sim.c'),'-lm','-o',exe],
         OUT / 'logs' / f'compile_{profile}_{kg}{"_extended" if extended else ""}.log')
    return exe


def matrix(profiles=None):
    (OUT/'profiles.json').write_text(json.dumps(PROFILES,indent=2),encoding='utf-8')
    def job(item):
        profile, kg = item
        exe = compile_host(profile, kg)
        knobs = [f'HOLDKI={PROFILES[profile]["RUN_HOLD_KI"]}'] if 'RUN_HOLD_KI' in PROFILES[profile] else []
        result = parse(run([exe,f'{kg}kg','matrix','NUL','*','*',*knobs], OUT / 'logs' / f'matrix_{profile}_{kg}.txt'))
        assert len(result) == 351
        return profile, str(kg), result
    data = json.loads((OUT/'matrix.json').read_text()) if profiles and (OUT/'matrix.json').exists() else {}
    with ThreadPoolExecutor(max_workers=3) as pool:
        for future in as_completed([pool.submit(job, (p,kg)) for p in (profiles or PROFILES) for kg in (15,20,30)]):
            profile, kg, result = future.result()
            data.setdefault(profile,{})[kg]=result
            print('MATRIX',profile,kg,len(result),flush=True)
    (OUT / 'matrix.json').write_text(json.dumps(data,indent=2),encoding='utf-8')
    summarize(data)


LOADED = ['inertia2','inertia4','inertia8','grav20%_r4','grav40%_r6','grav60%_r8',
          'stepload+3V','stepload-3V','randload','grav30%_r4_bl0.6','grav30%_r4_6.4V','push_r4','push_r8','push_r4_bl']
POINT = ['bigstep','reverse400','reverse_arrive','timed90','hold']


def summarize(data=None):
    import statistics as st
    if data is None:
        data=json.loads((OUT/'matrix.json').read_text())
    result={}
    for profile, models in data.items():
        result[profile]={}
        for kg,rows in sorted(models.items()):
            pts=[rows[f'{load}|{command}'] for load in LOADED for command in POINT]
            values=sorted(r['os'] for r in pts)
            d=dict(out=sum(abs(r['final'])>.5 for r in pts),ring=sum(r['ring'] for r in pts),
                   os_med=st.median(values),os_p90=values[63],os_max=max(values),
                   fault=sum(r['fault'] for r in rows.values()),stall=sum(r['stall'] for r in rows.values()),
                   hard_violations=sum(r['hard_violations'] for r in rows.values()),
                   state_maxdu=max(r['state_maxdu'] for r in pts), capture_maxdu=max(r['capture_maxdu'] for r in pts))
            result[profile][kg]=d
        print(profile,json.dumps(result[profile]),flush=True)
    (OUT/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    return result


def verify():
    manifest=json.loads((OUT/'production_hashes_start.json').read_text())
    changes=[]
    for directory, files in manifest.items():
        for name, sha in files.items():
            if hashlib.sha256((ROOT/directory/name).read_bytes()).hexdigest()!=sha:
                changes.append(directory+'/'+name)
    (OUT/'production_changes.json').write_text(json.dumps(changes,indent=2),encoding='utf-8')
    print('External production changes:',changes,flush=True)


def extra(profiles):
    def job(item):
        profile,kg=item
        exe=compile_host(profile,kg,extended=True)
        commands=['ext_long2','ext_long60','ext_stream200','ext_gap','ext_pause','ext_stop','ext_release','ext_jitter','ext_hold','ext_timed90']
        result={}
        for load in ('noload','inertia4','grav20%_r4','randload','grav30%_r4_bl0.6'):
            for command in commands:
                knobs=[f'HOLDKI={PROFILES[profile]["RUN_HOLD_KI"]}'] if 'RUN_HOLD_KI' in PROFILES[profile] else []
                result.update(parse(run([exe,f'{kg}kg','matrix','NUL',load,command,*knobs],OUT/'logs'/f'extra_{profile}_{kg}_{load}_{command}.txt')))
        for speed in (.5,1,2,4):
            for load in ('noload','grav20%_r4'):
                rows=parse(run([exe,f'{kg}kg','matrix','NUL',load,'slowv',f'SLOWV={speed}',*knobs],OUT/'logs'/f'extra_{profile}_{kg}_{load}_slow{speed}.txt'))
                result.update({key+f'_speed{speed}':row for key,row in rows.items()})
        print('EXTRA',profile,kg,len(result),flush=True)
        return profile,str(kg),result
    data=json.loads((OUT/'extra.json').read_text()) if (OUT/'extra.json').exists() else {}
    with ThreadPoolExecutor(max_workers=3) as pool:
        for profile,kg,result in pool.map(job,[(p,k) for p in profiles for k in (15,20,30)]):
            data.setdefault(profile,{})[kg]=result
    (OUT/'extra.json').write_text(json.dumps(data,indent=2),encoding='utf-8')
    return data


def traces(profiles):
    pairs=[('inertia4','bigstep'),('grav20%_r4','reverse400'),('push_r4','ext_hold'),
           ('noload','ext_stream200'),('grav20%_r4','ext_long2'),('noload','slowv')]
    for kg in (15,20,30):
        for profile in profiles:
            exe=OUT/f'{profile}_{kg}_extended.exe'
            for load,command in pairs:
                dest=OUT/'traces'/f'{profile}_{kg}_{load}_{command}.csv'
                dest.parent.mkdir(exist_ok=True)
                knobs=[f'HOLDKI={PROFILES[profile]["RUN_HOLD_KI"]}'] if 'RUN_HOLD_KI' in PROFILES[profile] else []
                run([exe,f'{kg}kg','matrix',dest,load,command,*knobs],OUT/'logs'/f'trace_{dest.stem}.txt')
    print('Traces complete',flush=True)


if __name__=='__main__':
    if '--run-only' not in sys.argv:
        prepare()
    matrix()
    verify()
