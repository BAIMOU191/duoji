from pathlib import Path
import importlib.util,sys,hashlib,json,shutil,subprocess,re
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=HERE/'output'
PROJECT=ROOT/'CH32V00X_APP_ADC_20kg'
spec=importlib.util.spec_from_file_location('support',ROOT/'tests/sync_noncontrol_adc_20261019/verify.py')
s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s);s.OUT=OUT
spec=importlib.util.spec_from_file_location('pwm',ROOT/'tests/pwm_jitter_20kg_20261019/run.py')
pwm=importlib.util.module_from_spec(spec);spec.loader.exec_module(pwm)
sys.path.insert(0,str(ROOT/'tests/control_optimization_20261019'))
import run_optimization as sim

STICTION='''/* 起转补偿以积分托载量为基准；只对补偿分量限变化率，主闭环和制动输出不滤波。
 * 最多5ms升/降一个fs，跨过速度/位移判据时逐步交回闭环，反向时也经过零。 */
static int32_t PosCtrl_Stiction(const PosCtrlFeedback_t *fb, int32_t demand, int32_t u)
{
    int32_t target = 0, need;
    int32_t step = (s_gain.fs_mv + STICK_SLEW_MS - 1) / STICK_SLEW_MS;

    if (Fx_Abs32(fb->pos - s_ctrl.anchor) >= s_gain.min_step) { s_ctrl.anchor = fb->pos; s_ctrl.still_ms = 0U; }
    else if (s_ctrl.still_ms < 0xFFFFU) s_ctrl.still_ms++;

    if (demand != 0 && s_ctrl.still_ms >= STICK_MOVE_MS && Fx_Abs32(fb->vel) < s_gain.vel_noise)
    {
        if (s_ctrl.still_ms >= STICK_ESC_MS) s_ctrl.boost = Fx_ClampRange(s_ctrl.boost + s_gain.fs_mv / STICK_ESC_DIV, 0, s_gain.fs_mv);
        need = Fx_MulShift(s_ctrl.integral_q16, 1, 16) + demand * s_gain.fs_mv;
        target = (((demand > 0) ? (u < need) : (u > need)) ? need - u : 0) + demand * s_ctrl.boost;
    }
    else
    {
        s_ctrl.boost -= s_gain.fs_mv / STICK_ESC_DIV;
        if (s_ctrl.boost < 0) s_ctrl.boost = 0;
    }
    s_ctrl.stick += Fx_Clamp32(target - s_ctrl.stick, step);
    return s_ctrl.stick;
}

'''

def snapshot():
    if (HERE/'source_hashes_before.json').exists():return
    manifest={f.relative_to(ROOT).as_posix():hashlib.sha256(f.read_bytes()).hexdigest() for p in ROOT.glob('CH32V00X_APP*') if p.is_dir() for folder in ('Application','Common','Drivers','System','User','SRC') for f in (p/folder).rglob('*') if f.is_file() and f.suffix in ('.c','.h','.S','.ld')}
    (HERE/'source_hashes_before.json').write_text(json.dumps(manifest,indent=2))
    for folder in ('Application','Common','Drivers','System','User','SRC','obj'):
        if (PROJECT/folder).exists():shutil.copytree(PROJECT/folder,HERE/'before'/folder)

def candidate(name,bw=None,stick=False):
    p=HERE/'candidates'/name
    if not (p/'Application/A_Config.c').exists():
        for folder in ('Application','Common','Drivers','System','SRC'):shutil.copytree(HERE/'before'/folder,p/folder,dirs_exist_ok=True)
    for folder in ('Application','Common'):(p/folder).mkdir(parents=True,exist_ok=True)
    cfg=(HERE/'before/Application/A_Config.h').read_text(encoding='utf-8')
    if bw:
        cfg=re.sub(r'(#define TUNE_OBS_BW_HZ\s+)21\b',lambda m:m[1]+str(bw),cfg)
        cfg=cfg.replace('#define TUNE_VEL_BW_RADS', '#define CTRL_VEL_BW_OBS_MAX ((TUNE_OBS_BW_HZ * 6283U / 1000U < 120U) ? TUNE_OBS_BW_HZ * 6283U / 1000U : 120U)\n#define TUNE_VEL_BW_RADS',1)
        cfg=cfg.replace('((CTRL_VEL_BW_DELAY_MAX < 120U) ? CTRL_VEL_BW_DELAY_MAX : 120U)', '((CTRL_VEL_BW_DELAY_MAX < CTRL_VEL_BW_OBS_MAX) ? CTRL_VEL_BW_DELAY_MAX : CTRL_VEL_BW_OBS_MAX)',1)
    (p/'Application/A_Config.h').write_text(cfg,encoding='utf-8')
    c=(HERE/'before/Common/C_Pos_Ctrl.c').read_text(encoding='utf-8')
    if stick:
        c=c.replace('#define STICK_MOVE_MS', '#define STICK_SLEW_MS   5U     /* 补偿分量每拍最多改变fs/5，主输出不受限 */\n#define STICK_MOVE_MS',1)
        c=c.replace('    int32_t  boost;', '    int32_t  stick;          /* 起转补偿的连续输出，mV */\n    int32_t  boost;',1)
        a=c.index('/* 起转补偿(只在运动中)');b=c.index('/* 参考在动用MOVE',a)
        c=c[:a]+STICTION+c[b:]
    if name.startswith('db'):
        divisor=int(name[2:])
        c=c.replace('u_fb = Fx_MulShift(s_gain.kv_q16, i_in, 16)',f'u_fb = Fx_MulShift(s_gain.kv_q16, v_pos + Fx_Deadband(ref->vel - fb->vel, s_gain.vel_noise / {divisor}), 16)',1)
    if name=='fir':
        c=c.replace('    int32_t  integral_q16;', '    int32_t  vel_error_prev; /* 上一拍速度残差 */\n    int32_t  integral_q16;',1)
        c=c.replace('int64_t di;   /* 本拍积分增量，mV Q16 */','int32_t vel_error = ref->vel - fb->vel;\n    int32_t vel_filtered = (vel_error + s_ctrl.vel_error_prev) / 2;\n    int64_t di;   /* 本拍积分增量，mV Q16 */\n    s_ctrl.vel_error_prev = vel_error;',1)
        c=c.replace('u_fb = Fx_MulShift(s_gain.kv_q16, i_in, 16)', 'u_fb = Fx_MulShift(s_gain.kv_q16, v_pos + vel_filtered, 16)',1)
        c=c.replace('cal->delay_us > 333U','(cal->delay_us + 500U) > 333U',1).replace('3UL * cal->delay_us','3UL * (cal->delay_us + 500U)',1)
    elif name.startswith('f') or name.startswith('m'):
        tau=float(name.split('_')[0][1:]);den=3 if tau==.5 else int(tau+1);num=2 if tau==.5 else 1
        c=c.replace('    int32_t  integral_q16;', '    int32_t  vel_error_q8;  /* 速度跟踪残差的一阶降噪状态，Q8 */\n    int32_t  integral_q16;',1)
        c=c.replace('    PosCtrl_State(ref, fb, e);',f'    PosCtrl_State(ref, fb, e);\n    s_ctrl.vel_error_q8 += ((ref->vel - fb->vel) * 256 - s_ctrl.vel_error_q8) * {num} / {den};',1)
        c=c.replace('u_fb = Fx_MulShift(s_gain.kv_q16, i_in, 16)', 'u_fb = Fx_MulShift(s_gain.kv_q16, v_pos + s_ctrl.vel_error_q8 / 256, 16)',1)
        # Keep qualification and integration on the original observer; filter only proportional damping.
        if name.startswith('f'):c=c.replace('u_fb = Fx_MulShift(s_gain.kv_q16, i_in, 16)', 'u_fb = Fx_MulShift(s_gain.kv_q16, v_pos + Fx_Deadband(s_ctrl.vel_error_q8 / 256, s_gain.vel_noise / 2), 16)',1)
        c=c.replace('cal->delay_us > 333U',f'(cal->delay_us + {int(tau*1000)}U) > 333U',1).replace('3UL * cal->delay_us',f'3UL * (cal->delay_us + {int(tau*1000)}U)',1)
    if '_s' in name:c=c.replace('#define STICK_SLEW_MS   5U',f'#define STICK_SLEW_MS   {int(name.split("_s")[1])}U')
    if '_b' in name:
        c=c.replace('    int32_t e = ref->pos - fb->pos;', '    uint16_t blend_prev = s_ctrl.hold_blend_ms;\n    int32_t e = ref->pos - fb->pos;',1)
        c=c.replace('if (s_ctrl.hold_blend_ms != 0U)\n        {','if (s_ctrl.hold_blend_ms != 0U || blend_prev != 0U)\n        {',1)
        c=c.replace('            u_fb += (int32_t)(((int64_t)hold_fb - u_fb) * s_ctrl.hold_blend_ms / HOLD_BLEND_MS);', '''            /* 交接权重变化时把消失的反馈力转入共用积分；保留原输出，避免冻结下来的动态积分再次起转。 */
            if (fb->valid && blend_prev != s_ctrl.hold_blend_ms)
                PosCtrl_IntegralAdd((int64_t)(u_fb - hold_fb) * ((int32_t)s_ctrl.hold_blend_ms - blend_prev) * 65536 / HOLD_BLEND_MS, limit_mv);
            u_fb += (int32_t)(((int64_t)hold_fb - u_fb) * s_ctrl.hold_blend_ms / HOLD_BLEND_MS);''',1)
    if '_r' in name:
        c=c.replace('''        /* 新提交的零位移目标可能已经在保持，不能因清到位资格而永远不完成。 */''','''        /* 保持后漂出精度窗一个可靠位移，重新收尾；从100%保持权重退出，避免换律跳变。 */
        if (Fx_Abs32(e) > s_gain.w_in + s_gain.min_step)
        {
            s_ctrl.state = PC_SETTLE;
            s_ctrl.cap_ms = 0U;
            s_ctrl.hold_blend_ms = HOLD_BLEND_MS;
            return;
        }
        /* 新提交的零位移目标可能已经在保持，不能因清到位资格而永远不完成。 */''',1)
        if '_r0' in name:c=c.replace('Fx_Abs32(e) > s_gain.w_in + s_gain.min_step','Fx_Abs32(e) > s_gain.w_in',1)
    (p/'Common/C_Pos_Ctrl.c').write_text(c,encoding='utf-8')
    return p

def build(name,p,source):
    fallback=['-I'+str(HERE/'before'/d) for d in ('Application','Common','Drivers','System','SRC/Debug')]
    return s.compile_host(name,source,p,20,fallback)

def main():
    HERE.mkdir(exist_ok=True);snapshot();(OUT/'20').mkdir(parents=True,exist_ok=True)
    allresults=json.loads((OUT/'pwm.json').read_text()) if (OUT/'pwm.json').exists() else {}
    matrices=json.loads((OUT/'matrix.json').read_text()) if (OUT/'matrix.json').exists() else {}
    plans=[('base',None,False),('stick',None,True),('bw18',18,True),('bw16',16,True),('bw14',14,True),('db2',None,True),('db4',None,True),('f1',None,True),('f2',None,True),('m1',None,True),('m0.5',None,True),('fir',None,True),('m1_s10',None,True),('stick_s10',None,True),('m1_s3',None,True),('m1_b',None,True),('m1_r',None,True),('m1_r0',None,True),('f1_r0',None,True)]
    cases=[(mode,T,noise) for mode,T in ((0,0),(0,1000),(3,0),(3,1000),(4,0),(4,1000),(5,0),(6,0),(8,0),(9,0)) for noise in (0,1,2,5)]
    for name,bw,stick in plans:
        if name in matrices:continue
        p=candidate(name,bw,stick);exe=build('pwm_'+name,p,ROOT/'tests/pwm_jitter_20kg_20261019/check.c')
        results={}
        for mode,T,noise in cases:
            tag=f'{name}_m{mode}_T{T}_n{noise}'
            trace=OUT/(tag+'.csv');pr=subprocess.run([str(exe),str(mode),str(T),str(noise),'0',str(trace)],capture_output=True,text=True)
            assert pr.returncode==0,pr.stderr
            results[tag]=pwm.analyze(trace)
        allresults[name]=results
        exe=build('matrix_'+name,p,ROOT/'tests/control_optimization_20261019/source/sim.c')
        matrices[name]=sim.parse(s.run([exe,'20kg','matrix','NUL','*','*'],OUT/(name+'_matrix.log')))
        (OUT/'pwm.json').write_text(json.dumps(allresults,indent=2));(OUT/'matrix.json').write_text(json.dumps(matrices,indent=2))
        m=matrices[name]
        print(name,'matrix',len(m),'fault',sum(r['fault'] for r in m.values()),'final_out',sum(abs(r['final'])>.5 for r in m.values()),'max_os',max(r['os'] for r in m.values()),flush=True)
        for mode in (0,5,8,9):
            r=results[f'{name}_m{mode}_T0_n1'];print('  mode',mode,'move',r['move'].get('max_step_pct'),'cruise',r['cruise250_450'].get('pwm_pp_pct'),'final window pp',r['last500'].get('pwm_pp_pct'),flush=True)
    manifest=json.loads((HERE/'source_hashes_before.json').read_text());assert all(hashlib.sha256((ROOT/f).read_bytes()).hexdigest()==v for f,v in manifest.items())
    print('PASS production unchanged',flush=True)
if __name__=='__main__':main()
