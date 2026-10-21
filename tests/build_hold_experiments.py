"""Create isolated diagnostic controller variants. Never edit production code."""
from pathlib import Path
import sys
import subprocess
import hashlib

root = Path(__file__).resolve().parent.parent
out = root / 'backups/lever_audit_20261013'
variants = out / 'variants'
variants.mkdir(parents=True, exist_ok=True)
source_path = root / 'CH32V00X_APP_ADC_30kg/Common/C_Pos_Ctrl.c'
source = source_path.read_text(encoding='utf-8')
original_hash = hashlib.sha256(source_path.read_bytes()).hexdigest()

clear = '''            s_ctrl.integral_q16 = 0;
            s_ctrl.loaded = s_ctrl.probing = 0U;
            s_ctrl.push_ms = 0U;'''
assert source.count(clear) == 1
no_clear = source.replace(clear, '''            /* Audit experiment: keep the learned support voltage. */
            s_ctrl.push_ms = 0U;''')
probe = '''static void PosCtrl_Probe(int32_t e)
{'''
assert source.count(probe) == 1
no_probe = source.replace(probe, probe + '\n    return; /* Audit experiment: disable integral probing only. */')
both = no_clear.replace(probe, probe + '\n    return; /* Audit experiment: retain support bias. */')

hold = '''    case PC_HOLD:
        u_fb = -Fx_MulShift(s_gain.dh_q16, v_d, 16);
        mode = I_PROBE;
        break;'''
assert source.count(hold) == 1
continuous = both.replace(hold, '''    case PC_HOLD:
        /* Experimental continuous loaded hold: same bias, weak P + PI,
         * measured velocity damping. No force deletion/probing/kick here. */
        i_in = Fx_MulShift(s_gain.kph_q8, Fx_Deadband(e, s_gain.min_step), 8) - fb->vel;
        u_fb = Fx_MulShift(s_gain.kh_q16, Fx_Deadband(e, s_gain.min_step), 16)
             - Fx_MulShift(s_gain.dh_q16, fb->vel, 16);
        mode = I_NORMAL;
        break;''')
continuous = continuous.replace('''            int64_t di = ((int64_t)s_gain.ki_q20 * i_in) >> 4;''',
'''            int64_t di = ((int64_t)s_gain.ki_q20 * i_in) >> 4;
            if (s_ctrl.state == PC_HOLD) di /= 4;''')
continuous = continuous.replace('''    int32_t v_d = Fx_Deadband(fb->vel, s_gain.vel_noise);''',
'''    int32_t v_d __attribute__((unused)) = Fx_Deadband(fb->vel, s_gain.vel_noise);''')

# Loaded recover with preserved bias and gradual correction; avoid stiction
# pulses while a load is already supported. Still an experiment, not a fix.
load_hold = both.replace('''        if (Fx_Abs32(e) > s_gain.w_in) demand = Fx_Sign32(e);''',
'''        demand = 0; /* Audit: PI adapts gradually, no RECOVER kick. */''')
load_hold = load_hold.replace('''int32_t v_d = Fx_Deadband(fb->vel, s_gain.vel_noise);''',
'''int32_t v_d = fb->vel;''')
load_hold = load_hold.replace('''int32_t v_a = Fx_Deadband(fb->vel, s_gain.vel_noise / 2);''',
'''int32_t v_a = fb->vel;''')
load_hold = load_hold.replace('''    if (s_ctrl.state == PC_HOLD)
    {   /* 不值得驱动''', '''    if (s_ctrl.state != PC_MOVE)
    {   /* 不值得驱动''')
load_hold = load_hold.replace('''            int64_t di = ((int64_t)s_gain.ki_q20 * i_in) >> 4;''',
'''            int64_t di = ((int64_t)s_gain.ki_q20 * i_in) >> 4;
            if (s_ctrl.state != PC_MOVE) {
                int64_t di_max = ((int64_t)s_gain.fs_mv << 16) / 100;
                if (di > di_max) di = di_max;
                if (di < -di_max) di = -di_max;
            }''')

unified = both
rest_start = unified.index('    case PC_SETTLE:', unified.index('int32_t C_PosCtrl_Update'))
rest_end = unified.index('    u_i = Fx_MulShift', rest_start)
unified = unified[:rest_start] + '''    case PC_SETTLE:
    case PC_HOLD:
    default:
        /* Audit: one continuous static law, preserve load bias.
         * Correct only outside the capture window, always damp velocity. */
        i_in = Fx_MulShift(s_gain.kph_q8, Fx_Deadband(e, s_gain.w_in), 8) - fb->vel;
        u_fb = Fx_MulShift(s_gain.kh_q16, Fx_Deadband(e, s_gain.w_in), 16)
             - Fx_MulShift(s_gain.dh_q16, fb->vel, 16);
        mode = I_NORMAL;
        break;
    }

''' + unified[rest_end:]
unified = unified.replace('''    int32_t v_d = Fx_Deadband(fb->vel, s_gain.vel_noise);''',
'''    int32_t v_d __attribute__((unused)) = Fx_Deadband(fb->vel, s_gain.vel_noise);''')
unified = unified.replace('''    int32_t v_a = Fx_Deadband(fb->vel, s_gain.vel_noise / 2);''',
'''    int32_t v_a __attribute__((unused)) = Fx_Deadband(fb->vel, s_gain.vel_noise / 2);''')
unified = unified.replace('''    if (s_ctrl.state == PC_HOLD)
    {   /* 不值得驱动''', '''    if (s_ctrl.state != PC_MOVE)
    {   /* 不值得驱动''')
unified = unified.replace('''            int64_t di = ((int64_t)s_gain.ki_q20 * i_in) >> 4;''',
'''            int64_t di = ((int64_t)s_gain.ki_q20 * i_in) >> 4;
            if (s_ctrl.state != PC_MOVE) {
                if (Fx_Abs32(e) <= s_gain.w_in && Fx_Abs32(fb->vel) < s_gain.vel_noise / 8) di = 0;
            }''')

gcc = sys.argv[1] if len(sys.argv)>1 else r'D:\SoftWare\mingw64\bin\gcc.exe'
includes = ['-I'+str(root/'tests/stubs')]
includes += ['-I'+str(root/'CH32V00X_APP_ADC_30kg'/p) for p in ['Application','Common','Drivers','System','SRC/Debug']]
for name, code in [('no_clear',no_clear),('no_probe',no_probe),('retain_bias',both),('continuous_hold',continuous),('load_hold',load_hold),('unified_rest',unified)]:
    path = variants / (name+'.c')
    path.write_text(code,encoding='utf-8')
    entry = variants / (name+'_entry.c')
    entry.write_text('#define AUDIT_POS_CTRL_SOURCE "'+path.as_posix()+'"\n#include "'+(root/'tests/lever_hold_sim.c').as_posix()+'"\n',encoding='utf-8')
    exe = out / ('lever_'+name+'.exe')
    subprocess.run([gcc,'-std=gnu99','-O2','-Wall','-Wextra','-Werror',*includes,str(entry),'-lm','-o',str(exe)],check=True)
    result = subprocess.run([str(exe),str(out/('summary_'+name+'.csv')),str(out/('trace_'+name+'.csv'))],check=True,capture_output=True,text=True)
    (out/('disturb_'+name+'.txt')).write_text(result.stdout,encoding='utf-8')
    subprocess.run([str(exe),str(out/('sweep_'+name+'.csv'))],check=True)
    print(name, 'built and simulated')
assert hashlib.sha256(source_path.read_bytes()).hexdigest() == original_hash
