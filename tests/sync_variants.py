"""Align app variants to the reviewed 30kg framework; retain hardware/calibration data."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'CH32V00X_APP_ADC_30kg'
VARIANTS = ['CH32V00X_APP_ADC_15kg', 'CH32V00X_APP_ADC_20kg',
            'CH32V00X_APP_ADC_20kg_single', 'CH32V00X_APP-IIC_20kg']

def read(p):
    return p.read_text(encoding='utf-8')

def write(p, text):
    p.write_text(text, encoding='utf-8', newline='\n')

def replace(text, old, new):
    assert old in text, old[:100]
    return text.replace(old, new, 1)

def function(text, name, body):
    start = re.search(r'^(?:static )?(?:void|uint8_t|uint16_t|int32_t) '+name+r'\([^;]*?\)\s*\{', text, re.M).start()
    brace = text.index('{', start)
    depth, end = 1, brace + 1
    while depth:
        if text[end] == '{': depth += 1
        elif text[end] == '}': depth -= 1
        end += 1
    return text[:start] + body.strip() + text[end:]

def config(old, circular):
    t = read(BASE / 'Application/A_Config.h')
    for name in ['FW_SERVO_TYPE', 'FW_HW_VERSION', 'FW_VOLTAGE', 'FW_TORQUE', 'CFG_DIR_INVERT']:
        line = re.search(r'^#define '+name+r'\s+.*$', old, re.M).group()
        t = re.sub(r'^#define '+name+r'\s+.*$', lambda _: line, t, flags=re.M)
    start = t.index('/* ==================== 2.')
    end = t.index('/* ==================== 3.')
    if 'MOTOR_KV_UV' in old:
        # Subsequent syncs must not erase a new calibration performed on this hardware.
        block_end = old.index('/* ==================== 电机标定结束 ==================== */')
        block_end += len('/* ==================== 电机标定结束 ==================== */')
        block = old[old.index('/* ==================== 2.'):block_end] + '\n\n'
    else:
        block = t[start:end]
        block = re.sub(r'^(#define MOTOR_\w+\s+)\S+', r'\g<1>0', block, flags=re.M)
        sign = re.search(r'^#define CAL_MOTOR_SIGN\s+(-?\d+)', old, re.M).group(1)
        block = re.sub(r'^(#define MOTOR_SIGN\s+)0', r'\g<1>'+sign, block, flags=re.M)
        for dest, source, scale in [('MOTOR_DELAY_US', 'MODEL_DELAY_TICKS', 1000),
                                    ('MOTOR_MIN_STEP_CDEG', 'CAL_MIN_STEP_CDEG', 1)]:
            value = int(re.search(r'^#define '+source+r'\s+(\d+)', old, re.M).group(1))*scale
            block = re.sub(r'^(#define '+dest+r'\s+)0\b', lambda m: m[1]+str(value), block, flags=re.M)
    t = t[:start]+block+t[end:]
    if circular:
        t = replace(t, '    SERVO_MODE_TURNS_CW = 7,', '    SERVO_MODE_360_CW = 5, /* 磁编码器整圈位置模式 */\n    SERVO_MODE_360_CCW,\n    SERVO_MODE_TURNS_CW = 7,')
        t = replace(t, '(((m) >= SERVO_MODE_270_CW && (m) <= SERVO_MODE_180_CCW) || ((m) >= SERVO_MODE_TURNS_CW && (m) <= SERVO_MODE_CUSTOM))',
                    '((m) >= SERVO_MODE_270_CW && (m) <= SERVO_MODE_CUSTOM)')
        t = t.replace('5/6 是整圈编码器的360度模式，\n * 电位器有死区不支持，编号空着，Flash 里遗留的5/6上电修回270度。',
                      '5/6 是磁编码器的360度模式，本型号支持整圈位置与多圈轨迹。')
    return t

def servo(circular):
    t = read(BASE / 'Application/A_Servo.c')
    if not circular: return t
    t = t.replace('位置模式(1~4, 11)', '位置模式(1~6, 11)').replace(
        '反馈是电位器的线性行程：不回绕，两端之外有测不到角度的死区，掉进去时开环脱困。',
        'MT6701反馈整圈回绕；应用层展开为连续坐标后交给同一观测器/规划器/控制器。读失败停机，无开环脱困。')
    t = re.sub(r'^#define SERVO_ESCAPE_.*\n', '', t, flags=re.M)
    t = t.replace('可转入区只有 [TRAVEL_LO, TRAVEL_HI]，放不下标称行程就围绕锚点收缩；每次从标称行程重算，不叠加',
                  '整圈可测，锚点只决定零点；每次从标称行程重算，不叠加')
    t = t.replace('编码器连续失效：先停，再交给脱困', '编码器连续失效：停机卸力，禁止开环脱困')
    t = t.replace('穿过死区读不到时保留上次采样，下次读数会包含这段位移', '磁编码器跨零点按最短采样差累计；连续读失败停机')
    t = replace(t, '    uint8_t  escape_dir, escape_retried, vbus_n;', '''    uint8_t  vbus_n;
    uint8_t  circular_valid, mt_active, mt_final;
    uint16_t mt_turn_ms;
    int32_t  mt_target; /* 连续坐标，多圈参考与反馈共用，算法层不处理回绕 */''')
    t = replace(t, '    uint16_t escape_ticks, escape_settle;\n', '')
    t = replace(t, 'enum { SERVO_ENC_OK = 0, SERVO_ENC_ESCAPING, SERVO_ENC_FAULT };',
                'enum { SERVO_ENC_OK = 0, SERVO_ENC_FAULT };')
    t = t.replace('等脱困结束', '等首个有效编码器读数').replace('在等脱困结束', '在等首个有效编码器读数')
    t = t.replace('；脱困中时置暂停，等新指令', '，恢复普通行程坐标')
    t = replace(t, '    s_servo.paused       = (uint8_t)(s_servo.enc_state == SERVO_ENC_ESCAPING);',
                '    s_servo.paused       = 0U;')
    t = t.replace('脱困与上电', '上电与输入检测')
    t = replace(t, 'static uint8_t Servo_SubmitQ3(uint16_t q3, uint16_t value);', '''static uint8_t Servo_SubmitQ3(uint16_t q3, uint16_t value);
static void Servo_MultiTurnAbort(void);
static void Servo_PlanMultiTurn(void);

static int32_t Servo_Unwrap(int32_t wrapped, int32_t near)
{
    /* 每拍位移须小于半圈；9圈指令的连续坐标远小于int32/Q8上限。 */
    while (wrapped - near > CDEG_RANGE / 2) wrapped -= CDEG_RANGE;
    while (near - wrapped > CDEG_RANGE / 2) wrapped += CDEG_RANGE;
    return wrapped;
}''')
    t = t.replace('s_servo.mode <= SERVO_MODE_180_CCW', 's_servo.mode <= SERVO_MODE_360_CCW')
    t = replace(t, 'return (mode <= SERVO_MODE_180_CCW) ? ((mode <= SERVO_MODE_270_CCW) ? 27000U : 18000U) : 0U;',
                'return (mode <= SERVO_MODE_360_CCW) ? ((mode <= SERVO_MODE_270_CCW) ? 27000U : ((mode <= SERVO_MODE_180_CCW) ? 18000U : 36000U)) : 0U;')
    t = function(t, 'Servo_DeriveCal', '''static void Servo_DeriveCal(uint8_t kind, int32_t anchor, uint16_t nominal,
                            uint16_t *span_out, int32_t *offset_out)
{
    *span_out = nominal; /* 整圈可测，无须按电位器两端收缩行程 */
    *offset_out = Servo_WrapAngle(anchor - ((kind == SERVO_CAL_MID) ? nominal / 2 : (s_servo.reverse ? nominal : 0)));
}''')
    t = replace(t, '    if (offset > ENCODER_POT_ANGLE_MAX) offset -= CDEG_RANGE;   /* Flash 按[0,36000)存，负零点存成36000+偏移 */',
                '    s_servo.circular_valid = 0U;')
    t = replace(t, '    s_servo.angle_circ = raw - s_servo.offset_cdeg;   /* 闭环用，不截断：截断会在顶住端点时把误差抹成0 */', '''    {
        int32_t wrapped = Servo_WrapAngle(raw - s_servo.offset_cdeg);
        int32_t near = (s_servo.mt_active || s_servo.span_cdeg == CDEG_RANGE) && s_servo.circular_valid
                     ? s_servo.angle_circ : s_servo.span_cdeg / 2;
        s_servo.angle_circ = Servo_Unwrap(wrapped, near);
        s_servo.circular_valid = 1U;
    }''')
    t = replace(t, '    s_servo.angle      = (uint16_t)Fx_ClampRange(s_servo.angle_circ, 0, s_servo.span_cdeg);', '''    s_servo.angle = (uint16_t)Fx_ClampRange(s_servo.mt_active
                     ? Servo_WrapAngle(s_servo.angle_circ) : s_servo.angle_circ, 0, s_servo.span_cdeg);''')
    t = function(t, 'Servo_SetTrajRange', '''static void Servo_SetTrajRange(void)
{
    s_servo.range_valid = (uint8_t)(s_servo.span_cdeg >= SERVO_CUSTOM_SPAN_MIN);
    s_servo.traj_lo = 0U;
    s_servo.traj_hi = s_servo.span_cdeg;
    if (s_servo.range_valid) C_Traj_SetRange(0, s_servo.span_cdeg);
}''')
    t = replace(t, '    s_servo.arrived      = 1U;\n    s_servo.move_pending = 0U;\n    s_servo.move_age     = 0xFFFFU;\n    s_servo.paused',
                '    Servo_MultiTurnAbort();\n    s_servo.arrived      = 1U;\n    s_servo.move_pending = 0U;\n    s_servo.move_age     = 0xFFFFU;\n    s_servo.paused')
    start = t.index('static void Servo_EscapeStart(')
    end = t.index('/* 上电嗅探信号线', start)
    t = t[:start]+t[end:]
    t = replace(t, '    else { s_servo.boot_pending = 1U; Servo_EscapeStart(); }   /* 上电就在死区：先脱困再执行上电动作 */', '''    else { s_servo.boot_pending = 1U; s_servo.enc_state = SERVO_ENC_FAULT; D_Motor_Release(0U); }''')
    start = t.index('    if (s_servo.enc_state == SERVO_ENC_ESCAPING)\n    {   /* 脱困结束')
    end = t.index('    s_servo.paused = 0U;', start)
    # The first paused assignment is inside the removed block; locate its closing return.
    end = t.index('    s_servo.paused = 0U;', t.index('        return SERVO_SUBMIT_OK;\n    }', start))
    t = t[:start]+'    Servo_MultiTurnAbort();\n'+t[end:]
    t = replace(t, 'pos  = Fx_ClampRange(raw - s_servo.offset_cdeg, 0, s_servo.span_cdeg);',
                'pos  = Fx_ClampRange(Servo_Unwrap(Servo_WrapAngle(raw - s_servo.offset_cdeg), s_servo.span_cdeg / 2), 0, s_servo.span_cdeg);')
    t = replace(t, '    if (s_servo.enc_state != SERVO_ENC_ESCAPING && Servo_IsPosition()) { s_servo.move_age = 0xFFFFU; Servo_Move(); }', '''    if (Servo_IsPosition()) {
        if (s_servo.mt_active) Servo_PlanMultiTurn();
        else { s_servo.move_age = 0xFFFFU; Servo_Move(); }
    }''')
    t = replace(t, '    int32_t raw, delta;\n\n    if (s_servo.remaining', '    int32_t raw = ENCODER_ANGLE_ERROR, delta;\n\n    if (s_servo.remaining')
    t = replace(t, '    if (s_servo.remaining == 0U) { Servo_Drive(0); return; }\n    Servo_Drive', '''    if (Servo_IsTurns() && s_servo.remaining != UINT32_MAX && raw == ENCODER_ANGLE_ERROR) {
        s_servo.turn_sample_valid = 0U;
        if (++s_servo.enc_fail >= SERVO_ENC_FAIL_MAX) {
            s_servo.enc_state = SERVO_ENC_FAULT;
            Servo_Drive(0);
            A_Servo_Release(0U);
            return;
        }
    }
    if (s_servo.remaining == 0U) { Servo_Drive(0); return; }
    Servo_Drive''')
    t = replace(t, '        Servo_EscapeStart();', '''        s_servo.enc_state = SERVO_ENC_FAULT;
        A_Servo_Release(0U);''')
    t = replace(t, '    C_Traj_Step(&s_servo.ref, hold);', '''    C_Traj_Step(&s_servo.ref, hold);
    if (s_servo.mt_active && s_servo.ref.done && !s_servo.mt_final && !s_servo.paused) {
        Servo_PlanMultiTurn();
        C_Traj_Step(&s_servo.ref, 0U);
    }''')
    t = replace(t, '    if (!s_servo.arrived && !s_servo.move_pending && s_servo.ref.done && C_PosCtrl_Reached()) s_servo.arrived = 1U;', '''    if (!s_servo.arrived && !s_servo.move_pending && s_servo.ref.done && C_PosCtrl_Reached()) {
        if (s_servo.mt_active && !s_servo.paused && s_servo.mt_final) Servo_MultiTurnFinish();
        if (!s_servo.mt_active) s_servo.arrived = 1U;
    }''')
    t = replace(t, '        if (Servo_ReadPosition()) { s_servo.enc_state = SERVO_ENC_OK; Servo_Resync(1U); }', '''        if (Servo_ReadPosition()) {
            s_servo.enc_state = SERVO_ENC_OK;
            Servo_Resync(1U);
            if (s_servo.boot_pending) { s_servo.boot_pending = 0U; Servo_ApplyBootAction(); }
        }''')
    t = replace(t, '    if (s_servo.enc_state == SERVO_ENC_ESCAPING) { Servo_EscapeTick(); return; }\n', '')
    t = replace(t, '    if (s_servo.enc_state == SERVO_ENC_ESCAPING) return SERVO_WORK_RECOVERING;\n', '')
    t = replace(t, '(s_servo.paused && (!Servo_IsPosition() || s_servo.enc_state == SERVO_ENC_ESCAPING))',
                '(s_servo.paused && !Servo_IsPosition())')
    start = t.index('/* ============================== 运动指令')
    t = t[:start]+MULTITURN+'\n'+t[start:]
    return t

MULTITURN = '''/* 磁编码器专用：多圈只扩展坐标/范围，仍走同一个普通Plan、观测器和电压控制器。 */
static void Servo_PlanMultiTurn(void)
{
    int32_t here = C_Traj_GetPos(), delta = s_servo.mt_target - here;
    uint32_t distance = (uint32_t)Fx_Abs32(delta), ms;
    int32_t waypoint = s_servo.mt_target;
    s_servo.mt_final = 1U;
    ms = (uint32_t)((int64_t)s_servo.mt_turn_ms * distance / CDEG_RANGE);
    if (ms > 60000U) {
        int32_t segment = (int32_t)((int64_t)distance * 60000 / ms);
        waypoint = here + (delta < 0 ? -segment : segment);
        ms = 60000U;
        s_servo.mt_final = 0U;
    }
    s_servo.arrived = 0U;
    C_Traj_Plan(waypoint, (uint16_t)ms);
}

static void Servo_MultiTurnFinish(void)
{
    uint16_t target = s_servo.target_angle;
    s_servo.mt_active = 0U;
    s_servo.circular_valid = 0U;
    (void)Servo_ReadPosition();
    /* 360度两端同一物理角度：折回时取目标所在分支，防止一整圈的假误差。 */
    s_servo.angle_circ = Servo_Unwrap(Servo_WrapAngle(s_servo.angle_circ), target);
    s_servo.angle = (uint16_t)Fx_ClampRange(s_servo.angle_circ, 0, s_servo.span_cdeg);
    Servo_ObsInit();
    /* 多圈坐标折回单圈后，旧保持锚点仍在展开坐标，须按坐标变化重置后重新捕获。 */
    C_PosCtrl_Reset();
    Servo_SetTrajRange();
    C_Traj_Hold(target);
    C_Traj_Step(&s_servo.ref, 0U);
    s_servo.arrived = 1U;
}

static void Servo_MultiTurnAbort(void)
{
    if (!s_servo.mt_active) return;
    s_servo.mt_active = 0U;
    s_servo.circular_valid = 0U;
    (void)Servo_ReadPosition();
    Servo_ObsInit();
    C_PosCtrl_Reset(); /* 中止多圈并改变保持目标，按当前控制器接口清除旧坐标状态 */
    Servo_SetTrajRange();
    C_Traj_Hold(s_servo.angle);
    C_Traj_Step(&s_servo.ref, 0U);
    s_servo.target_angle = s_servo.angle;
}

uint8_t A_Servo_SubmitTurns(uint16_t pwm, uint8_t turns, uint16_t turn_ms)
{
    uint16_t target;
    int32_t here, delta, total, lo, hi;
    if (pwm < SERVO_PWM_MIN || pwm > SERVO_PWM_MAX || pwm < g_config.pulse_lo
        || pwm > g_config.pulse_hi || turns > 9U) return SERVO_SUBMIT_VALUE;
    if (!A_Protect_CanDrive() || !Servo_IsPosition()
        || !s_servo.range_valid || s_servo.enc_state != SERVO_ENC_OK
        || A_Protect_Fault() != PROT_FAULT_NONE) return SERVO_SUBMIT_STATE;
    if (!Servo_ReadPosition()) return SERVO_SUBMIT_STATE;
    if (s_servo.torque != SERVO_TORQUE_ON) A_Servo_RestoreTorque();
    if (s_servo.mt_active || s_servo.paused) { Servo_MultiTurnAbort(); Servo_Resync(0U); }
    if (C_Traj_Is_Done()) C_Traj_Hold(s_servo.obs.pos);
    target = Servo_PulseToAngle(Q3(pwm));
    here = C_Traj_GetPos();
    delta = (int32_t)target - here;
    total = (int32_t)turns * CDEG_RANGE + Fx_Abs32(delta);
    if (delta < 0) total = -total;
    if (!total) return Servo_SubmitQ3(Q3(pwm), 0U);
    s_servo.mt_target = here + total;
    s_servo.mt_turn_ms = turn_ms;
    s_servo.mt_active = 1U;
    s_servo.target_angle = target;
    s_servo.paused = s_servo.timed_position = s_servo.move_pending = 0U;
    s_servo.resume_valid = 1U;
    s_servo.move_age = 0xFFFFU;
    s_servo.remaining = 0U;
    lo = (total < 0) ? s_servo.mt_target : here;
    hi = (total < 0) ? here : s_servo.mt_target;
    C_Traj_SetRangeOpen(lo, hi);
    Servo_PlanMultiTurn();
    return SERVO_SUBMIT_OK;
}
'''

def sensors(old_header, circular):
    """Share unit conversions/current semantics while keeping sensor geometry."""
    h = read(BASE / 'Application/A_Sensor.h')
    c = read(BASE / 'Application/A_Sensor.c')
    if circular:
        h = old_header[:old_header.index('#define ADC_VDD_MV')] + h[h.index('#define ADC_VDD_MV'):]
        c = (c[:c.index('/* 本芯片没有除法指令')]
             + '#include "D_mt6701.h"\n\n'
             + c[c.index('/* 电压：'):c.index('/* 读电位器角度')]
             + '/* MT6701整圈反馈，读取失败透传ENCODER_ANGLE_ERROR。 */\n'
             + 'int32_t A_Encoder_Read(void)\n{\n    return (int32_t)D_MT6701_Read_Angle();\n}\n\n'
             + c[c.index('/* 读电池电压'):])
    # Preserve measured potentiometer endpoints and board electrical constants.
    for name in ['ENCODER_POT_ADC_MIN', 'ENCODER_POT_ADC_MAX', 'ENCODER_POT_SPAN_CDEG',
                 'ENCODER_POT_MARGIN_CDEG', 'ENCODER_POT_ADC_FLOAT', 'ADC_VDD_MV',
                 'ADC_FULL_SCALE', 'VOLTAGE_DIV_NUM', 'VOLTAGE_DIV_DEN',
                 'CURRENT_SENSE_MOHM', 'CURRENT_SENSE_GAIN']:
        line = re.search(r'^#define '+name+r'\s+[^\n]+', old_header, re.M)
        if line:
            # Take only the value; baseline comments describe the new interface.
            value = line.group().split()[2]
            h = re.sub(r'^(#define '+name+r'\s+)\S+', lambda m: m[1]+value, h, flags=re.M)
    return h, c


def magnetic_adc():
    """Use the 30kg acquisition chain with one current rank and no PC4 input."""
    h = read(BASE / 'Drivers/D_adc.h')
    h = h.replace('[电流(运放内部输出IN9), 电位器(PC4)]', '[电流(运放内部输出IN9)]')
    h = h.replace('电流、电位器保留4位小数(Q4)', '电流保留4位小数(Q4)')
    h = h.replace(' + 电位器(239.5+12.5)', '').replace('(4U + 82U + 504U + 504U)', '(4U + 82U + 504U)')
    h = re.sub(r'^#define D_ADC_ENCODER_.*\n', '', h, flags=re.M)
    h = re.sub(r'^uint8_t  D_ADC_Encoder_Read.*\n', '', h, flags=re.M)
    c = read(BASE / 'Drivers/D_adc.c')
    c = c.replace('规则组DMA(电流 + 电位器)', '规则组DMA(电流)')
    c = replace(c, '#define RANKS         2U   /* 规则组 [电流, 电位器]，DMA 缓冲区按此交错 */',
                '#define RANKS         1U   /* MT6701走I2C，规则组只采电流 */')
    c = c.replace('DMA循环缓冲：[电流, 电位器] × 20', 'DMA循环缓冲：电流 × 20')
    c = c.replace('uint32_t cur_half, pot_half;', 'uint32_t cur_half;')
    c = re.sub(r'^    uint16_t pot_q4;.*\n', '', c, flags=re.M)
    start = c.index('static uint32_t Block_Sum(')
    c = c[:start] + c[c.index('/* 半块电流', start):]
    c = replace(c, 'RCC_PB2Periph_GPIOC | RCC_PB2Periph_GPIOD | RCC_PB2Periph_ADC1',
                'RCC_PB2Periph_GPIOD | RCC_PB2Periph_ADC1')
    c = re.sub(r'^    gpio.GPIO_Pin  = GPIO_Pin_4;[^\n]*\n    GPIO_Init\(GPIOC, &gpio\);\n', '', c, flags=re.M)
    c = replace(c, 'adc.ADC_ScanConvMode       = ENABLE;', 'adc.ADC_ScanConvMode       = DISABLE;')
    c = c.replace('/* 电流排第1，孔径时刻由触发点决定，运放输出阻抗低用短采样档；电位器源阻抗高用长采样档 */',
                  '/* 规则组仅电流，运放输出阻抗低用短采样档；角度由MT6701经I2C读取 */')
    c = re.sub(r'^    ADC_RegularChannelConfig\(ADC1, ADC_Channel_2,.*\n', '', c, flags=re.M)
    c = re.sub(r'^        s.pot_half = .*\n', '', c, flags=re.M)
    c = re.sub(r'^    s.pot_q4 = .*\n', '', c, flags=re.M)
    start = c.index('uint8_t D_ADC_Encoder_Read(')
    c = c[:start] + c[c.index('uint16_t D_ADC_Voltage_Read(', start):]
    return h, c


def align():
    for name in VARIANTS:
        p = ROOT / name
        circular = 'IIC' in name
        old_config = read(p / 'Application/A_Config.h')
        old_sensor = read(p / 'Application/A_Sensor.h')
        calibration_mode = re.search(r'^#define APP_MODE_CALIB\s*$', read(p / 'User/main.c'), re.M) is not None
        # Keep sensor geometry, MT6701, and the IIC one-rank current DMA.
        for directory in ['Application', 'Common', 'Drivers', 'System', 'User']:
            for src in (BASE / directory).glob('*'):
                if not src.is_file(): continue
                if src.name in ['A_Config.h', 'A_Servo.c', 'A_Sensor.h', 'A_Sensor.c']: continue
                if circular and src.name in ['D_adc.c', 'D_adc.h']: continue
                (p / directory / src.name).write_bytes(src.read_bytes())
        for obsolete in ['C_Friction.c', 'C_Friction.h', 'C_Plant_Adapt.c', 'C_Plant_Adapt.h']:
            (p / 'Common' / obsolete).unlink(missing_ok=True)
        write(p / 'Application/A_Config.h', config(old_config, circular))
        write(p / 'Application/A_Servo.c', servo(circular))
        sensor_h, sensor_c = sensors(old_sensor, circular)
        write(p / 'Application/A_Sensor.h', sensor_h)
        write(p / 'Application/A_Sensor.c', sensor_c)
        if circular:
            adc_h, adc_c = magnetic_adc()
            write(p / 'Drivers/D_adc.h', adc_h)
            write(p / 'Drivers/D_adc.c', adc_c)
            h = read(p / 'Application/A_Servo.h')
            h = h.replace('、脱困中排队', '')
            h = replace(h, 'void     A_Servo_Pause(void);', 'uint8_t  A_Servo_SubmitTurns(uint16_t pwm, uint8_t turns, uint16_t turn_ms); /* 0~9圈，T为每圈ms */\nvoid     A_Servo_Pause(void);')
            write(p / 'Application/A_Servo.h', h)
            main = read(p / 'User/main.c')
            main = replace(main, '#include "D_motor.h"', '#include "D_motor.h"\n#include "D_mt6701.h"')
            main = replace(main, '    D_TMP112_Init();', '    D_MT6701_Init();                   /* MT6701与TMP112共用I2C */\n    D_TMP112_Init();')
            main = main.replace('I2C1  400kHz，温度传感器TMP112用', 'I2C1  400kHz，MT6701+TMP112共用').replace('等电位器与ADC滤波读数稳定', '等磁编码器读数稳定')
            main = main.replace('规则组DMA + 电压注入', '电流DMA + 电压注入')
            uart = read(p / 'Application/A_UartCmd.c')
            uart = replace(uart, '    if (n == 9U && body[4]', '''    uint16_t turns;
    if (n == 11U && body[4] == 'N' && body[6] == 'T'
        && Text_Digits(body, 4U, &pwm) && Text_Digits(body + 5, 1U, &turns)
        && Text_Digits(body + 7, 4U, &arg)) {
        (void)A_Servo_SubmitTurns(pwm, (uint8_t)turns, arg);
        return;
    }
    if (n == 9U && body[4]''')
            uart = uart.replace('。电位器不支持整圈的多圈指令 nnnnNnTnnnn', '')
            write(p / 'Application/A_UartCmd.c', uart)
            # Int16 displacement records need a sector narrower than 32767 cdeg.
            # A guarded 270-degree sector uses the exact same identification as ADC.
            cal = read(p / 'Application/A_Calib_Min.c').replace('ENCODER_POT_SPAN_CDEG', 'CAL_SECTOR_CDEG')
            cal = replace(cal, '#define CAL_GUARD_CDEG', '#define CAL_SECTOR_CDEG 27000 /* 磁编码器标定用270度安全扇区，记录位移不跨零点/不溢出int16 */\n#define CAL_GUARD_CDEG')
            cal = replace(cal, '(ENCODER_ANGLE_HI - CAL_GUARD_CDEG)', '(CAL_SECTOR_CDEG - CAL_GUARD_CDEG)')
            cal = cal.replace('电位器', '编码器')
            write(p / 'Application/A_Calib_Min.c', cal)
        else:
            main = read(p / 'User/main.c')
        # Preserve the user's build-mode selection independently of framework alignment.
        main = re.sub(r'^(?:// )?#define APP_MODE_CALIB\s*$',
                      '#define APP_MODE_CALIB' if calibration_mode else '// #define APP_MODE_CALIB',
                      main, flags=re.M)
        write(p / 'User/main.c', main)
        # Update generated Common lists as well, so existing MRS builds stop linking removed modules.
        mk = read(BASE / 'obj/Common/subdir.mk').replace(BASE.name, name)
        write(p / 'obj/Common/subdir.mk', mk)
        print('Aligned', name)

if __name__ == '__main__':
    align()
