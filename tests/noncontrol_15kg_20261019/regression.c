/* Real production modules, only hardware and plant supplied by the prior audit. */
#define main original_audit_main
#include "../full_audit_20261019/audit.c"
#undef main

static int baseline;
static void expect(int result, const char *name, int was_broken)
{
    printf("%s %s\n", result ? "PASS" : "FAIL", name);
    assert(result == !(baseline && was_broken));
}

static void factory(unsigned entry)
{
    if(entry<2){text(entry?"#000PCLE!":"#000PCLE0!");return;}
    uint8_t f[10]={0xaa,0x66,0,entry==2?7:8,0};
    unsigned n=f[4];f[5+n]=Bin_Crc8(f+2,3+n);f[6+n]=0x66;f[7+n]=0xaa;
    for(unsigned i=0;i<8+n;i++)Wire_Byte(f[i]);
}

static void factory_tests(void)
{
    unsigned good=0;
    for(unsigned entry=0;entry<4;entry++)for(unsigned mode=0;mode<3;mode++){
        reset();if(mode)A_Servo_Release(mode==2);factory(entry);
        good+=(g_runtime_config.torque_mode==1 && A_Servo_TorqueOn());
    }
    expect(good==12,"factory_all_four_entries_all_user_modes",1);
    for(unsigned entry=0;entry<4;entry++){
        reset();voltage=5800;ticks(70);factory(entry);ticks(30);
        assert(s_prot.fault && !A_Servo_TorqueOn());
        voltage=7670;ticks(600);assert(A_Servo_TorqueOn());
    }
    expect(1,"factory_retains_active_fault_then_holds_here",0);
}

static void ownership_tests(void)
{
    unsigned good=0;
    for(unsigned timing=0;timing<2;timing++)for(unsigned mode=0;mode<3;mode++){
        reset();if(timing && mode)A_Servo_Release(mode==2);
        voltage=5800;ticks(70);
        if(!timing && mode)A_Servo_Release(mode==2);
        voltage=7670;ticks(600);
        good+=(A_Servo_TorqueOn()==!mode && g_runtime_config.torque_mode==(mode==2?2:mode==1?0:1));
    }
    expect(good==6,"fault_recovery_preserves_latest_user_release_or_damping",1);
    reset();command(2200,0);ticks(40);voltage=5800;ticks(70);
    unsigned request=g_runtime_config.torque_mode;
    A_Servo_RestoreTorque();assert(!A_Servo_TorqueOn());
    assert(A_Servo_Submit(1800,0)==SERVO_SUBMIT_STATE);
    assert(g_runtime_config.torque_mode==request);
    voltage=7670;ticks(600);
    assert(!s_servo.resume_valid && !s_servo.move_pending);
    expect(1,"fault_rejects_commands_and_discards_old_motion",0);
}

static void detector_tests(void)
{
    for(unsigned path=0;path<3;path++){
        reset();current_override=4000;g_runtime_config.prot_current_time_ms=100;
        for(int t=0;t<150;t++){
            if(t%20==0)param(path==0?PARAM_RT_TORQUE_LIMIT:path==1?PARAM_TORQUE_LIMIT:PARAM_RT_HOLD_KP,(t/20)%2?40:50,2);
            tick();
        }
        expect((s_prot.fault&PROT_FAULT_OVERCUR)!=0,path==0?"sram_torque_cannot_reset_overcurrent":path==1?"flash_torque_cannot_reset_overcurrent":"hold_gain_cannot_reset_overcurrent",path==0);
    }
    reset();temp=600;g_runtime_config.prot_temp_time_ms=1000;
    for(int t=0;t<1250;t++){if(t%100==0)param(PARAM_RT_TORQUE_LIMIT,(t/100)%2?40:50,2);tick();}
    expect((s_prot.fault&PROT_FAULT_OVERTEMP)!=0,"sram_torque_cannot_reset_temperature",1);
    reset();run_protect=0;current_override=2500;A_Protect_Task();
    int limited=s_prot.pwm_limit;
    assert(param(PARAM_TORQUE_LIMIT,A_Config_TorqueMax(),2)==BIN_ST_OK);
    expect(s_prot.pwm_limit==limited,"same_flash_torque_write_preserves_derating",1);
    reset();g_runtime_config.torque_limit_ckgcm=0;
    expect(Servo_SubmitQ3(Q3(1800),0)==SERVO_SUBMIT_STATE,"pwm_internal_submit_checks_zero_torque_gate",1);
}

static void bus_and_window_tests(void)
{
    reset();voltage=0;A_Servo_Init();voltage=7670;ticks(600);command(1800,0);ticks(3000);
    expect(s_servo.to_duty_q16!=0,"zero_bus_initialization_recovers_without_nonzero_output",1);
    unsigned bad=0,accepted=0;
    for(int lo=60;lo<=90;lo++)for(int hi=60;hi<=90;hi++){
        reset();
        if(param(PARAM_VOLTAGE_MIN,lo,1)!=BIN_ST_OK)continue;
        if(param(PARAM_VOLTAGE_MAX,hi,1)!=BIN_ST_OK)continue;
        accepted++;bad+=(lo+2*PROT_VOLTAGE_HYST_DV>=hi);
        Config_t stored=g_config;Config_FillDefaults();Config_Repair(&stored);
        assert(stored.voltage_min_dv==lo && stored.voltage_max_dv==hi);
    }
    printf("DATA accepted_windows=%u empty_or_single_point=%u\n",accepted,bad);
    expect(bad==0,"all_accepted_voltage_windows_have_recovery_interior",1);
    reset();adc_bad=1;int start=ms;
    while(s_servo.enc_state!=SERVO_ENC_FAULT && ms-start<5000)tick();
    expect(ms-start==3005,"encoder_search_two_equal_1500ms_budgets",1);
}

static void combined_tests(void)
{
    /* Two faults clear independently; factory restore never acknowledges away a fault.
     * The last explicit user request remains the owner after all faults clear. */
    for(unsigned entry=0;entry<4;entry++)for(unsigned mode=0;mode<3;mode++){
        reset();g_runtime_config.damping_percent=37;temp=600;voltage=5800;ticks(300);
        assert((s_prot.fault&(PROT_FAULT_OVERTEMP|PROT_FAULT_UNDERVOLT))==(PROT_FAULT_OVERTEMP|PROT_FAULT_UNDERVOLT));
        factory(entry);assert(g_runtime_config.torque_mode==1 && !A_Servo_TorqueOn());
        A_Servo_SetDamping(37);
        if(mode)A_Servo_Release(mode==2);
        voltage=7670;ticks(600);assert(!A_Servo_TorqueOn() && (s_prot.fault&PROT_FAULT_OVERTEMP));
        temp=250;ticks(300);
        assert(!s_prot.fault && A_Servo_TorqueOn()==!mode);
        assert(g_runtime_config.torque_mode==(mode==2?2:mode==1?0:1));
        assert(g_runtime_config.damping_percent==37 && !s_servo.resume_valid);
    }
    puts("PASS multiple_faults_factory_and_latest_user_request_12_combinations");

    reset();run_protect=0;current_override=4000;temp=600;
    g_runtime_config.prot_current_time_ms=1000;g_runtime_config.prot_temp_time_ms=2000;
    for(int i=0;i<25;i++)A_Protect_Task();
    assert(s_prot.cur_cnt==25 && s_prot.temp_cnt==1);
    /* A higher current threshold invalidates only its own history. */
    g_runtime_config.prot_current_limit_ma=4500;A_Protect_Task();
    assert(!s_prot.cur_cnt && s_prot.temp_cnt==1);
    g_runtime_config.prot_current_limit_ma=3500;A_Protect_Task();
    assert(s_prot.cur_cnt==1 && s_prot.temp_cnt==1);
    g_runtime_config.prot_current_limit_ma=3000;A_Protect_Task();
    assert(s_prot.cur_cnt==2 && s_prot.temp_cnt==1);
    /* Duration changes preserve duration evidence, lower thresholds preserve it too. */
    g_runtime_config.prot_current_time_ms=30;A_Protect_Task();
    assert(s_prot.fault&PROT_FAULT_OVERCUR);
    g_runtime_config.prot_temp_limit_dC=650;A_Protect_Task();assert(!s_prot.temp_cnt);
    assert(s_prot.fault&PROT_FAULT_OVERCUR);
    puts("PASS threshold_and_duration_changes_keep_independent_history_and_faults");

    reset();run_protect=0;current_override=2500;
    A_Protect_Task();int limited=s_prot.pwm_limit;assert(limited<CFG_PWM_FULL);
    assert(param(PARAM_TORQUE_LIMIT,A_Config_TorqueMax(),2)==BIN_ST_OK);
    assert(s_prot.pwm_limit==limited);
    g_runtime_config.prot_temp_time_ms=1000;A_Protect_Task();
    assert(s_prot.pwm_limit<=limited);
    puts("PASS same_torque_and_unrelated_parameters_keep_current_derating");

    reset();assert(param(PARAM_TORQUE_LIMIT,0,2)==BIN_ST_OK);ticks(30);
    assert(!A_Servo_TorqueOn());
    assert(param(PARAM_TORQUE_LIMIT,A_Config_TorqueMax(),2)==BIN_ST_OK);
    assert(s_prot.pwm_limit>D_ADC_CUR_MIN_TICKS);
    assert(A_Servo_Submit(1800,0)==SERVO_SUBMIT_OK);ticks(1500);
    assert(fabs(pos-s_servo.target_angle)<50);
    puts("PASS zero_torque_then_restore_can_start_without_current_sample_deadlock");

    reset();g_runtime_config.torque_limit_ckgcm=0;A_Servo_Control();
    assert(!A_Servo_TorqueOn());
    assert(Servo_SubmitQ3(Q3(1800),0)==SERVO_SUBMIT_STATE && !s_servo.resume_valid);
    g_runtime_config.torque_limit_ckgcm=A_Config_TorqueMax();A_Servo_Control();
    assert(A_Servo_TorqueOn() && !s_servo.resume_valid);
    A_Servo_Release(1);assert(Servo_SubmitQ3(Q3(1800),0)==SERVO_SUBMIT_OK && A_Servo_TorqueOn());
    puts("PASS pwm_and_uart_share_drive_gate_and_fresh_motion_restores_torque");

    /* Previously stored narrow windows are repaired on a real config reload. */
    reset();g_config.voltage_min_dv=60;g_config.voltage_max_dv=63;
    A_Config_MarkDirty();assert(A_Config_SaveTask());A_Config_Init();
    assert(g_config.voltage_min_dv==60 && g_config.voltage_max_dv==90);
    puts("PASS invalid_stored_voltage_window_repaired_on_reload");
}

int main(int argc,char **argv)
{
    baseline=argc>1 && !strcmp(argv[1],"baseline");
    factory_tests();ownership_tests();detector_tests();bus_and_window_tests();
    if(!baseline)combined_tests();
    flash_tests();parameter_matrix();
    puts("PASS noncontrol regression complete");return 0;
}
