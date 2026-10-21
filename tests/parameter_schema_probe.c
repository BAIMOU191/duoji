/* Export the actual protocol table, factory values and read lengths for document QA. */
#define PARAMETER_SCHEMA_PROBE
#define main firmware_test_main
#include "variant_alignment_sim.c"
#undef main
/* The diagnostic snapshot now reads the raw ADC channel; protocol QA does not
 * exercise that hardware-only diagnostic path. */
uint8_t D_ADC_Encoder_Read(uint16_t *out) { *out = 0U; return 0U; }
#define CONST(k) printf("\"%s\":%lld,",#k,(long long)(k))
int main(void) {
    reset();Config_FillDefaults();A_Config_ReloadRuntime(1);
    printf("{\"constants\":{");
    CONST(FW_SERVO_TYPE);CONST(FW_TORQUE);CONST(CFG_DIR_INVERT);
    CONST(FW_HW_VERSION);CONST(FW_VOLTAGE);CONST(MOTOR_CAL_MV);
    CONST(SERVO_BOOT_MOVE_MS);
    CONST(SERVO_VERSION_MAJOR);CONST(SERVO_VERSION_MINOR);CONST(SERVO_VERSION_PATCH);
    CONST(CFG_DEF_HOLD_KP);CONST(CFG_DEF_HOLD_KI);
    CONST(CFG_DEF_RESOLUTION_CDEG);
    CONST(CFG_HOLD_KP_MAX);CONST(CFG_HOLD_KI_MAX);
    CONST(CFG_RESOLUTION_MIN_CDEG);
    CONST(CFG_SMOOTH_MIN_MS);CONST(CFG_SMOOTH_MAX_MS);CONST(CFG_MOVE_MIN_MAX_MS);
    CONST(PROT_STALL_THRESHOLD_MIN_CDEG);CONST(PROT_STALL_THRESHOLD_MAX_CDEG);
    CONST(MOTOR_KV_UV);CONST(MOTOR_KA_NV);CONST(MOTOR_NOISE_CDEG);CONST(MOTOR_MIN_STEP_CDEG);
    CONST(MOTOR_RATED_VEL_DPS);CONST(MOTOR_RATED_ACC_DPS2);CONST(MOTOR_RATED_DEC_DPS2);
    CONST(PROT_TEMP_SAMPLE_MS);CONST(PROT_CURRENT_COOLDOWN_MS);
    CONST(PROT_VOLTAGE_TRIP_MS);CONST(PROT_VOLTAGE_RECOVER_MS);
    CONST(PROT_VOLTAGE_HYST_DV);CONST(PROT_TEMP_HYST_DC);
    CONST(TRAJ_JMAX_MAX_CDPSS3);
    printf("\"config_size\":%u},\"params\":[",(unsigned)sizeof(Config_t));
    for(unsigned i=0;i<sizeof(k_params)/sizeof(k_params[0]);i++) {
        printf("%s{\"id\":%u,\"size\":%u,\"flags\":%u,\"min\":%lu,\"max\":%lu,"
               "\"flash\":%u,\"runtime\":%u,\"default\":%lu}",i?",":"",k_params[i].id,
               k_params[i].size,k_params[i].flags,(unsigned long)k_params[i].min,(unsigned long)k_params[i].max,
               k_params[i].cfg!=PF_NONE,k_params[i].rt!=PF_NONE,
               (unsigned long)Param_Load(Param_Addr(k_params[i].cfg,k_params[i].rt),k_params[i].size));
    }
    printf("],\"reads\":{");
    unsigned sep=0;
    for(unsigned i=0;i<256;i++) {
        uint8_t data[32],len=0;
        if(Bin_ParamRead((uint8_t)i,data,&len)==BIN_ST_OK)
            printf("%s\"%02X\":%u",sep++?",":"",i,len-1);
    }
    printf("},\"actions\":{");
    const uint8_t actions[]={PARAM_MOTION_REQUEST,PARAM_MOTION_ACTION,PARAM_CAL_ACTION};
    for(unsigned a=0;a<sizeof(actions);a++) {
        unsigned accepted=0;
        for(uint8_t size=1;size<=4;size++) {
            uint8_t data[4]={0},baud=0;
            if(Bin_ParamWrite(actions[a],data,size,&baud)!=BIN_ST_LENGTH) {
                assert(!accepted);accepted=size;
            }
        }
        assert(accepted);printf("%s\"%02X\":%u",a?",":"",actions[a],accepted);
    }
    printf("},\"text\":[");
    for(unsigned i=0;i<sizeof(k_text)/sizeof(k_text[0]);i++)
        printf("%s[\"%s\",%u]",i?",":"",k_text[i].key,k_text[i].digits);
    /* Documented behavioral distinctions are checked on the real application code. */
    reset();A_Servo_Release(0);A_Config_Default(0);A_Servo_ApplyConfig();
    assert(g_runtime_config.torque_mode==1 && A_Servo_GetWorkState()==SERVO_WORK_RELEASE_LOW);
    reset();g_config.servo_mode=SERVO_MODE_TURNS_CW;A_Servo_ApplyConfig();
    assert(A_Servo_Submit(1500,0)==SERVO_SUBMIT_OK);
    assert(!s_servo.motor_mv && A_Servo_GetWorkState()==SERVO_WORK_MOVING);
    printf("],\"removed_addresses\":{");
    const uint8_t removed[]={0x36,0x8D,0x39,0x90,0x8C};
    for(unsigned i=0;i<sizeof(removed);i++) {
        uint8_t out[32],len=0,baud=0,data[2]={0};
        unsigned read_status=Bin_ParamRead(removed[i],out,&len);
        unsigned write_status=Bin_ParamWrite(removed[i],data,2,&baud);
        assert(Param_Find(removed[i])<0 && read_status==BIN_ST_VALUE);
        assert(write_status==(removed[i]<0x80 ? BIN_ST_VALUE : BIN_ST_READONLY));
        printf("%s\"%02X\":{\"read_status\":%u,\"write_status\":%u}",i?",":"",removed[i],read_status,write_status);
    }
    printf("},\"hold_boundary_checks\":[");
    const uint8_t hold_ids[]={PARAM_HOLD_KP,PARAM_RT_HOLD_KP,PARAM_HOLD_KI,PARAM_RT_HOLD_KI};
    for(unsigned i=0;i<sizeof(hold_ids);i++) {
        int8_t ix=Param_Find(hold_ids[i]);
        uint16_t maximum=(uint16_t)k_params[ix].max;
        uint8_t data[2],baud=0;
        const uint16_t values[]={0,maximum,(uint16_t)(maximum+1),65535};
        for(unsigned j=0;j<4;j++) {
            Bin_Put16(data,values[j]);
            assert(Bin_ParamWrite(hold_ids[i],data,2,&baud)==(j<2 ? BIN_ST_OK : BIN_ST_VALUE));
        }
        assert(Bin_ParamWrite(hold_ids[i],data,1,&baud)==BIN_ST_LENGTH);
        printf("%s\"%02X\"",i?",":"",hold_ids[i]);
    }
    printf("]}\n");
    return 0;
}
