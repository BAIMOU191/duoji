#define main existing_protocol_regression_main
#include "variant_alignment_sim.c"
#undef main
#include "encoder_stub.h"

static uint32_t read_value(uint8_t id)
{
    uint8_t bytes[8],length=0;
    assert(Bin_ParamRead(id,bytes,&length)==BIN_ST_OK && bytes[0]==id);
    return length==3 ? Bin_Get16(bytes+1) : Bin_Get32(bytes+1);
}

int main(void)
{
    reset();
    assert(CFG_DEF_VMAX_DPS==163U && CFG_DEF_AMAX_DPS2==1400U && CFG_DEF_DMAX_DPS2==1400U);
    assert(read_value(PARAM_TRAJ_VMAX)==163U && read_value(PARAM_RT_TRAJ_VMAX)==163U);
    assert(read_value(PARAM_TRAJ_AMAX)==1400U && read_value(PARAM_RT_TRAJ_AMAX)==1400U);
    assert(read_value(PARAM_TRAJ_DMAX)==1400U && read_value(PARAM_RT_TRAJ_DMAX)==1400U);
    assert(s_tj.vmax_cdps==16300 && s_tj.amax_cdpss==140000 && s_tj.dmax_cdpss==140000);
    puts("PASS actual_defaults_and_readback V=163 A=1400 D=1400");

    assert(write_param(PARAM_RT_TRAJ_VMAX,100U,2)==BIN_ST_OK);
    assert(write_param(PARAM_RT_TRAJ_AMAX,5000U,4)==BIN_ST_OK);
    assert(write_param(PARAM_RT_TRAJ_DMAX,4500U,4)==BIN_ST_OK);
    assert(read_value(PARAM_RT_TRAJ_VMAX)==100U && read_value(PARAM_TRAJ_VMAX)==163U);
    assert(s_tj.vmax_cdps==10000 && s_tj.amax_cdpss==500000 && s_tj.dmax_cdpss==450000);
    assert(write_param(PARAM_TRAJ_AMAX,6000U,4)==BIN_ST_OK);
    assert(s_tj.amax_cdpss==600000 && read_value(PARAM_TRAJ_AMAX)==6000U);
    assert(A_Config_SaveTask());A_Config_Init();
    assert(read_value(PARAM_RT_TRAJ_VMAX)==163U && read_value(PARAM_RT_TRAJ_AMAX)==6000U);
    puts("PASS flash_and_sram_writes_use_actual_values_without_hidden_scaling");

    assert(write_param(PARAM_RT_TRAJ_VMAX,MOTOR_RATED_VEL_DPS,2)==BIN_ST_OK);
    assert(write_param(PARAM_RT_TRAJ_AMAX,MOTOR_RATED_ACC_DPS2,4)==BIN_ST_OK);
    assert(write_param(PARAM_RT_TRAJ_DMAX,MOTOR_RATED_DEC_DPS2,4)==BIN_ST_OK);
    assert(s_tj.vmax_cdps==26000 && s_tj.amax_cdpss==800000 && s_tj.dmax_cdpss==800000);
    A_Config_Default(1);A_Servo_ApplyConfig();
    assert(s_tj.vmax_cdps==16300 && s_tj.amax_cdpss==140000 && s_tj.dmax_cdpss==140000);
    puts("PASS rated_limits_unchanged_and_factory_restore_applies_new_defaults");

    saved=g_config;saved.servo_id=23;saved.baud_code=6;saved.image_tag=999U;
    saved.traj_vmax_dps=234U;saved.traj_amax_dps2=saved.traj_dmax_dps2=5600U;saved_valid=1;
    A_Config_Init();
    assert(g_config.servo_id==23 && g_config.baud_code==6);
    assert(g_runtime_config.traj_vmax_dps==163U && g_runtime_config.traj_amax_dps2==1400U);
    assert(g_runtime_config.traj_dmax_dps2==1400U);
    puts("PASS changed_image_uses_new_defaults_preserving_id_and_baud");
    return 0;
}
