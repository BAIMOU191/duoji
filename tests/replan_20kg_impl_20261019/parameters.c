#define main protocol_original_main
#include "protocol_fixture.c"
#undef main

int main(void)
{
    reset();
    assert(g_runtime_config.traj_vmax_dps==MOTOR_RATED_VEL_DPS*9U/10U);
    assert(g_runtime_config.traj_amax_dps2==MOTOR_RATED_ACC_DPS2*7U/10U);
    assert(g_runtime_config.traj_dmax_dps2==MOTOR_RATED_DEC_DPS2*7U/10U);
    assert(g_runtime_config.traj_move_min_ms==150U && TUNE_LOAD_RATIO_X10==10U);
    assert(write_param(PARAM_RT_TRAJ_AMAX,1234,4)==BIN_ST_OK);
    assert(s_tj.amax_cdpss==123400 && g_config.traj_amax_dps2==CFG_DEF_AMAX_DPS2);
    assert(write_param(PARAM_TRAJ_AMAX,2345,4)==BIN_ST_OK);
    assert(s_tj.amax_cdpss==234500 && A_Config_SaveTask());
    A_Config_Init();assert(g_runtime_config.traj_amax_dps2==2345);
    assert(write_param(PARAM_RT_TRAJ_VMAX,0,2)==BIN_ST_OK);
    assert(write_param(PARAM_RT_TRAJ_AMAX,0,4)==BIN_ST_OK);
    assert(write_param(PARAM_RT_TRAJ_DMAX,0,4)==BIN_ST_OK);
    assert(s_tj.vmax_cdps==MOTOR_RATED_VEL_DPS*100);
    assert(s_tj.amax_cdpss==MOTOR_RATED_ACC_DPS2*100);
    assert(s_tj.dmax_cdpss==MOTOR_RATED_DEC_DPS2*100);
    A_Config_Default(1);A_Servo_ApplyConfig();
    assert(s_tj.vmax_cdps==CFG_DEF_VMAX_DPS*100 && s_tj.amax_cdpss==CFG_DEF_AMAX_DPS2*100);
    assert(s_tj.dmax_cdpss==CFG_DEF_DMAX_DPS2*100);
    saved=g_config;saved.servo_id=23;saved.baud_code=6;saved.image_tag=999;saved_valid=1;
    saved.traj_vmax_dps=1;saved.traj_amax_dps2=1;
    A_Config_Init();
    assert(g_config.servo_id==23 && g_config.baud_code==6);
    assert(g_runtime_config.traj_vmax_dps==CFG_DEF_VMAX_DPS);
    assert(g_runtime_config.traj_amax_dps2==CFG_DEF_AMAX_DPS2);
    puts("PASS per_variant_defaults_flash_sram_no_scaling_rated_limits_and_image_reload");
    return 0;
}
