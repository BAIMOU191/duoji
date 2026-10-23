/* 15kg 堵转保持：当前角度为目标、直接进 HOLD、无积分托举力，状态同普通到位 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "C_Pos_Ctrl.h"
#include "C_Motor_Model.h"

static MotorCal_t cal = {
    .kv_uv = 250, .ka_nv = 6829, .fc_mv = 313, .fs_mv = 546,
    .delay_us = 3206, .cal_mv = 7640, .noise_cdeg = 6, .min_step_cdeg = 9,
    .sign = 1, .rated_vel_dps = 260, .rated_acc_dps2 = 8000, .rated_dec_dps2 = 8000,
};

int main(void)
{
    TrajRef_t ref;
    PosCtrlFeedback_t fb;
    PosCtrlDbg_t dbg;

    C_MotorModel_Init(&cal);
    C_PosCtrl_Init(&cal, 100, 16);
    {
        PosCtrlGains_t g = { .pos_bw = 20, .vel_bw = 70, .int_bw = 20, .load_ratio_x10 = 20 };
        (void)C_PosCtrl_SetGains(&g);
    }
    C_PosCtrl_SetHold(50, 80, 0);

    memset(&ref, 0, sizeof(ref));
    ref.pos = 1000; ref.done = 0; ref.vel = 2000; /* 正在运动中 */
    fb.pos = 800; fb.vel = 2000; fb.valid = 1;

    /* 模拟运动中被堵转：HoldHere 将当前角度设为目标后 StallHold */
    ref.pos = 800; ref.vel = 0; ref.acc = 0; ref.done = 1;
    C_PosCtrl_StallHold();

    /* 1) 直接进 HOLD，不再走 SETTLE 捕获/交接 */
    assert(C_PosCtrl_Reached());
    assert(C_PosCtrl_ArrivalQualified());
    (void)C_PosCtrl_Update(&ref, &fb, 20000, &dbg);
    assert(dbg.state == PC_HOLD);
    assert(dbg.u_i == 0);
    printf("forced HOLD: state=%d qualified reached, u_i=%d u_out=%d\n",
           dbg.state, dbg.u_i, dbg.u_out);

    /* 2) 窗内无积分托举；窗外仍有弹簧 */
    fb.pos = 800;
    (void)C_PosCtrl_Update(&ref, &fb, 20000, &dbg);
    assert(dbg.u_i == 0);
    assert(dbg.u_fb == 0);
    fb.pos = 800 - 200; /* 被推离 2° */
    (void)C_PosCtrl_Update(&ref, &fb, 20000, &dbg);
    assert(dbg.u_i == 0);
    assert(dbg.u_fb > 0);
    printf("window in/out: u_i=%d u_fb(out)=%d\n", dbg.u_i, dbg.u_fb);

    /* 3) 持续窗外不重新积累积分 */
    {
        int i;
        for (i = 0; i < 100; i++) (void)C_PosCtrl_Update(&ref, &fb, 20000, &dbg);
    }
    assert(dbg.u_i == 0);
    printf("after 100ms out-of-window: u_i=%d\n", dbg.u_i);

    /* 4) 新指令后恢复 */
    C_PosCtrl_BeginMotion();
    ref.done = 0; ref.pos = 1200; ref.vel = 1000;
    fb.pos = 800; fb.vel = 0;
    (void)C_PosCtrl_Update(&ref, &fb, 20000, &dbg);
    assert(dbg.state == PC_MOVE);
    printf("after BeginMotion: state=%d (MOVE)\n", dbg.state);

    printf("PASS stall -> target=current, force HOLD, no integral lift\n");
    return 0;
}
