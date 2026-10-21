#define main original_sim_main
#include "../replan_jolt_20261019/sim.c"
#undef main

static void initialize(int dir)
{
    C_Traj_Init(13500);C_Traj_SetRange(0,27000);C_Traj_SetLimits(17000,420000,420000,8400000);
    C_Traj_SetMoveMinMs(150);C_Traj_Plan(13500+dir*4500,1000);
}

int main(void)
{
    int joins=0,steps=0;
    int times[]={50,150,300,700},deltas[]={-3000,-500,500,3000},durations[]={200,1000,3000,65535};
    for(int dir=-1;dir<=1;dir+=2)for(unsigned ti=0;ti<4;ti++)
        for(unsigned di=0;di<4;di++)for(unsigned du=0;du<4;du++){
            initialize(dir);TrajRef_t ref={0};
            for(int t=0;t<times[ti];t++)C_Traj_Step(&ref,0);
            TrajRef_t prev=ref;int goal=13500+dir*4500+deltas[di];
            C_Traj_PlanTimed(goal,durations[du]);
            if(!s_tj.poly)continue;
            joins++;
            for(int t=0;t<durations[du];t++){
                C_Traj_Step(&ref,0);steps++;
                assert(ref.pos>=0 && ref.pos<=27000);
                assert((ref.pos-prev.pos)*dir>=-1);
                assert(abs(ref.vel)<=17001 && abs(ref.acc)<=420001);
                assert(abs(ref.acc-prev.acc)<=8402);
                assert(abs(ref.pos-prev.pos)<=((abs(ref.vel)>abs(prev.vel)?abs(ref.vel):abs(prev.vel))+999)/1000+3);
                prev=ref;
            }
            assert(ref.done && ref.pos==goal && !ref.vel && !ref.acc);
        }
    assert(joins>20);
    printf("PASS polynomial signed joins=%d steps=%d V_A_D_J_range_and_terminal_checks\n",joins,steps);
    initialize(1);TrajRef_t r;for(int i=0;i<700;i++)C_Traj_Step(&r,0);
    C_Traj_PlanTimed(18500,1000);assert(s_tj.poly);
    /* Saturation cannot freeze the first decelerating transition. */
    C_Traj_Step(&r,1);assert(s_tj.t==1);
    C_Traj_Stop();assert(!s_tj.poly);
    for(int i=0;i<2000&&!C_Traj_Is_Done();i++)C_Traj_Step(&r,0);
    assert(C_Traj_Is_Done());
    C_Traj_Hold(14000);assert(!s_tj.poly && C_Traj_Is_Done());
    puts("PASS polynomial_stop_hold_and_saturated_braking");
    initialize(1);for(int i=0;i<300;i++)C_Traj_Step(&r,0);
    __typeof__(s_tj) before=s_tj;C_Traj_PlanTimed(5000,1000);assert(!s_tj.poly);
    __typeof__(s_tj) timed=s_tj;s_tj=before;C_Traj_Plan(5000,1000);
    assert(!memcmp(&timed,&s_tj,sizeof(s_tj)));
    puts("PASS opposite_direction_fallback_matches_original_plan");
    return 0;
}
