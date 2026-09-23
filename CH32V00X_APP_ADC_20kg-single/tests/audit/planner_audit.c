#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "C_Traj_Planner.c"
static uint32_t seed=47;
static uint32_t random32(void) {seed=seed*1664525U+1013904223U;return seed;}
int main(int argc,char **argv) {
    TrajRef_t r={0};int failures=0;
    C_Traj_Init(13500,TUNE_SMOOTH_ACC,TUNE_SMOOTH_DEC);C_Traj_Set_Range(200,26800);
    C_Traj_Plan(25000,0);for(int i=0;i<200;i++)C_Traj_Step(&r,0);
    int pos=r.pos,vel=r.vel;C_Traj_Plan(pos+1,0);
    printf("moving retarget: pos=%d velocity=%d duration=%ldms\n",pos,vel,(long)s_tj.T);
    if(s_tj.T>2000)failures++;
    if(argc>1 && argv[1][0]=='s')return failures?1:0;
    for(int test=0;test<10000;test++) {
        if(test%10==0) {C_Traj_Init(13500,(uint8_t)(random32()%101),(uint8_t)(random32()%101));C_Traj_Set_Range(200,26800);C_Traj_Step(&r,0);}
        int target=(int)(random32()%27001);uint16_t time=(test%3==0)?(uint16_t)random32():0;
        C_Traj_Plan(target,time);
        int steps=1+(int)(random32()%2000);
        for(int k=0;k<steps;k++) {
            int before=r.pos;C_Traj_Step(&r,0);
            if(r.pos<200 || r.pos>26800 || abs(r.vel)>TRAJ_VMAX_CDPS+2 || abs(r.pos-before)>TRAJ_VMAX_CDPS/1000+4) {
                if(failures<10)printf("FAIL test=%d k=%d target=%d time=%u before=%d pos=%ld vel=%ld T=%ld\n",test,k,target,time,before,(long)r.pos,(long)r.vel,(long)s_tj.T);
                failures++;break;
            }
        }
    }
    printf("planner random plans=10000 failures=%d\n",failures);return failures?1:0;
}
