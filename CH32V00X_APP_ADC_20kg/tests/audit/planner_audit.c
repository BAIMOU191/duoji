#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "C_Traj_Planner.c"
static uint32_t seed=47;
static uint32_t random32(void) {seed=seed*1664525U+1013904223U;return seed;}
/* [lo,hi] 上对数均匀取值：V/A/J 的可设范围跨好几个数量级 */
static int32_t log_uniform(double lo,double hi) {return (int32_t)lround(lo*pow(hi/lo,(random32()>>8)/16777216.0));}
int main(int argc,char **argv) {
    TrajRef_t r={0};int failures=0;
    C_Traj_Init(13500);C_Traj_Set_Range(200,26800);
    C_Traj_Plan(25000,0);for(int i=0;i<200;i++)C_Traj_Step(&r,0);
    int pos=r.pos,vel=r.vel;C_Traj_Plan(pos+1,0);
    printf("moving retarget: pos=%d velocity=%d duration=%ldms\n",pos,vel,(long)s_tj.T);
    if(s_tj.T>2000)failures++;
    if(argc>1 && argv[1][0]=='s')return failures?1:0;
    /* 随机轨迹上限 x 随机目标/请求时间/打断时刻：参考不出行程、不超速，
     * 加速度不超过 A、逐拍加加速度不超过 J(重规划那一拍和撞行程边界前后除外：规划器状态只含位置和速度，
     * 边界上加速度被强制归零)。逐拍加加速度另含段内时间u的Q15量化，段长t拍时相对误差约 t/32768 */
    int32_t vset=TRAJ_VMAX_CDPS,aset=TRAJ_AMAX_CDPSS,dset=TRAJ_DMAX_CDPSS,jset=TRAJ_JMAX_CDPSS3;
    double worst_a=0,worst_j=0;int worst_end=0;
    for(int test=0;test<10000;test++) {
        if(test%10==0) {
            vset=log_uniform(500,TRAJ_VMAX_HW_CDPS);aset=log_uniform(TRAJ_AMAX_MIN_CDPSS,2000000);
            dset=log_uniform(TRAJ_AMAX_MIN_CDPSS,2000000);
            jset=log_uniform(TRAJ_JMAX_MIN_CDPSS3,TRAJ_JMAX_MAX_CDPSS3);
            C_Traj_Init(13500);C_Traj_Set_Limits(vset,aset,dset,jset);C_Traj_Set_Range(200,26800);C_Traj_Step(&r,0);
        }
        int target=(int)(random32()%27001);uint16_t time=(test%3==0)?(uint16_t)random32():0;
        C_Traj_Plan(target,time);
        int steps=1+(int)(random32()%2000);
        int32_t last_acc=r.acc;int last_edge=1;
        for(int k=0;k<steps;k++) {
            int before=r.pos,was_done=C_Traj_Is_Done();C_Traj_Step(&r,0);
            if(!was_done && C_Traj_Is_Done() && abs(r.pos-before)>worst_end)worst_end=abs(r.pos-before); /* 终点吸收的残差 */
            int edge=(r.pos<=200 || r.pos>=26800);
            double jtol=1.03+(double)(s_tj.t1>s_tj.t3?s_tj.t1:s_tj.t3)/32768.0;
            double ra=fabs((double)r.acc)/(((int64_t)r.acc*r.vel<0)?dset:aset), rj=fabs((double)(r.acc-last_acc))*1000.0/jset/jtol;
            int jchk=(k>0 && !edge && !last_edge);
            if(ra>worst_a)worst_a=ra;
            if(jchk && rj>worst_j)worst_j=rj;
            if(r.pos<200 || r.pos>26800 || abs(r.vel)>vset+2 || abs(r.pos-before)>vset/1000+4
               || ra>1.02 || (jchk && fabs((double)(r.acc-last_acc))>jset*jtol/1000.0+2)) {
                if(failures<10)printf("FAIL test=%d k=%d target=%d time=%u V=%ld A=%ld J=%ld before=%d pos=%ld vel=%ld acc=%ld last_acc=%ld T=%ld\n",
                    test,k,target,time,(long)vset,(long)aset,(long)jset,before,(long)r.pos,(long)r.vel,(long)r.acc,(long)last_acc,(long)s_tj.T);
                failures++;break;
            }
            last_acc=r.acc;last_edge=edge;
        }
    }
    /* 终点吸收的残差：整数拍分辨不开时的到位误差，须远小于最小可靠步长 */
    if(worst_end>TUNE_RESOLUTION_CDEG/2)failures++;
    printf("planner random plans=10000 worst |a|/限=%.3f worst jerk/(J*tol)=%.3f worst end step=%dcdeg failures=%d\n",worst_a,worst_j,worst_end,failures);return failures?1:0;
}
