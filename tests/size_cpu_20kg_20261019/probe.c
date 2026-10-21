#include <stdint.h>
#include <string.h>
#include "C_Traj_Planner.c"

int probe_make(int32_t v0,int32_t a0,int32_t v1,int32_t n,int32_t vmax,
               int32_t amax,int32_t dmax,int32_t jmax,int32_t *data)
{
    TrajSeg_t g={0};s_tj.amax_cdpss=amax;s_tj.dmax_cdpss=dmax;s_tj.jmax_cdpss3=jmax;
#ifdef OPTIMIZED
    int32_t an=Traj_DivSat((int64_t)a0*n*1024,15625);
    int ret=Traj_PolyMake(&g,v0,an,v1,n,vmax);
    if(ret){g.inv_q30=Traj_DivSat(1LL<<30,n);g.acck=Traj_DivSat(1000000LL<<14,n);}
#else
    int ret=Traj_PolyMake(&g,v0,a0,v1,n,vmax);
#endif
    memcpy(data,&g,sizeof(g));return ret;
}
void probe_eval(const int32_t *data,int32_t tick,int dir,int32_t *out)
{
    TrajSeg_t g;memcpy(&g,data,sizeof(g));g.p0=13500;
    g.v0_qt*=dir;g.r_q15*=dir;g.peak_q15*=dir;g.inv_r_q15*=dir;
    Traj_PolyEval(&g,tick,out,out+1,out+2);
}
void probe_reset(int32_t v,int32_t a,int32_t d,int32_t j,int32_t tm)
{
    C_Traj_Init(13500);C_Traj_SetRange(0,27000);C_Traj_SetLimits(v,a,d,j);C_Traj_SetMoveMinMs(tm);
}
void probe_command(int32_t goal,int32_t ms,int32_t type)
{
    if(type==0)C_Traj_Plan(goal,ms);
    else if(type==1)C_Traj_PlanTimed(goal,ms);
    else if(type==2)C_Traj_Stop();
    else C_Traj_Hold(goal);
}
void probe_step(int32_t hold,int32_t *out)
{
    TrajRef_t ref={0};C_Traj_Step(&ref,hold);
    out[0]=ref.pos;out[1]=ref.vel;out[2]=ref.acc;out[3]=ref.done;
}
