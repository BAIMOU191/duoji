/* Simulation-only double precision C2 quintic. Not intended as MCU firmware. */
#undef C_Traj_Plan
#undef C_Traj_Step
#undef C_Traj_Stop
#undef C_Traj_Hold
#undef C_Traj_Is_Done
#include <math.h>
static struct { double c[6],seconds; int t,T,active; } qplan;
static unsigned q_fallback;
static double polynomial(const double *c,int n,double t)
{ double x=c[n];while(n--)x=x*t+c[n];return x; }
/* Isolate roots between derivative roots: all extrema, no sampling blind spots. */
static int roots01(const double *c,int n,double *out)
{
    if(n<=0)return 0;
    if(n==1){double x=-c[0]/c[1];if(isfinite(x)&&x>0&&x<1){out[0]=x;return 1;}return 0;}
    double d[6],crit[6],cuts[8];for(int i=1;i<=n;i++)d[i-1]=i*c[i];
    int count=roots01(d,n-1,crit),nr=0;cuts[0]=0;
    for(int i=0;i<count;i++)cuts[i+1]=crit[i];cuts[count+1]=1;
    for(int i=0;i<=count;i++){
        double lo=cuts[i],hi=cuts[i+1],a=polynomial(c,n,lo),b=polynomial(c,n,hi);
        if(i && fabs(a)<1e-8)out[nr++]=lo;
        if(a*b<0){for(int k=0;k<45;k++){double mid=(lo+hi)*.5,v=polynomial(c,n,mid);if(a*v<=0)hi=mid;else{lo=mid;a=v;}}out[nr++]=(lo+hi)*.5;}
    }
    return nr;
}
static int within(const double *c,int n,double lo,double hi)
{
    double d[6],r[6];for(int i=1;i<=n;i++)d[i-1]=i*c[i];int nr=roots01(d,n-1,r);
    double a=polynomial(c,n,0),b=polynomial(c,n,1);
    if(a<lo-1e-6||a>hi+1e-6||b<lo-1e-6||b>hi+1e-6)return 0;
    for(int i=0;i<nr;i++){a=polynomial(c,n,r[i]);if(a<lo-1e-6||a>hi+1e-6)return 0;}
    return 1;
}
static int q_fit(double x,double v,double a,double target,double T)
{
    double D=target-x,V=v*T,A=a*T*T;
    double *c=qplan.c;
    c[0]=x;c[1]=V;c[2]=A*.5;c[3]=10*D-6*V-1.5*A;c[4]=-15*D+8*V+1.5*A;c[5]=6*D-3*V-.5*A;
    double cv[5],ca[4],cj[3];
    for(int i=1;i<=5;i++)cv[i-1]=i*c[i]/T;
    for(int i=1;i<=4;i++)ca[i-1]=i*cv[i]/T;
    for(int i=1;i<=3;i++)cj[i-1]=i*ca[i]/T;
    double acceleration=fmin(s_tj.amax_cdpss,s_tj.dmax_cdpss);
    double vmax=fmax(s_tj.vmax_cdps,fabs(v))*1.000001;
    /* 0.5% jerk margin for integer rounding of the 1ms reference. */
    return within(c,5,s_tj.lo,s_tj.hi)&&within(cv,4,-vmax,vmax)
        &&within(ca,3,-fmax(acceleration,fabs(a)),fmax(acceleration,fabs(a)))
        &&within(cj,2,-s_tj.jmax_cdpss3*.995,s_tj.jmax_cdpss3*.995);
}
void C_Traj_Plan(int32_t target,uint16_t ms)
{
    double x=s_tj.pos,v=s_tj.vel_qt*1000./65536,a=s_tj.acc_now;
    /* Keep the original standstill move. Only compare replan handoff behavior. */
    if(s_tj.vel_qt==0 && s_tj.acc_now==0){qplan.active=0;Legacy_Plan(target,ms);return;}
    __typeof__(s_tj) saved=s_tj;
    Legacy_Plan(target,ms);
    int T=s_tj.T+s_tj.pre_T;
    s_tj=saved;
    if(T<2)T=2;
    target=Fx_ClampRange(target,s_tj.lo,s_tj.hi);
    for(int k=0;k<40;k++){
        if(q_fit(x,v,a,target,T*.001)){
            qplan.T=T;qplan.t=0;qplan.seconds=T*.001;qplan.active=1;return;
        }
        T=(int)ceil(T*1.10)+1;
    }
    q_fallback++;qplan.active=0;Legacy_Plan(target,ms);
}
void C_Traj_Step(TrajRef_t *out,uint8_t hold)
{
    if(!qplan.active){Legacy_Step(out,hold);return;}
    double at=qplan.t/(double)qplan.T,cv[5],ca[4];
    for(int i=1;i<=5;i++)cv[i-1]=i*qplan.c[i]/qplan.seconds;
    for(int i=1;i<=4;i++)ca[i-1]=i*cv[i]/qplan.seconds;
    double v=polynomial(cv,4,at),a=polynomial(ca,3,at);
    if(qplan.t<qplan.T && (!hold || v*a<0))qplan.t++;
    at=qplan.t/(double)qplan.T;
    out->pos=(int32_t)lround(polynomial(qplan.c,5,at));
    out->vel=qplan.t>=qplan.T?0:(int32_t)lround(polynomial(cv,4,at));
    out->acc=qplan.t>=qplan.T?0:(int32_t)lround(polynomial(ca,3,at));
    out->done=qplan.t>=qplan.T;
    s_tj.pos=out->pos;s_tj.vel_qt=(int32_t)lround(out->vel*65536./1000);s_tj.acc_now=out->acc;
}
uint8_t C_Traj_Is_Done(void){return qplan.active?(qplan.t>=qplan.T):Legacy_Done();}
void C_Traj_Hold(int32_t pos){qplan.active=0;Legacy_Hold(pos);}
void C_Traj_Stop(void){qplan.active=0;Legacy_Stop();}
