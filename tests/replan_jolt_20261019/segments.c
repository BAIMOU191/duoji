/* Simulation-only event-driven three-segment C2 reference. */
#undef C_Traj_Plan
#undef C_Traj_Step
#undef C_Traj_Stop
#undef C_Traj_Hold
#undef C_Traj_Is_Done
#include <math.h>
static struct { double v[2][4],x0,x1,x2,target; int n1,nc,n3,t,T,active; } qplan;
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

static void velocity_coeff(double *c,double v0,double a0,double v1,double t)
{c[0]=v0;c[1]=a0*t;c[2]=3*(v1-v0)-2*a0*t;c[3]=2*(v0-v1)+a0*t;}
static double segment_pos(const double *v,double seconds,double u)
{return seconds*(v[0]*u+v[1]*u*u/2+v[2]*u*u*u/3+v[3]*u*u*u*u/4);}
static int segment_valid(const double *v,double x,double T,double vmax,int direction)
{
    double p[5]={x},a[3],j[2];
    for(int i=0;i<4;i++)p[i+1]=v[i]*T/(i+1);
    for(int i=1;i<4;i++)a[i-1]=i*v[i]/T;
    for(int i=1;i<3;i++)j[i-1]=i*a[i]/T;
    double am=fmin(s_tj.amax_cdpss,s_tj.dmax_cdpss);
    return within(p,4,s_tj.lo,s_tj.hi)&&within(v,3,direction>0?0:-vmax,direction<0?0:vmax)
        &&within(a,2,-am,am)&&within(j,1,-s_tj.jmax_cdpss3*.995,s_tj.jmax_cdpss3*.995);
}
static int make_segments(double x,double v,double a,double target,int N,double fraction)
{
    int n1=(int)ceil(N*fraction),n3=n1,nc=N-n1-n3;
    if(n1<2||nc<0)return 0;
    double T=N*.001,t1=n1*.001,t3=n3*.001;
    double vc=(target-x-v*t1/2-a*t1*t1/12)/(T-(t1+t3)/2);
    int dir=(target>x)?1:-1;
    if(vc*dir<0)return 0;
    velocity_coeff(qplan.v[0],v,a,vc,t1);velocity_coeff(qplan.v[1],vc,0,0,t3);
    double x1=x+segment_pos(qplan.v[0],t1,1),x2=x1+vc*nc*.001;
    double vmax=fmax(s_tj.vmax_cdps,fabs(v))*1.000001;
    if(!segment_valid(qplan.v[0],x,t1,vmax,v*dir>=0?dir:0)
       ||!segment_valid(qplan.v[1],x2,t3,vmax,dir))return 0;
    if(x1<s_tj.lo||x1>s_tj.hi||x2<s_tj.lo||x2>s_tj.hi)return 0;
    qplan.x0=x;qplan.x1=x1;qplan.x2=x2;qplan.target=target;
    qplan.n1=n1;qplan.nc=nc;qplan.n3=n3;qplan.T=N;qplan.t=0;qplan.active=1;return 1;
}
void C_Traj_Plan(int32_t target,uint16_t ms)
{
    if(s_tj.vel_qt==0&&s_tj.acc_now==0){qplan.active=0;Legacy_Plan(target,ms);return;}
    double x=s_tj.pos,v=s_tj.vel_qt*1000./65536,a=s_tj.acc_now;
    __typeof__(s_tj) saved=s_tj;Legacy_Plan(target,ms);
    int N=s_tj.T+s_tj.pre_T;s_tj=saved;if(N<6)N=6;
    double fractions[]={.35,.30,.25,.20,.15,.10,.05};
    for(int i=0;i<40;i++){
        for(unsigned k=0;k<7;k++)if(make_segments(x,v,a,target,N,fractions[k]))return;
        N=(int)ceil(N*1.1)+1;
    }
    q_fallback++;qplan.active=0;Legacy_Plan(target,ms);
}
static void eval_segment(int tick,TrajRef_t *out)
{
    const double *c;double u,T,x;
    if(tick>=qplan.T){out->pos=lround(qplan.target);out->vel=out->acc=0;out->done=1;return;}
    out->done=0;
    if(tick>qplan.n1&&tick<=qplan.n1+qplan.nc){out->pos=lround(qplan.x1+qplan.v[0][0]*0+(tick-qplan.n1)*.001*qplan.v[1][0]);out->vel=lround(qplan.v[1][0]);out->acc=0;return;}
    if(tick<=qplan.n1){c=qplan.v[0];T=qplan.n1*.001;u=tick/(double)qplan.n1;x=qplan.x0;}
    else{c=qplan.v[1];T=qplan.n3*.001;u=(tick-qplan.n1-qplan.nc)/(double)qplan.n3;x=qplan.x2;}
    out->pos=lround(x+segment_pos(c,T,u));out->vel=lround(polynomial(c,3,u));out->acc=lround((c[1]+2*c[2]*u+3*c[3]*u*u)/T);
}
void C_Traj_Step(TrajRef_t *out,uint8_t hold)
{
    if(!qplan.active){Legacy_Step(out,hold);return;}
    TrajRef_t r;eval_segment(qplan.t,&r);
    if(qplan.t<qplan.T&&(!hold||(int64_t)r.vel*r.acc<0))qplan.t++;
    eval_segment(qplan.t,out);s_tj.pos=out->pos;s_tj.vel_qt=lround(out->vel*65536./1000);s_tj.acc_now=out->acc;
}
uint8_t C_Traj_Is_Done(void){return qplan.active?qplan.t>=qplan.T:Legacy_Done();}
void C_Traj_Hold(int32_t x){qplan.active=0;Legacy_Hold(x);}
void C_Traj_Stop(void){qplan.active=0;Legacy_Stop();}
