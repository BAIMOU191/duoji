#define main old_sim_main
#include "../control_optimization_20261019/source/sim.c"
#undef main

#ifndef C2_TEST
#define q_fallback 0U
#endif
static int Tim0,Tim1,change_ms,delta,period,stream_mode;
static void scenario(int t)
{
    if(stream_mode){
        if(t<=1800 && t%period==0)cmd(stream_mode==2?18000:14500+t*3,Tim1);
    }else{
        if(t==0)cmd(18000,Tim0);
        if(t==change_ms)cmd(18000+delta,Tim1);
    }
}
static void one(int load,int T0,int T1,int tm,int d,int per,FILE *csv)
{
    Plant pl=base_plant();pl.r=load==0?1:4;
    if(load==2){pl.bl=30;pl.res_hz=40;}
    Tim0=T0;Tim1=T1;change_ms=tm;delta=d;period=abs(per);stream_mode=per<0?2:per>0?1:0;
    load_fn=0;g_kind=0;sim_reset(&pl,0,0);
    double before_v=0,min_v=1e9,ref_jump=0,maxdu=0,maxjerk=0,lastacc=0,lastu=0,lastv=0;
    double amin=1e9,amax=-1e9,acc20[21]={0},jerk20=0,vmin=1e9,vmax=-1e9,vsum=0,vsq=0,lag=0;
    int moving=0,arr=-1,settle=-1,win=0,lastcmd=per?1800/abs(per)*abs(per):tm;
    int faults=0,lastfault=0,hard=0,events=0,dips=0,approach=0,back_ticks=0;
    double over=0;
    unsigned fallback=q_fallback;
    int duration=lastcmd+(T1>1000?T1:1000)+2200;
    for(int t=0;t<duration;t++){
        if(!per && t==tm){before_v=vl;moving=!s_servo.ref.done && fabs(vl)>500;}
        scenario(t);sim_tick();
        if(t==lastcmd)approach=sgn(s_servo.target_angle-xl);
        if(t>=lastcmd){double ex=(xl-s_servo.target_angle)*approach;if(ex>over)over=ex;if(vl*approach < -100)back_ticks++;}
        if(t>=lastcmd && s_servo.arrived && arr<0 && t>lastcmd+3)arr=t-lastcmd;
        if(t>=lastcmd){if(fabs(xl-s_servo.target_angle)<=50){if(!win){settle=t-lastcmd;win=1;}}else{win=0;settle=-1;}}
        int event=per?(t>0&&t<=1800&&t%per==0):t==tm;
        if(event)events++;
        int measure=per?(t>=400&&t<1700):(t>=tm&&t<tm+80);
        double a=(vl-lastv)*10; /* degree/s^2 from cdeg/s per ms */
        acc20[t%21]=vl;
        if(measure){
            if(fabs(s_servo.ref.acc-lastacc)>ref_jump)ref_jump=fabs(s_servo.ref.acc-lastacc);
            if(fabs(s_servo.dbg.u_out-lastu)>maxdu)maxdu=fabs(s_servo.dbg.u_out-lastu);
            double j=fabs(a-(lastv-acc20[(t+19)%21])*10)*1000;
            if(j>maxjerk)maxjerk=j;
            if(vl<min_v)min_v=vl;
            if(a<amin)amin=a;if(a>amax)amax=a;
            if(vl<vmin)vmin=vl;if(vl>vmax)vmax=vl;vsum+=vl;vsq+=vl*vl;
            if(per && fabs(14500+t*3-xl)>lag)lag=fabs(14500+t*3-xl);
            if(event && vl<500)dips++;
        }
        if(s_prot.fault && !lastfault)faults++;lastfault=s_prot.fault;
        if(s_servo.debug_control_valid && abs(s_servo.dbg.u_out)>audit_used_limit)hard++;
        if(csv)fprintf(csv,"%d,%.7f,%.7f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",t,xl/100,vl/100,s_servo.target_angle,s_servo.ref.pos,s_servo.ref.vel,s_servo.ref.acc,s_servo.dbg.u_out,s_servo.dbg.u_fb,s_servo.dbg.u_i,s_servo.dbg.u_stick,s_servo.dbg.state,s_servo.ref.done,s_servo.arrived,s_ctrl.hold_blend_ms,s_servo.obs.vel);
        lastacc=s_servo.ref.acc;lastu=s_servo.dbg.u_out;lastv=vl;
    }
    int n=per?1300:80;double mean=vsum/n,sd=sqrt(fmax(0,vsq/n-mean*mean));
    printf("CASE,load=%d,T0=%d,T1=%d,at=%d,delta=%d,period=%d,moving=%d,vbefore=%.4f,vmin80=%.4f,dv_ref_acc=%.4f,du=%.1f,amin=%.3f,amax=%.3f,arr=%d,settle=%d,final=%.5f,fault=%d,hard=%d,meanv=%.4f,sdv=%.4f,lag=%.4f,fallback=%u,over=%.4f,back_ms=%d\n",
        load,T0,T1,tm,d,per,moving,before_v/100,min_v/100,ref_jump/100,maxdu,amin,amax,arr,settle,(xl-s_servo.target_angle)/100,faults,hard,mean/100,sd/100,lag/100,q_fallback-fallback,over/100,back_ticks);
}
int main(int argc,char **argv)
{
    if(argc>1 && !strcmp(argv[1],"trace")){
        assert(argc==9);FILE *f=fopen(argv[8],"w");assert(f);
        fprintf(f,"t,x,v,target,ref,refv,refa,u,ufb,ui,stick,state,done,arrived,blend,obs\n");one(atoi(argv[2]),atoi(argv[3]),atoi(argv[4]),atoi(argv[5]),atoi(argv[6]),atoi(argv[7]),f);fclose(f);return 0;
    }
    int T[][2]={{0,0},{200,200},{1000,1000},{3000,3000},{0,1000},{1000,0}};
    int when[]={50,150,300,700},ds[]={-9000,-3000,-500,500,3000,6000};
    for(int load=0;load<3;load++){
        for(unsigned i=0;i<6;i++)for(unsigned a=0;a<4;a++)for(unsigned d=0;d<6;d++)one(load,T[i][0],T[i][1],when[a],ds[d],0,0);
        for(int ti=0;ti<3;ti++)for(int per=5;per<=200;per=per==5?10:per==10?20:per==20?50:per==50?100:200){one(load,0,ti==0?0:ti==1?1000:3000,0,0,per,0);if(per==200)break;}
    }
    return 0;
}
