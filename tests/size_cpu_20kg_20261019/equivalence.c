#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#define API(p) int p##_probe_make(int32_t,int32_t,int32_t,int32_t,int32_t,int32_t,int32_t,int32_t,int32_t*); \
 void p##_probe_eval(const int32_t*,int32_t,int,int32_t*); \
 void p##_probe_reset(int32_t,int32_t,int32_t,int32_t,int32_t); \
 void p##_probe_command(int32_t,int32_t,int32_t);void p##_probe_step(int32_t,int32_t*)
API(B);API(A);
static uint32_t rng=987654321U;
static uint32_t random_u(void){rng=rng*1664525U+1013904223U;return rng;}
int main(void)
{
    int accepted=0;long evaluations=0,steps=0;
    for(int i=0;i<1000000;i++){
        int n=8+random_u()%22931;
        if(i%3==0)n=8+random_u()%1000;
        int vmax=1+random_u()%6553600;
        int v0=(int)(random_u()%(vmax+25))-12;
        int v1=(int)(random_u()%(vmax+25))-12;
        int a0=(int)(random_u()%20000001)-10000000;
        if(i%2==0)a0=0;
        else if(i%3==0)a0=(int)(random_u()%20001)-10000;
        int amax=100+random_u()%9999901,dmax=100+random_u()%9999901,jmax=100+random_u()%199999901;
        int32_t b[9],a[9];int old=B_probe_make(v0,a0,v1,n,vmax,amax,dmax,jmax,b),now=A_probe_make(v0,a0,v1,n,vmax,amax,dmax,jmax,a);
        if(old!=now || (old && memcmp(b,a,sizeof(b)))){printf("Mismatch make i=%d v0=%d a0=%d v1=%d n=%d vmax=%d old=%d new=%d\n",i,v0,a0,v1,n,vmax,old,now);return 1;}
        if(!old)continue;
        accepted++;int times[]={0,1,n/2,n-1,n};
        for(int sign=-1;sign<=1;sign+=2)for(int k=0;k<5;k++){
            int32_t ob[3],oa[3];B_probe_eval(b,times[k],sign,ob);A_probe_eval(a,times[k],sign,oa);
            if(memcmp(ob,oa,sizeof(ob))){printf("Mismatch eval i=%d t=%d sign=%d\n",i,times[k],sign);return 2;}evaluations++;
        }
    }
    for(int scene=0;scene<300;scene++){
        int vmax=100+random_u()%99901,am=100+random_u()%9999901,dm=100+random_u()%9999901,j=100+random_u()%199999901,tm=random_u()%501;
        B_probe_reset(vmax,am,dm,j,tm);A_probe_reset(vmax,am,dm,j,tm);
        for(int t=0;t<20000;t++){
            if(t%37==0){int target=(int)(random_u()%40001)-5000,ms=random_u()%5==0?65535:random_u()%4001,type=random_u()%20;type=type<9?0:type<18?1:type==18?2:3;B_probe_command(target,ms,type);A_probe_command(target,ms,type);}
            int hold=random_u()%11==0;int32_t ob[4],oa[4];B_probe_step(hold,ob);A_probe_step(hold,oa);
            if(memcmp(ob,oa,sizeof(ob))){printf("Mismatch trajectory scene=%d t=%d\n",scene,t);return 3;}steps++;
        }
    }
    printf("PASS bit-identical: 1000000 coefficient cases, accepted=%d, signed_evaluations=%ld, public_steps=%ld\n",accepted,evaluations,steps);
    return 0;
}
