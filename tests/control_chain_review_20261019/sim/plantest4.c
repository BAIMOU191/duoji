#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "C_Traj_Planner.c"
int main(void){
  TrajRef_t r; FILE *f=fopen("fig/f4.csv","w"); fprintf(f,"t,cmd,ref\n");
  C_Traj_Init(13500); C_Traj_SetRange(0,27000);
  C_Traj_SetLimits(14500,525000,525000,10500000); C_Traj_SetMoveMinMs(150);
  for(int t=0;t<4000;t++){ double c=13500+3000*sin(6.283185307*1.0*t/1000.0);
    if(t%20==0) C_Traj_Plan((int32_t)lround(c),20); C_Traj_Step(&r,0); fprintf(f,"%d,%.1f,%d\n",t,c,r.pos);}
  fclose(f); return 0; }
