"""Compile the production input path and reproduce idle twitch triggers."""
import argparse, json, pathlib, subprocess
HERE=pathlib.Path(__file__).resolve().parent
ROOT=HERE.parent

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--label',default='fixed');args=parser.parse_args()
    out=HERE/'results/pwm_idle';out.mkdir(exist_ok=True,parents=True)
    exe=out/'pwm_idle.exe'
    sources=[HERE/'audit/pwm_idle_audit.c',ROOT/'Application/A_Sensor.c']+[ROOT/'Common'/f'{n}.c' for n in ['C_Pos_Ctrl','C_Speed_Observer','C_Traj_Planner']]
    inc=['-I'+str(p) for p in [HERE/'sim/stubs',ROOT/'Application',ROOT/'Common',ROOT/'Drivers']]
    subprocess.run(['gcc','-std=c99','-O2',*inc,*map(str,sources),'-lm','-o',str(exe)],check=True)
    cases=[('constant',0),('spike1',100),('spike2',100),('spike2',-100),
           ('plateau',6),('plateau',8),('random',3),('random',6),('random',8),
           ('step',5),('step',10),('step',100),('step',-100),('ramp',0)]
    results=[]
    for mode,amp in cases:
        r=json.loads(subprocess.check_output([str(exe),mode,str(amp)],text=True));results.append(r);print(r)
    (out/(args.label+'.json')).write_text(json.dumps(results,indent=2),encoding='utf-8')

if __name__=='__main__':main()
