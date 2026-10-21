from pathlib import Path
import importlib.util,subprocess,sys,json
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=HERE/'output'
spec=importlib.util.spec_from_file_location('s',ROOT/'tests/sync_noncontrol_adc_20261019/verify.py')
s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s);s.OUT=OUT
NAMES=('C_Traj_Init','C_Traj_SetRange','C_Traj_SetRangeOpen','C_Traj_SetLimits','C_Traj_SetMoveMinMs','C_Traj_GetPos','C_Traj_Hold','C_Traj_Plan','C_Traj_PlanTimed','C_Traj_Stop','C_Traj_Step','C_Traj_Is_Done','probe_make','probe_eval','probe_reset','probe_command','probe_step')
def main():
    (OUT/'20').mkdir(parents=True,exist_ok=True)
    objects=[]
    for prefix,project in [('B',HERE/'before/CH32V00X_APP_ADC_20kg'),('A',HERE/'candidate')]:
        obj=OUT/(prefix+'.o');objects.append(obj)
        args=[s.CC,'-std=gnu99','-O2','-ftrapv','-Wall','-Wextra','-c',HERE/'probe.c','-I'+str(project/'Common'),'-o',obj]
        args+=['-D'+n+'='+prefix+'_'+n for n in NAMES]
        if prefix=='A':args+=['-DOPTIMIZED']
        s.run(args,OUT/(prefix+'_compile.log'))
    exe=OUT/'equivalence.exe'
    s.run([s.CC,'-O2','-ftrapv',HERE/'equivalence.c',*objects,'-o',exe],OUT/'equivalence_compile.log')
    print(s.run([exe],OUT/'equivalence.log'),flush=True)
if __name__=='__main__':main()
