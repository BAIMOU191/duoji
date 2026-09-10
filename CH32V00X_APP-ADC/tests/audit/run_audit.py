import pathlib,subprocess,json,sys,ctypes
HERE=pathlib.Path(__file__).resolve().parent;ROOT=HERE.parents[1];OUT=HERE/'results'
ctypes.windll.kernel32.SetErrorMode(0x0001|0x0002) # No crash dialog; failures return to the test runner.
def compile_host(name,source,main,sanitize=False,extra=()):
    exe=OUT/f'{name}.exe'
    args=['gcc','-std=c99','-O1' if sanitize else '-O2','-g','-Wall','-Wextra','-Wno-unused-parameter']
    if sanitize:args+=['-fsanitize=undefined','-fsanitize-undefined-trap-on-error']
    args+=['-I'+str(ROOT/'tests/sim/stubs'),'-I'+str(source/'Application'),'-I'+str(source/'Common'),'-I'+str(ROOT/'Drivers'),'-I'+str(ROOT/'Application'),'-I'+str(ROOT/'Common')]
    args+=[str(main),str(source/'Application/A_Sensor.c')]
    args+=[str(source/'Common'/f'{n}.c') for n in ['C_Pos_Ctrl','C_Speed_Observer','C_Traj_Planner']]
    subprocess.run(args+list(extra)+['-lm','-o',str(exe)],check=True)
    return exe
def compile_standalone(name,main,extra=()):
    # A_Protect.c alone, with its own stubs: A_Sensor.c would collide with them.
    exe=OUT/f'{name}.exe'
    args=['gcc','-std=c99','-O2','-g','-Wall','-Wextra','-Wno-unused-parameter']
    args+=['-I'+str(ROOT/'tests/sim/stubs'),'-I'+str(ROOT/'Application'),'-I'+str(ROOT/'Common'),'-I'+str(ROOT/'Drivers')]
    subprocess.run(args+[str(main)]+list(extra)+['-lm','-o',str(exe)],check=True)
    return exe
if __name__=='__main__':
    report={}
    for v in (sys.argv[1:] or ['baseline','fixed']):
        source=HERE/'baseline' if v=='baseline' else ROOT
        exe=compile_host(v,source,HERE/'servo_audit.c')
        report[v]={}
        for scenario in ['stop','release','pause','range','save','protection','invalid_range']:
            r=subprocess.run([str(exe),scenario],capture_output=True,text=True,timeout=20)
            report[v][scenario]={'returncode':r.returncode,'output':r.stdout+r.stderr}
    # Protection always runs against the current tree: the baseline snapshot
    # predates the stall chain, so there is nothing to compare it against.
    exe=compile_standalone('protect',HERE/'protect_audit.c')
    r=subprocess.run([str(exe)],capture_output=True,text=True,timeout=20)
    report.setdefault('fixed',{})['protect_unit']={'returncode':r.returncode,'output':r.stdout+r.stderr}
    exe=compile_standalone('stall',HERE/'stall_audit.c',
        [str(ROOT/'Application/A_Sensor.c')]+[str(ROOT/'Common'/f'{n}.c') for n in ['C_Pos_Ctrl','C_Speed_Observer','C_Traj_Planner']])
    for scenario in ['stall','free','slow','recover','hold_load','jam','released']:
        r=subprocess.run([str(exe),scenario],capture_output=True,text=True,timeout=30)
        report['fixed']['stall_'+scenario]={'returncode':r.returncode,'output':r.stdout+r.stderr}
    (OUT/'state_cases.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    if any(r['returncode'] for v, cases in report.items() if v!='baseline' for r in cases.values()):
        sys.exit(1)
