import pathlib,subprocess,json,ctypes
HERE=pathlib.Path(__file__).resolve().parent;ROOT=HERE.parents[1];OUT=HERE/'results'
ctypes.windll.kernel32.SetErrorMode(3)
report={}
for version,source in [('before',HERE/'baseline'),('after',ROOT)]:
    report[version]={}
    flags=['gcc','-std=c99','-O1','-g','-Wall','-Wextra']
    if version=='after':flags+=['-fsanitize=undefined','-fsanitize-undefined-trap-on-error']
    for test in ['i2c','temperature']:
        exe=OUT/f'{test}_{version}.exe'
        inc=[HERE/'stubs',ROOT/'tests/sim/stubs',source/'Drivers',source/'Application',ROOT/'Common']
        inputs=[HERE/f'{test}_test.c']
        if test=='i2c':
            inc.insert(0,HERE/'stubs/i2c')
            text=(source/'Drivers/D_i2c.c').read_text(encoding='utf-8')
            assert text.count('dummy = I2Cx->STAR2;')==1
            # Hook only the volatile hardware read to model clearing ADDR; preserve driver control flow.
            text=text.replace('dummy = I2Cx->STAR2;','dummy = test_read_star2(I2Cx);')
            generated=OUT/f'i2c_{version}.c';generated.write_text(text,encoding='utf-8');inputs.append(generated)
        subprocess.run(flags+['-I'+str(p) for p in inc]+list(map(str,inputs))+['-o',str(exe)],check=True)
        r=subprocess.run([str(exe)],capture_output=True,text=True,timeout=20)
        report[version][test]={'returncode':r.returncode,'output':r.stdout+r.stderr}
        if version=='after':assert r.returncode==0,report[version][test]
(OUT/'verification.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
