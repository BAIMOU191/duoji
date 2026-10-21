"""Boundary probes and MCU builds of frozen baseline/candidate copies only."""
from pathlib import Path
import importlib.util
import contextlib
import io
import json
import re
import shutil
import run_optimization as r


def boundary(profiles=('optimized','limits')):
    source = r.HERE / 'source'
    shutil.copy2(r.HERE / 'unit.c', source / 'unit.c')
    results={}
    for profile in profiles:
        for kg in (15,20,30):
            base=source/'baseline'/f'{kg}kg'
            exe=r.OUT/f'unit_{profile}_{kg}.exe'
            include=['-I'+str(r.ROOT/'tests/stubs'),'-I'+str(source/'candidate')]
            include+=['-I'+str(base/d) for d in ('Application','Common','Drivers','System','SRC/Debug')]
            flags=[f'-D{k}={v}' for k,v in r.PROFILES[profile].items() if k.startswith('OPT_')]
            r.run([r.CC,'-std=gnu99','-O2','-Wall','-Wextra','-Wl,--image-base,0x400000',*include,*flags,source/'unit.c','-lm','-o',exe],r.OUT/'logs'/f'unit_compile_{profile}_{kg}.log')
            key=f'{profile}_{kg}'
            results[key]=r.run([exe],r.OUT/'logs'/f'unit_{key}.txt')
            print('BOUNDARY',key,results[key].strip(),flush=True)
    (r.OUT/'boundary_results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')


def target(profiles=('baseline','optimized')):
    spec=importlib.util.spec_from_file_location('wch_builder',r.ROOT/'tests/verify_alignment.py')
    build=importlib.util.module_from_spec(spec);spec.loader.exec_module(build)
    build.OUT=r.OUT/'target_builds'
    results=json.loads((r.OUT/'target_results.json').read_text()) if (r.OUT/'target_results.json').exists() else {}
    for profile in profiles:
        for kg in (15,20,30):
            stage=r.HERE/'source/target'/f'{profile}_{kg}kg'
            shutil.copytree(r.HERE/'source/baseline'/f'{kg}kg',stage,dirs_exist_ok=True)
            if profile!='baseline':
                for file in (r.HERE/'source/candidate').iterdir():
                    text=file.read_text(encoding='utf-8')
                    if file.name=='C_Control_Options.h':
                        for k,v in r.PROFILES[profile].items():
                            if k.startswith('OPT_'):
                                text=re.sub(r'(#define '+k+r' )\d+',lambda m:m[1]+str(v),text)
                    (stage/('Application' if file.name=='A_Servo.c' else 'Common')/file.name).write_text(text,encoding='utf-8')
            captured=io.StringIO()
            # A failed link is a failure, not a reason to use an old ELF.
            with contextlib.redirect_stdout(captured):
                warnings=build.build(stage)
            results[stage.name]={'passed':True,'warnings':warnings,'log':captured.getvalue()}
            (r.OUT/'logs'/f'target_{stage.name}.txt').write_text(captured.getvalue(),encoding='utf-8')
            print('TARGET PASS',stage.name,flush=True)
    (r.OUT/'target_results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')


if __name__=='__main__':
    boundary();target();r.verify()
