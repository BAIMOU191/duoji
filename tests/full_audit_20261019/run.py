from pathlib import Path
import subprocess, json, hashlib, sys, importlib.util
ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
CC=Path(r'D:\SoftWare\mingw64\bin\gcc.exe')
PROJECTS=[ROOT/f'CH32V00X_APP_ADC_{x}kg' for x in (15,20,30)]
def execute(cmd,log):
    p=subprocess.run(list(map(str,cmd)),cwd=ROOT,capture_output=True,text=True,encoding='utf-8',errors='replace')
    log.write_text(p.stdout+p.stderr,encoding='utf-8')
    if p.returncode: raise RuntimeError(f'{log}\n{p.stdout}\n{p.stderr}')
    return p.stdout
def inventory():
    paths={}
    for p in PROJECTS:
        paths[p.name]={f.relative_to(p).as_posix():hashlib.sha256(f.read_bytes()).hexdigest()
            for d in ['Application','Common','Drivers','System','User','SRC'] for f in (p/d).rglob('*') if f.suffix in ['.c','.h','.S','.ld']}
    (OUT/'source_hashes.json').write_text(json.dumps(paths,indent=2),encoding='utf-8')
    diffs=[];semantic=[]
    for rel in sorted(set().union(*(set(x) for x in paths.values()))):
        if len({paths[p.name].get(rel) for p in PROJECTS})>1:diffs.append(rel)
        if len({(p/rel).read_text(encoding='utf-8') if (p/rel).exists() else None for p in PROJECTS})>1:semantic.append(rel)
    (OUT/'differences.json').write_text(json.dumps(diffs,indent=2),encoding='utf-8')
    (OUT/'text_differences.json').write_text(json.dumps(semantic,indent=2),encoding='utf-8')
    print('Differing source files:',diffs,'; after newline normalization:',semantic,flush=True)
def host():
    for p in PROJECTS:
        dst=OUT/p.name;dst.mkdir(exist_ok=True)
        inc=['-I'+str(ROOT/'tests/stubs')]+['-I'+str(p/d) for d in ['Application','Common','Drivers','System','SRC/Debug']]
        exe=dst/'audit.exe'
        execute([CC,'-std=gnu99','-O2','-Wall','-Wextra','-Wl,--image-base,0x400000',*inc,OUT/'audit.c','-lm','-o',exe],dst/'compile.log')
        result=execute([exe,dst/'trace.csv'],dst/'results.txt');print(p.name+'\n'+result,flush=True)
        legacy=dst/'legacy_parameters.exe'
        execute([CC,'-std=gnu99','-O2','-Wall','-Wextra',*inc,OUT/'legacy_wrapper.c','-lm','-o',legacy],dst/'legacy_compile.log')
        print(execute([legacy],dst/'legacy_results.txt'),flush=True)
def target():
    spec=importlib.util.spec_from_file_location('existing_build',ROOT/'tests/verify_alignment.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);module.OUT=OUT/'target_builds'
    for p in PROJECTS: module.build(p)
if __name__=='__main__':
    inventory()
    if '--target' in sys.argv:target()
    else:host()
