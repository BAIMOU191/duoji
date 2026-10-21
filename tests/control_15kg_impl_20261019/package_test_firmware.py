"""Refresh only 15kg's normal firmware artifacts, preserving previous artifacts first."""
from pathlib import Path
import hashlib
import importlib.util
import json
import shutil
import subprocess
import difflib
import verify_impl as check


def package():
    assert json.loads((check.OUT/'target_status.json').read_text())['passed']
    assert '#define APP_MODE_CALIB\n' not in (check.PROJECT/'User/main.c').read_text(encoding='utf-8').replace('// #define APP_MODE_CALIB','')
    spec=importlib.util.spec_from_file_location('wch_paths',check.ROOT/'tests/verify_alignment.py')
    build=importlib.util.module_from_spec(spec);spec.loader.exec_module(build)
    src=check.OUT/'target'/check.PROJECT.name
    dest=check.PROJECT/'obj'
    dest.mkdir(exist_ok=True)
    previous=check.HERE/'before/obj';previous.mkdir(exist_ok=True)
    for ext in ('elf','hex','bin','map','lst'):
        old=dest/('CH32V00X_APP.'+ext)
        if old.exists() and not (previous/old.name).exists():shutil.copy2(old,previous/old.name)
    for ext in ('elf','hex','bin','map'):
        shutil.copy2(src/('CH32V00X_normal.'+ext),dest/('CH32V00X_APP.'+ext))
    with (dest/'CH32V00X_APP.lst').open('w',encoding='utf-8') as file:
        result=subprocess.run([str(build.WCH/'riscv-wch-elf-objdump.exe'),'-S','-d',str(dest/'CH32V00X_APP.elf')],stdout=file,stderr=subprocess.PIPE,text=True)
        assert result.returncode==0,result.stderr
    hashes={file.name:hashlib.sha256(file.read_bytes()).hexdigest() for file in dest.glob('CH32V00X_APP.*')}
    (check.OUT/'firmware_hashes.json').write_text(json.dumps(hashes,indent=2),encoding='utf-8')
    start=json.loads((check.HERE/'source_hashes_before.json').read_text())
    after={name:hashlib.sha256((check.ROOT/name).read_bytes()).hexdigest() for name in start}
    (check.OUT/'source_hashes_after.json').write_text(json.dumps(after,indent=2),encoding='utf-8')
    diff=''
    for file in (check.HERE/'before').rglob('*'):
        if file.is_file() and 'obj' not in file.relative_to(check.HERE/'before').parts:
            rel=file.relative_to(check.HERE/'before')
            diff+=''.join(difflib.unified_diff(file.read_text(encoding='utf-8').splitlines(True),(check.PROJECT/rel).read_text(encoding='utf-8').splitlines(True),fromfile='before/'+rel.as_posix(),tofile='after/'+rel.as_posix()))
    (check.HERE/'implementation.diff').write_text(diff,encoding='utf-8')
    print('Normal firmware ready:',dest/'CH32V00X_APP.bin',flush=True)
    print('BIN bytes:',(dest/'CH32V00X_APP.bin').stat().st_size,'SHA256:',hashes['CH32V00X_APP.bin'],flush=True)


if __name__=='__main__':
    check.verify_scope();package()
