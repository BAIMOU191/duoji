"""Rebuild this checkout with the installed WCH toolchain (no stale IDE paths)."""
import pathlib,subprocess,shutil,hashlib,json
from run import ROOT,HERE
tool=pathlib.Path(r'D:/SoftWare/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC12/bin')
gcc=tool/'riscv-wch-elf-gcc.exe'
dest=HERE/'results'/'firmware';dest.mkdir(exist_ok=True)
flags=['-march=rv32ec_zmmul_xw','-mabi=ilp32e','-msmall-data-limit=0','-msave-restore','-Os',
       '-fsigned-char','-ffunction-sections','-fdata-sections','-fno-common','-Wall','-Wextra','-g']
inc=[ROOT/p for p in ['SRC/Core','SRC/Debug','SRC/Peripheral/inc','User','System','Drivers','Common','Application']]
sources=[]
for folder in ['Application','Common','Drivers','User','System','SRC/Core','SRC/Debug','SRC/Peripheral/src']:
    sources+=sorted((ROOT/folder).glob('*.c'))
sources+=list((ROOT/'SRC/Startup').glob('*.S'))
objects=[]
for source in sources:
    obj=dest/(source.stem+'.o');objects.append(str(obj))
    subprocess.run([str(gcc),*flags,*['-I'+str(p) for p in inc],'-c',str(source),'-o',str(obj)],check=True)
elf=dest/'CH32V00X_APP.elf'
subprocess.run([str(gcc),*flags,'-T',str(ROOT/'SRC/Ld/Link.ld'),'-nostartfiles',
               '-Wl,--gc-sections,-Map,'+str(dest/'CH32V00X_APP.map'),'--specs=nano.specs','--specs=nosys.specs',
               '-o',str(elf),*objects],check=True)
for fmt,ext in [('ihex','hex'),('binary','bin')]:
    subprocess.run([str(tool/'riscv-wch-elf-objcopy.exe'),'-O',fmt,str(elf),str(dest/f'CH32V00X_APP.{ext}')],check=True)
size=subprocess.check_output([str(tool/'riscv-wch-elf-size.exe'),str(elf)],text=True)
(dest/'CH32V00X_APP.siz').write_text(size)
print(size)
with (dest/'CH32V00X_APP.lst').open('w') as f:
    subprocess.run([str(tool/'riscv-wch-elf-objdump.exe'),'--all-headers','--demangle','--disassemble','-M','xw',str(elf)],stdout=f,check=True)
# Publish only linked artifacts. Keep the first prior build for rollback.
backup=HERE/'results'/'original_build';backup.mkdir(exist_ok=True)
for ext in ['elf','hex','bin','map','siz','lst']:
    name=f'CH32V00X_APP.{ext}';old=ROOT/'obj'/name
    if old.exists() and not (backup/name).exists():shutil.copy2(old,backup/name)
    shutil.copy2(dest/name,old)
manifest={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in dest.glob('CH32V00X_APP.*')}
(dest/'hashes.json').write_text(json.dumps(manifest,indent=2))
