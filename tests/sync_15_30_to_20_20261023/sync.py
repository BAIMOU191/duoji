"""Sync 15kg/30kg app code to the 20kg baseline, keeping only model+calibration values.

Profile = hardware identity + motor calibration (FW_TORQUE and the MOTOR_* block of
A_Config.h). Everything else -- all sources, project file lists, even newline style --
is copied byte-for-byte from the 20kg baseline.

Scope of this run (2026-10-23): CH32V00X_APP_ADC_15kg and CH32V00X_APP_ADC_30kg only.
20kg_single / IIC / IAP are untouched.
"""
from pathlib import Path
import hashlib
import json
import re
import shutil
import sys

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'CH32V00X_APP_ADC_20kg'
VARIANTS = ['CH32V00X_APP_ADC_15kg', 'CH32V00X_APP_ADC_30kg']
HERE = Path(__file__).parent
OUT = HERE / 'output'
SNAP = HERE / 'backup'

SOURCE_DIRS = ['Application', 'Common', 'Drivers', 'System', 'User']
PROFILE_FIELDS = [
    'FW_TORQUE',
    'MOTOR_CAL_MV', 'MOTOR_KV_UV', 'MOTOR_KA_NV', 'MOTOR_FC_MV', 'MOTOR_FS_MV',
    'MOTOR_DELAY_US', 'MOTOR_NOISE_CDEG', 'MOTOR_MIN_STEP_CDEG', 'MOTOR_SIGN',
    'MOTOR_RATED_VEL_DPS', 'MOTOR_RATED_ACC_DPS2', 'MOTOR_RATED_DEC_DPS2',
]
PROJECT_FILES = [f'obj/{d}/subdir.mk' for d in
                 ['Application', 'Common', 'Drivers', 'System', 'User', 'SRC/Core',
                  'SRC/Debug', 'SRC/Peripheral/src', 'SRC/Startup']] + ['obj/makefile']
NEUTRAL_FILES = ['obj/objects.mk', 'obj/sources.mk', 'CH32V00X_APP.wvproj', '.cproject',
                 'CH32V00X_APP.launch', '.project', '.template']


def digest(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def read(p: Path) -> str:
    return p.read_text(encoding='utf-8')


def profile_values(text: str):
    vals = {}
    for name in PROFILE_FIELDS:
        m = re.search(r'^#define ' + name + r'\s+(\S+)', text, re.M)
        assert m, name
        vals[name] = m.group(1)
    return vals


def apply_profile(template: str, values: dict) -> str:
    out = template
    # model name in FW_TORQUE's comment follows the model value
    out = re.sub(r'^(#define FW_TORQUE\s+\S+\s+/\*\*.*?扭力型号 )\d+kg',
                 lambda m: m[1] + values['FW_TORQUE'].rstrip('U') + 'kg', out, flags=re.M)
    for name in PROFILE_FIELDS:
        out, n = re.subn(r'^(#define ' + name + r'\s+)\S+', lambda m: m[1] + values[name],
                         out, flags=re.M)
        assert n == 1, name
    return out


def sync(dry=False):
    summary = {}
    for v in VARIANTS:
        dst_root = ROOT / v
        values = profile_values(read(dst_root / 'Application/A_Config.h'))
        changed = []
        for d in SOURCE_DIRS:
            for src in sorted((BASE / d).glob('*')):
                if not src.is_file():
                    continue
                dst = dst_root / d / src.name
                if src.name == 'A_Config.h':
                    text = apply_profile(read(src), values)
                    raw = text.replace('\n', '\r\n') if b'\r\n' in src.read_bytes() else text
                    if not dst.exists() or dst.read_bytes() != raw.encode('utf-8'):
                        changed.append(f'{d}/{src.name}')
                        if not dry:
                            dst.write_bytes(raw.encode('utf-8'))
                    continue
                if not dst.exists() or dst.read_bytes() != src.read_bytes():
                    changed.append(f'{d}/{src.name}')
                    if not dry:
                        shutil.copy2(src, dst)
            for dst in sorted((dst_root / d).glob('*')):
                if dst.is_file() and not (BASE / d / dst.name).exists():
                    changed.append(f'REMOVED {d}/{dst.name}')
                    if not dry:
                        dst.unlink()
        for rel in PROJECT_FILES:
            src, dst = BASE / rel, dst_root / rel
            if not src.exists():
                continue
            # generated makefiles embed the absolute project path; rewrite the
            # baseline variant name to this variant everywhere it appears.
            text = read(src).replace(BASE.name, v)
            same = dst.exists() and read(dst) == text
            if not same:
                changed.append(rel)
                if not dry:
                    dst.write_text(text, encoding='utf-8', newline='\n')
        for rel in NEUTRAL_FILES:
            src, dst = BASE / rel, dst_root / rel
            if src.exists() and (not dst.exists() or dst.read_bytes() != src.read_bytes()):
                changed.append(rel)
                if not dry:
                    shutil.copy2(src, dst)
        summary[v] = changed
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'changed_sources.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    for v, items in summary.items():
        print(f'== {v}: {len(items)} files touched')
        for i in items:
            print('   ', i)
    return summary


def verify():
    ok = True
    for v in VARIANTS:
        dst_root = ROOT / v
        values = profile_values(read(dst_root / 'Application/A_Config.h'))
        for d in SOURCE_DIRS:
            base_files = {f.name for f in (BASE / d).glob('*') if f.is_file()}
            var_files = {f.name for f in (dst_root / d).glob('*') if f.is_file()}
            if base_files != var_files:
                ok = False
                print(f'FAIL file set {v}/{d}: {base_files ^ var_files}')
            for name in sorted(base_files & var_files):
                b, a = BASE / d / name, dst_root / d / name
                if name == 'A_Config.h':
                    if read(a) != apply_profile(read(b), values):
                        ok = False
                        print(f'FAIL profile merge {v}/{d}/{name}')
                elif b.read_bytes() != a.read_bytes():
                    ok = False
                    print(f'FAIL byte match {v}/{d}/{name}')
        for rel in PROJECT_FILES + NEUTRAL_FILES:
            src, dst = BASE / rel, dst_root / rel
            if not src.exists():
                continue
            if rel in PROJECT_FILES:
                same = dst.exists() and read(dst) == read(src).replace(BASE.name, v)
            else:
                same = dst.exists() and dst.read_bytes() == src.read_bytes()
            if not same:
                ok = False
                print(f'FAIL project file {v}/{rel}')
        # profile values unchanged from the pre-sync backup
        old = profile_values(read(SNAP / v / 'Application/A_Config.h'))
        if old != values:
            ok = False
            print(f'FAIL profile values drifted for {v}: {old} != {values}')
        else:
            print(f'profile preserved for {v}: {values}')
    print('PASS: 15kg/30kg source text matches 20kg exactly except '
          'A_Config.h profile fields' if ok else 'VERIFY FAILED')
    return ok


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'all'
    if cmd == 'dry':
        sync(dry=True)
    elif cmd in ('sync', 'all'):
        sync(dry=False)
    if cmd in ('verify', 'all'):
        sys.exit(0 if verify() else 1)
