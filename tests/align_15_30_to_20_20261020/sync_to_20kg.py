"""Sync 15kg/30kg/20kg_single app code to the 20kg baseline, keeping only model values.

Profile = hardware identity + motor calibration (FW_TORQUE and the MOTOR_* block of
A_Config.h). Everything else -- all sources, project file lists, even newline style --
is copied byte-for-byte from the 20kg baseline.
"""
from pathlib import Path
import hashlib
import json
import re
import shutil
import sys

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'CH32V00X_APP_ADC_20kg'
VARIANTS = ['CH32V00X_APP_ADC_15kg', 'CH32V00X_APP_ADC_30kg',
            'CH32V00X_APP_ADC_20kg_single']
HERE = Path(__file__).parent
OUT = HERE / 'output'
SNAP = HERE / 'backup_before_align'

SOURCE_DIRS = ['Application', 'Common', 'Drivers', 'System', 'User']
PROFILE_FIELDS = [
    'FW_TORQUE',
    'MOTOR_CAL_MV', 'MOTOR_KV_UV', 'MOTOR_KA_NV', 'MOTOR_FC_MV', 'MOTOR_FS_MV',
    'MOTOR_DELAY_US', 'MOTOR_NOISE_CDEG', 'MOTOR_MIN_STEP_CDEG', 'MOTOR_SIGN',
    'MOTOR_RATED_VEL_DPS', 'MOTOR_RATED_ACC_DPS2', 'MOTOR_RATED_DEC_DPS2',
]
# generated makefiles that name source objects; they carry the absolute project
# path in obj/makefile and must track the baseline source list exactly.
PROJECT_FILES = [f'obj/{d}/subdir.mk' for d in
                 ['Application', 'Common', 'Drivers', 'System', 'User', 'SRC/Core',
                  'SRC/Debug', 'SRC/Peripheral/src', 'SRC/Startup']] + ['obj/makefile']
# neutral IDE/project descriptors: byte-identical to the baseline (they carry the
# shared -flto settings, which the baseline uses and stale variants did not).
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
    # model name in FW_TORQUE's comment follows the model value in the template
    out = re.sub(r'^(#define FW_TORQUE\s+\S+\s+/\*\*.*?扭力型号 )\d+kg',
                 lambda m: m[1] + '20kg', out, flags=re.M)
    out = re.sub(r'^(#define FW_TORQUE\s+\S+\s+/\*\*.*?扭力型号 )\d+kg',
                 lambda m: m[1] + values['FW_TORQUE'].rstrip('U') + 'kg', out, flags=re.M)
    for name in PROFILE_FIELDS:
        out, n = re.subn(r'^(#define ' + name + r'\s+)\S+', lambda m: m[1] + values[name],
                         out, flags=re.M)
        assert n == 1, name
    return out


def snapshot():
    """Record the pre-alignment baseline once; back up every variant exactly once."""
    hash_file = HERE / 'source_hashes_before.json'
    if hash_file.exists():
        print(f'kept existing {hash_file.name} (pre-alignment baseline)')
    else:
        files = {}
        for p in ROOT.glob('CH32V00X_APP*'):
            if not p.is_dir():
                continue
            for d in SOURCE_DIRS + ['obj']:
                base = p / d
                if not base.exists():
                    continue
                for f in base.rglob('*'):
                    if f.is_file():
                        files[f.relative_to(ROOT).as_posix()] = digest(f)
        hash_file.write_text(json.dumps(files, indent=2), encoding='utf-8')
        print(f'snapshotted {len(files)} source/obj files -> {hash_file}')
    for v in VARIANTS:
        # never refresh an existing backup: it must keep the pre-alignment state
        if (SNAP / v / 'Application' / 'A_Config.h').exists():
            print(f'backup already present for {v}, left untouched: {SNAP / v}')
            continue
        for d in SOURCE_DIRS + ['obj']:
            src = ROOT / v / d
            dst = SNAP / v / d
            if src.is_dir():
                shutil.copytree(src, dst, dirs_exist_ok=True)
        for src in (ROOT / v).iterdir():
            if src.is_file() and not (SNAP / v / src.name).exists():
                shutil.copy2(src, SNAP / v / src.name)
        print(f'backed up {v} -> {SNAP / v}')


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
            # only the makefile carries the baseline variant name (absolute paths);
            # .cproject/.wvproj/.launch are variant-neutral and must stay byte-identical
            text = read(src).replace(BASE.name, v) if rel == 'obj/makefile' else read(src)
            same = (read(dst) == text) if rel == 'obj/makefile' \
                else (dst.exists() and dst.read_bytes() == src.read_bytes())
            if not same:
                changed.append(rel)
                if not dry:
                    if rel == 'obj/makefile':
                        dst.write_text(text, encoding='utf-8', newline='\n')
                    else:
                        shutil.copy2(src, dst)   # keep the baseline's byte-exact line endings
        # neutral IDE/project descriptors must stay byte-identical to the baseline
        for rel in NEUTRAL_FILES:
            src, dst = BASE / rel, dst_root / rel
            if src.exists() and (not dst.exists() or dst.read_bytes() != src.read_bytes()):
                changed.append(rel)
                if not dry:
                    shutil.copy2(src, dst)
        summary[v] = changed
    # merge with the record of earlier runs so no per-variant history is lost
    hist_file = OUT / 'changed_sources.json'
    history = json.loads(hist_file.read_text(encoding='utf-8')) if hist_file.exists() else {}
    for v, items in summary.items():
        history[v] = sorted(set(history.get(v, [])) | set(items))
    hist_file.write_text(json.dumps(history, indent=2), encoding='utf-8')
    for v, items in summary.items():
        print(f'== {v}: {len(items)} files touched this run')
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
            if rel == 'obj/makefile':
                same = read(dst) == read(src).replace(BASE.name, v)
            elif rel in NEUTRAL_FILES:
                same = dst.read_bytes() == src.read_bytes()
            else:
                same = read(dst) == read(src)
            if not same:
                ok = False
                print(f'FAIL project file {v}/{rel}')
        # profile values unchanged from the pre-sync variant snapshot
        old = profile_values(read(SNAP / v / 'Application/A_Config.h'))
        if old != values:
            ok = False
            print(f'FAIL profile values drifted for {v}: {old} != {values}')
    print('PASS: 15kg/30kg/20kg_single source text matches 20kg exactly except '
          'A_Config.h profile fields' if ok else 'VERIFY FAILED')
    return ok


if __name__ == '__main__':
    OUT.mkdir(parents=True, exist_ok=True)
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'all'
    if cmd in ('snapshot', 'all'):
        snapshot()
    if cmd in ('sync', 'all'):
        sync()
    if cmd in ('verify', 'all'):
        sys.exit(0 if verify() else 1)
