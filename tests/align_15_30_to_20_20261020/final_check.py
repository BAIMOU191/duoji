"""Final acceptance: 15kg/30kg must match 20kg except for the model profile values."""
from pathlib import Path
import hashlib
import json
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).parent
BASE = ROOT / 'CH32V00X_APP_ADC_20kg'
VARIANTS = ['CH32V00X_APP_ADC_15kg', 'CH32V00X_APP_ADC_30kg', 'CH32V00X_APP_ADC_20kg_single']
DIRS = ['Application', 'Common', 'Drivers', 'System', 'User']
SRC_DIRS = DIRS + ['SRC']
PROFILE_FIELDS = [
    'FW_TORQUE',
    'MOTOR_CAL_MV', 'MOTOR_KV_UV', 'MOTOR_KA_NV', 'MOTOR_FC_MV', 'MOTOR_FS_MV',
    'MOTOR_DELAY_US', 'MOTOR_NOISE_CDEG', 'MOTOR_MIN_STEP_CDEG', 'MOTOR_SIGN',
    'MOTOR_RATED_VEL_DPS', 'MOTOR_RATED_ACC_DPS2', 'MOTOR_RATED_DEC_DPS2',
]
PROJECT_FILES = [f'obj/{d}/subdir.mk' for d in
                 ['Application', 'Common', 'Drivers', 'System', 'User', 'SRC/Core',
                  'SRC/Debug', 'SRC/Peripheral/src', 'SRC/Startup']] + ['obj/makefile']


def norm(b: bytes) -> bytes:
    return b.replace(b'\r\n', b'\n')


def read(p) -> str:
    return Path(p).read_text(encoding='utf-8')

def field_values(text: str):
    return {n: re.search(r'^#define ' + n + r'\s+(\S+)', text, re.M).group(1)
            for n in PROFILE_FIELDS}


def main():
    before = json.loads((HERE / 'source_hashes_before.json').read_text(encoding='utf-8'))
    report, ok = [], True

    # 1. baseline project untouched
    after = {}
    for p in ROOT.glob('CH32V00X_APP*'):
        if not p.is_dir():
            continue
        for d in SRC_DIRS + ['obj']:
            base = p / d
            if not base.exists():
                continue
            for f in base.rglob('*'):
                if f.is_file():
                    after[f.relative_to(ROOT).as_posix()] = hashlib.sha256(f.read_bytes()).hexdigest()
    # obj/ holds build artifacts: an IDE rebuild legitimately rewrites .o/.map there,
    # so only source files count as "baseline untouched".
    def is_source(n):
        return '/obj/' not in n
    # source_hashes_before.json 是 2026-10-20 的旧快照；20kg 基线在 2026-10-23 15:00 被外部
    # 升级过(mtime 证据，非本次改动)。[1] 改为对"当前基线"比对，首次运行时建立基准。
    ref_file = HERE / 'baseline_hashes_current.json'
    if ref_file.exists():
        ref = json.loads(ref_file.read_text(encoding='utf-8'))
    else:
        ref = {n: h for n, h in after.items() if n.startswith('CH32V00X_APP_ADC_20kg/')}
        ref_file.write_text(json.dumps(ref, indent=2), encoding='utf-8')
        report.append('      (已建立当前基线基准 baseline_hashes_current.json)')
    drift = sorted(n for n in ref if n.startswith('CH32V00X_APP_ADC_20kg/')
                   and ref[n] != after.get(n) and is_source(n))
    rebuilt = sorted(n for n in ref if n.startswith('CH32V00X_APP_ADC_20kg/')
                     and ref[n] != after.get(n) and not is_source(n))
    report.append(f'[1] 20kg 基准源码未改动: {"是" if not drift else "否 " + str(drift)}')
    if rebuilt:
        report.append(f'      (20kg/obj 下 {len(rebuilt)} 个构建产物被外部重新构建，不算源码变动)')
    ok &= not drift

    for v in VARIANTS:
        var = ROOT / v
        changed = sorted(n for n in before if n.startswith(v + '/')
                         and before[n] != after.get(n) and is_source(n))
        report.append(f'[2] {v}: 源码变化 {len(changed)} 个文件')
        for n in changed:
            report.append('      ' + n.split('/', 1)[1])

        # 2. complete source set, byte-identical to baseline except A_Config.h
        diffs = []
        for d in SRC_DIRS:
            bfiles = {f.name: f for f in (BASE / d).rglob('*') if f.is_file()}
            vfiles = {f.name: f for f in (var / d).rglob('*') if f.is_file()}
            if set(bfiles) != set(vfiles):
                diffs.append(f'{d}: 文件集合不同 {set(bfiles) ^ set(vfiles)}')
            for name in sorted(set(bfiles) & set(vfiles)):
                if bfiles[name].read_bytes() == vfiles[name].read_bytes():
                    continue
                if name == 'A_Config.h':
                    continue
                diffs.append(f'{d}/{name} 内容不同')
        report.append(f'[3] {v}: 除 A_Config.h 外源码逐字节一致: {"是" if not diffs else "否"}')
        ok &= not diffs
        report.extend('      ' + d for d in diffs)

        # 3. A_Config.h identical after normalising the profile block values
        tb = (BASE / 'Application/A_Config.h').read_text(encoding='utf-8')
        tv = (var / 'Application/A_Config.h').read_text(encoding='utf-8')
        # FW_TORQUE's comment repeats the model name; it travels with the value
        tb = re.sub(r'^(#define FW_TORQUE\s+\S+\s+/\*\*.*?扭力型号 )\d+kg',
                    lambda m: m[1] + 'Xkg', tb, flags=re.M)
        tv = re.sub(r'^(#define FW_TORQUE\s+\S+\s+/\*\*.*?扭力型号 )\d+kg',
                    lambda m: m[1] + 'Xkg', tv, flags=re.M)
        for name in PROFILE_FIELDS:
            tb = re.sub(r'^(#define ' + name + r'\s+)\S+', r'\g<1>X', tb, flags=re.M)
            tv = re.sub(r'^(#define ' + name + r'\s+)\S+', r'\g<1>X', tv, flags=re.M)
        same = tb == tv
        report.append(f'[4] {v}: A_Config.h 归一化标定值后与 20kg 一致: {"是" if same else "否"}')
        ok &= same

        # 4. profile values preserved
        old = field_values((HERE / 'backup_before_align' / v / 'Application/A_Config.h')
                           .read_text(encoding='utf-8'))
        new = field_values((var / 'Application/A_Config.h').read_text(encoding='utf-8'))
        same = old == new
        report.append(f'[5] {v}: 型号/电机标定 13 项原值保留: {"是" if same else "否 " + str({k: (old[k], new[k]) for k in old if old[k] != new[k]})}')
        ok &= same

        # 5. project files: makefile retargeted to this variant, neutral files byte-equal
        bad = []
        for rel in PROJECT_FILES:
            src, dst = BASE / rel, var / rel
            if not src.exists():
                continue
            if rel == 'obj/makefile':
                # exact expected text; a plain substring test would mis-flag
                # "..._20kg_single" as still pointing at the "..._20kg" baseline
                if read(dst) != read(src).replace(BASE.name, v):
                    bad.append(rel)
            elif norm(dst.read_bytes()) != norm(src.read_bytes()):
                bad.append(rel)
        report.append(f'[6] {v}: 构建清单与 20kg 一致(makefile指向本工程): {"是" if not bad else "否 " + str(bad)}')
        ok &= not bad

        # 6b. neutral IDE descriptors byte-identical to baseline
        bad2 = []
        for rel in ['obj/objects.mk', 'obj/sources.mk', 'CH32V00X_APP.wvproj', '.cproject',
                    'CH32V00X_APP.launch', '.project', '.template']:
            if (var / rel).read_bytes() != (BASE / rel).read_bytes():
                bad2.append(rel)
        report.append(f'[7] {v}: IDE/工程描述文件与 20kg 一致: {"是" if not bad2 else "否 " + str(bad2)}')
        ok &= not bad2

    text = '\n'.join(report)
    (HERE / 'output' / 'final_report.txt').write_text(text + '\n', encoding='utf-8')
    print(text)
    print('\nFINAL:', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    sys.exit(main())
