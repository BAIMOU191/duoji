"""Build a merged A_Config.h per variant: 20kg template + variant calibration values."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'CH32V00X_APP_ADC_20kg'
VARIANTS = ['CH32V00X_APP_ADC_15kg', 'CH32V00X_APP_ADC_30kg']
OUT = Path(__file__).parent / 'merged'

# Fields allowed to differ between models (hardware identity + motor calibration).
PROFILE_FIELDS = [
    'FW_TORQUE',
    'MOTOR_CAL_MV', 'MOTOR_KV_UV', 'MOTOR_KA_NV', 'MOTOR_FC_MV', 'MOTOR_FS_MV',
    'MOTOR_DELAY_US', 'MOTOR_NOISE_CDEG', 'MOTOR_MIN_STEP_CDEG', 'MOTOR_SIGN',
    'MOTOR_RATED_VEL_DPS', 'MOTOR_RATED_ACC_DPS2', 'MOTOR_RATED_DEC_DPS2',
]


def field_values(text: str):
    vals = {}
    for name in PROFILE_FIELDS:
        m = re.search(r'^#define ' + name + r'\s+(\S+)', text, re.M)
        vals[name] = m.group(1) if m else None
    return vals


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    template = (BASE / 'Application/A_Config.h').read_text(encoding='utf-8')
    # model name in FW_TORQUE's comment follows the model value in the template
    template = re.sub(r'^(#define FW_TORQUE\s+\S+\s+/\*\*.*?扭力型号 )\d+kg',
                      lambda m: m[1] + '20kg', template, flags=re.M)
    report = []
    for v in VARIANTS:
        src = (ROOT / v / 'Application/A_Config.h').read_text(encoding='utf-8')
        keep = field_values(src)
        out = template
        # keep the model name inside the trailing comment of FW_TORQUE as well
        out = re.sub(r'^(#define FW_TORQUE\s+\S+\s+/\*\*.*?扭力型号 )\d+kg',
                     lambda m: m[1] + keep['FW_TORQUE'].rstrip('U') + 'kg', out, flags=re.M)
        for name in PROFILE_FIELDS:
            if keep[name] is None:
                continue
            new, n = re.subn(r'^(#define ' + name + r'\s+)\S+', lambda m: m[1] + keep[name],
                             out, flags=re.M)
            assert n == 1, (v, name, n)
            out = new
        (OUT / f'{v}__A_Config.h').write_text(out, encoding='utf-8', newline='\n')
        report.append(f'{v}: ' + ', '.join(f'{k}={keep[k]}' for k in PROFILE_FIELDS))
        # sanity: template fields outside the profile set are untouched
        for name in re.findall(r'^#define (\w+)', out, re.M):
            pass
    (OUT / 'profile_values.txt').write_text('\n'.join(report), encoding='utf-8')
    print('\n'.join(report))


if __name__ == '__main__':
    main()
