"""Build 20kg_single: production source (expect calibration-only static asserts)
and a synthetic-calibrated TEST COPY (proves the aligned code compiles and links).

Nothing here writes to the production project; the synthetic copy lives under output/.
"""
from pathlib import Path
import re
import shutil
import sys

import build as B

ROOT = B.ROOT
BASE = ROOT / 'CH32V00X_APP_ADC_20kg'
SINGLE = ROOT / 'CH32V00X_APP_ADC_20kg_single'
STAGE = B.OUT / 'CH32V00X_APP_ADC_20kg_single_synthetic_model_TARGET_CHECK_ONLY'

CAL_FIELDS = ['MOTOR_CAL_MV', 'MOTOR_KV_UV', 'MOTOR_KA_NV', 'MOTOR_FC_MV', 'MOTOR_FS_MV',
              'MOTOR_DELAY_US', 'MOTOR_NOISE_CDEG', 'MOTOR_MIN_STEP_CDEG', 'MOTOR_SIGN',
              'MOTOR_RATED_VEL_DPS', 'MOTOR_RATED_ACC_DPS2', 'MOTOR_RATED_DEC_DPS2']


def stage():
    if STAGE.exists():
        shutil.rmtree(STAGE)
    for d in ['Application', 'Common', 'Drivers', 'System', 'User', 'SRC']:
        shutil.copytree(SINGLE / d, STAGE / d)
    # TEST COPY ONLY: borrow the baseline's valid calibration so static asserts can
    # be exercised. The production source keeps its own pending (zero) calibration.
    donor = (BASE / 'Application/A_Config.h').read_text(encoding='utf-8')
    path = STAGE / 'Application/A_Config.h'
    cfg = path.read_text(encoding='utf-8')
    for name in CAL_FIELDS:
        value = re.search(r'^#define ' + name + r'\s+(\S+)', donor, re.M).group(1)
        cfg, n = re.subn(r'^(#define ' + name + r'\s+)\S+', lambda m: m[1] + value, cfg, flags=re.M)
        assert n == 1, name
    path.write_text(cfg, encoding='utf-8', newline='\n')
    print('staged synthetic test copy ->', STAGE)


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    stage()
    print('\n--- synthetic-calibrated TEST COPY (normal + calibration) ---')
    text, warnings = B.build('single_synthetic', project=STAGE,
                            out=B.OUT / 'single_synthetic_build')
    print(text)
    print('compiler warnings:', warnings.count('warning:'))


if __name__ == '__main__':
    main()
