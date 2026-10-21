"""Restore V/A/D only; reuse control/protocol regression against actual 15kg code."""
from pathlib import Path
import hashlib
import json
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/'tests/control_15kg_impl_20261019'))
import verify_impl as check
check.HERE=HERE
check.OUT=HERE/'output'


def verify_scope():
    start=json.loads((HERE/'source_hashes_before.json').read_text())
    changed=[name for name,sha in start.items() if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=sha]
    assert changed==['CH32V00X_APP_ADC_15kg/Application/A_Config.h'],changed
    (check.OUT/'changed_sources.json').write_text(json.dumps(changed,indent=2),encoding='utf-8')
    print('PASS scope: only 15kg V/A/D defaults changed; all controller sources unchanged',flush=True)


check.verify_scope=verify_scope

if __name__=='__main__':
    check.simulation();check.checks();check.target();verify_scope()
    import package_test_firmware as package
    package.package()
