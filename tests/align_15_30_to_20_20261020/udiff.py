"""Generic UTF-8 unified diff of one variant against the 20kg baseline."""
from pathlib import Path
import difflib
import sys

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'CH32V00X_APP_ADC_20kg'
DIRS = ['Application', 'Common', 'Drivers', 'System', 'User']


def main(v: str, files=None):
    other = ROOT / v
    for d in DIRS:
        bmap = {f.name: f for f in (BASE / d).glob('*') if f.is_file()}
        vmap = {f.name: f for f in (other / d).glob('*') if f.is_file()}
        for name in sorted(set(bmap) | set(vmap)):
            rel = f'{d}/{name}'
            if files and rel not in files:
                continue
            if name not in bmap:
                print(f'--- ONLY-IN-VAR    {rel}')
                continue
            if name not in vmap:
                print(f'--- ONLY-IN-20kg   {rel}')
                continue
            tb = bmap[name].read_text(encoding='utf-8').splitlines()
            tv = vmap[name].read_text(encoding='utf-8').splitlines()
            if tb == tv:
                continue
            print(f'=== DIFF {rel}')
            for line in difflib.unified_diff(tb, tv, f'20kg/{rel}', f'{v}/{rel}',
                                             lineterm='', n=2):
                print(line)


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    want = set(sys.argv[2:])
    main(sys.argv[1], want or None)
