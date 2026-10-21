"""Audit text-level differences of 15kg/30kg app sources against the 20kg baseline."""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'CH32V00X_APP_ADC_20kg'
VARIANTS = ['CH32V00X_APP_ADC_15kg', 'CH32V00X_APP_ADC_30kg']
# Source dirs that make up the application code.
DIRS = ['Application', 'Common', 'Drivers', 'System', 'User']


def read(p: Path) -> str:
    return p.read_text(encoding='utf-8')


def main():
    out = []
    for v in VARIANTS:
        other = ROOT / v
        out.append(f'########## {v} vs CH32V00X_APP_ADC_20kg ##########')
        names = set()
        for d in DIRS:
            for p in (BASE / d).glob('*'):
                if p.is_file():
                    names.add(f'{d}/{p.name}')
            for p in (other / d).glob('*'):
                if p.is_file():
                    names.add(f'{d}/{p.name}')
        for rel in sorted(names):
            a, b = other / rel, BASE / rel
            if not a.exists():
                out.append(f'--- ONLY IN 20kg: {rel}')
                continue
            if not b.exists():
                out.append(f'--- ONLY IN {v}: {rel}')
                continue
            ta, tb = read(a), read(b)
            if ta == tb:
                out.append(f'--- SAME: {rel}')
                continue
            out.append(f'--- DIFF: {rel}  ({len(ta.splitlines())} vs {len(tb.splitlines())} lines)')
            la, lb = ta.splitlines(), tb.splitlines()
            import difflib
            for line in difflib.unified_diff(lb, la, fromfile='20kg/' + rel,
                                             tofile=v + '/' + rel, lineterm='', n=2):
                out.append(line)
    text = '\n'.join(out)
    (Path(__file__).parent / 'audit_report.txt').write_text(text, encoding='utf-8')
    print(text)


if __name__ == '__main__':
    main()
