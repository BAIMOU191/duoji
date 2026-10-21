"""Report per-file difference scope for 15kg/30kg app sources against the 20kg baseline."""
from pathlib import Path
import difflib

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'CH32V00X_APP_ADC_20kg'
VARIANTS = ['CH32V00X_APP_ADC_15kg', 'CH32V00X_APP_ADC_30kg']
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
                out.append(f'ONLY-IN-20kg   {rel}')
                continue
            if not b.exists():
                out.append(f'ONLY-IN-VAR   {rel}')
                continue
            ta, tb = read(a), read(b)
            if ta == tb:
                out.append(f'SAME           {rel}')
                continue
            sm = difflib.SequenceMatcher(None, tb.splitlines(), ta.splitlines(), autojunk=False)
            added = removed = 0
            hunks = []
            for tag, i1, i2, j1, j2 in sm.get_opcodes():
                if tag == 'equal':
                    continue
                added += max(0, j2 - j1)
                removed += max(0, i2 - i1)
                first = (ta.splitlines()[j1] if j1 < len(ta.splitlines()) else '')[:60]
                hunks.append(f'{tag} -{i2 - i1}/+{j2 - j1} @{i1 + 1}: {first}')
            out.append(f'DIFF           {rel}  (-{removed}/+{added} lines, {len(hunks)} hunks)')
            for h in hunks:
                out.append('      ' + h)
    text = '\n'.join(out)
    (Path(__file__).parent / 'scope_report.txt').write_text(text, encoding='utf-8')
    print(text)


if __name__ == '__main__':
    main()
