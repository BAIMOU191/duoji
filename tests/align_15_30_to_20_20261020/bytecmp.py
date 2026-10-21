"""Byte-level compare of app sources: 15kg / 30kg against the 20kg baseline.

Normalises CRLF vs LF only (byte diffs otherwise), so the report separates
"same text, different newline style" from real content differences.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'CH32V00X_APP_ADC_20kg'
VARIANTS = ['CH32V00X_APP_ADC_15kg', 'CH32V00X_APP_ADC_30kg']
DIRS = ['Application', 'Common', 'Drivers', 'System', 'User']


def norm(b: bytes) -> bytes:
    return b.replace(b'\r\n', b'\n')


def main():
    out = []
    for v in VARIANTS:
        other = ROOT / v
        out.append(f'########## {v} vs CH32V00X_APP_ADC_20kg (byte compare) ##########')
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
            ba, bb = a.read_bytes(), b.read_bytes()
            if ba == bb:
                out.append(f'BYTE-SAME      {rel}')
            elif norm(ba) == norm(bb):
                out.append(f'EOL-ONLY       {rel}')
            else:
                out.append(f'CONTENT-DIFF   {rel}')
    text = '\n'.join(out)
    (Path(__file__).parent / 'byte_report.txt').write_text(text, encoding='utf-8')
    print(text)


if __name__ == '__main__':
    main()
