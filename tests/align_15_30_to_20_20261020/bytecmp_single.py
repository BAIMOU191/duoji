"""Byte-level compare of one app variant against the 20kg baseline."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'CH32V00X_APP_ADC_20kg'
DIRS = ['Application', 'Common', 'Drivers', 'System', 'User', 'SRC']


def norm(b: bytes) -> bytes:
    return b.replace(b'\r\n', b'\n')


def main(v: str, out_name: str):
    other = ROOT / v
    out = [f'########## {v} vs CH32V00X_APP_ADC_20kg (byte compare) ##########']
    names = set()
    for d in DIRS:
        for base in (BASE / d).rglob('*'):
            if base.is_file():
                names.add(base.relative_to(BASE).as_posix())
        for p in other.joinpath(d).rglob('*'):
            if p.is_file():
                names.add(p.relative_to(other).as_posix())
    for rel in sorted(names):
        a, b = other / rel, BASE / rel
        if not a.exists():
            out.append(f'ONLY-IN-20kg   {rel}')
            continue
        if not b.exists():
            out.append(f'ONLY-IN-VAR    {rel}')
            continue
        ba, bb = a.read_bytes(), b.read_bytes()
        if ba == bb:
            out.append(f'BYTE-SAME      {rel}')
        elif norm(ba) == norm(bb):
            out.append(f'EOL-ONLY       {rel}')
        else:
            out.append(f'CONTENT-DIFF   {rel}')
    text = '\n'.join(out)
    (Path(__file__).parent / out_name).write_text(text, encoding='utf-8')
    print(text)


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    main('CH32V00X_APP_ADC_20kg_single', 'byte_single.txt')
