"""Compare subdir.mk / obj makefiles: 15kg and 30kg against the 20kg baseline."""
from pathlib import Path
import difflib

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'CH32V00X_APP_ADC_20kg'
FILES = ['obj/makefile', 'obj/Application/subdir.mk', 'obj/Common/subdir.mk']


def main():
    for rel in FILES:
        b = (BASE / rel).read_text(encoding='utf-8').splitlines()
        for v in ['CH32V00X_APP_ADC_15kg', 'CH32V00X_APP_ADC_30kg']:
            a = (ROOT / v / rel).read_text(encoding='utf-8').splitlines()
            d = list(difflib.unified_diff(b, a, '20kg/' + rel, v + '/' + rel, lineterm='', n=1))
            print(f'===== {v} {rel}: {"IDENTICAL" if not d else str(len(d)) + " diff lines"}')
            for line in d[:60]:
                print(line)


if __name__ == '__main__':
    main()
