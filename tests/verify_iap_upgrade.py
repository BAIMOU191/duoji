"""Exercise the real Boot and APP upgrade paths together at hardware boundaries."""
from pathlib import Path
from verify_alignment import ROOT, OUT, calibrated, test_profile, run

if __name__ == '__main__':
    for project in sorted(ROOT.glob('CH32V00X_APP*')):
        profile = project if calibrated(project) else test_profile(project)
        out = OUT/project.name
        out.mkdir(parents=True, exist_ok=True)
        inc = ['-I'+str(ROOT/'tests/stubs')]+['-I'+str(profile/d) for d in
                ['Application', 'Common', 'Drivers', 'System', 'SRC/Debug']]
        exe = out/'iap_upgrade_sim.exe'
        run([r'D:\SoftWare\mingw64\bin\gcc.exe', '-std=gnu99', '-O2', '-Wall', '-Wextra', '-Werror',
             *inc, ROOT/'tests/boot_upgrade_sim.c', ROOT/'tests/iap_upgrade_sim.c', '-o', exe],
            out/'iap_upgrade_compile.log')
        print(project.name, run([exe], out/'iap_upgrade_run.log').strip(), flush=True)
