"""Summarize the reproduced data and check archived candidates with the MCU toolchain.

Only copies below continued_validation are compiled; production sources are read only.
"""
from pathlib import Path
import importlib.util
import json
import re
import shutil
import contextlib
import io
from concurrent.futures import ThreadPoolExecutor

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUT = HERE / 'continued_validation'


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def summarize():
    archived = module('claude_summary', HERE / 'sim_conservative/runcfg.py')
    matrix = json.loads((OUT / 'matrix.json').read_text())
    summary = {}
    for package, models in matrix.items():
        summary[package] = {}
        for kg, rows in models.items():
            data = {tuple(k.split('|')): v for k, v in rows.items()}
            stats = archived.summarize(data)
            stats['loaded_point_cases'] = len(archived.LOADED) * len(archived.PM)
            stats['point_fault_events'] = sum(data[l, c]['fault'] for l in archived.LOADED for c in archived.PM)
            stats['point_stall_events'] = sum(data[l, c]['stall'] for l in archived.LOADED for c in archived.PM)
            stats['point_bad_final_despite_arrival'] = sum(abs(data[l, c]['final']) > .5 and data[l, c]['arr'] >= 0 for l in archived.LOADED for c in archived.PM)
            summary[package][kg] = stats
    (OUT / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    for package in ('base0', 'A0', 'T1', 'T2', 'T3', 'T3_J10', 'T3_J30', 'T3_J40'):
        print(package, ' | '.join(f'{kg}: med={d["os_med"]:.2f} p90={d["os_p90"]:.2f} max={d["os_max"]:.2f} ring={d["ring"]} out={d["out"]}/70 fault={d["fault"]:.0f} stall={d["stall"]:.0f}' for kg, d in sorted(summary[package].items())), flush=True)


def target_check():
    build = module('target_build', ROOT / 'tests/verify_alignment.py')
    build.OUT = OUT / 'target_builds'
    packages = json.loads((OUT / 'packages.json').read_text())
    results = {}
    # Baseline control establishes that the compiler/linker environment itself works.
    for package, kg in [('base0', 15), ('T3', 15), ('T3', 20), ('T3', 30)]:
        stage = OUT / 'target_sources' / f'{package}_{kg}kg'
        for directory in ('Application', 'Common', 'Drivers', 'System', 'User', 'SRC'):
            shutil.copytree(ROOT / f'CH32V00X_APP_ADC_{kg}kg' / directory, stage / directory, dirs_exist_ok=True)
        if package != 'base0':
            for file in (OUT / 'source/cons').iterdir():
                if not file.is_file():
                    continue
                source = file.read_text(encoding='utf-8')
                for key, value in packages[package].items():
                    source = re.sub(r'(\b' + key + r'\s*=\s*)-?\d+', lambda m: m[1] + str(value), source)
                destination = stage / ('Application' if file.name == 'A_Servo.c' else 'Common') / file.name
                destination.write_text(source, encoding='utf-8')
        log = io.StringIO()
        try:
            with contextlib.redirect_stdout(log):
                warnings = build.build(stage)
            results[stage.name] = {'passed': True, 'warnings': warnings, 'log': log.getvalue()}
        except Exception as exc:
            results[stage.name] = {'passed': False, 'log': log.getvalue() + str(exc)}
        (OUT / 'logs' / f'target_{stage.name}.txt').write_text(results[stage.name]['log'], encoding='utf-8')
        print('TARGET', stage.name, 'PASS' if results[stage.name]['passed'] else 'FAIL', results[stage.name]['log'][-1200:], flush=True)
    (OUT / 'target_results.json').write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8')


def long_command_check():
    runner = module('continue_runner', HERE / 'continue_validation.py')
    source = OUT / 'source'
    runner.instrument_source(source)
    text = (source / 'sim_instrumented.c').read_text(encoding='utf-8')
    scripts = '''
static void sc_audit_long2(int t) {
    if(t<=4000 && t%100==0){cmd(13500+1000.0*t/1000,2000);g_motion_end=t;}
}
static void sc_audit_long60(int t) {
    if(t<=4000 && t%100==0){cmd(13500+1000.0*t/1000,60000);g_motion_end=t;}
}
static void sc_audit_gap(int t) {
    if(t<=4000 && (t<1000 || t>=2000) && t%100==0){cmd(13500+1000.0*t/1000,0);g_motion_end=t;}
}
'''
    text = text.replace('/* ---------- 负载函数 ---------- */', scripts + '\n/* ---------- 负载函数 ---------- */')
    text = text.replace('CmdCase cmds[] = {', '''CmdCase cmds[] = {
        {"audit_long2",sc_audit_long2,8000,0},
        {"audit_long60",sc_audit_long60,66000,0},
        {"audit_gap",sc_audit_gap,6500,0},''')
    (source / 'sim_instrumented.c').write_text(text, encoding='utf-8')
    executables = {(kg, cand): runner.compile_host(source, kg, cand, instrumented=True) for kg in (15, 20, 30) for cand in (False, True)}
    jobs = [(kg, package, load, command) for kg in (15, 20, 30) for package in ('base0', 'T2', 'T3') for load in ('noload', 'grav20%_r4') for command in ('audit_long2', 'audit_long60', 'audit_gap')]

    def job(item):
        kg, package, load, command = item
        key = f'{package}_{kg}_{load}_{command}'
        output = runner.execute([executables[kg, package != 'base0'], f'{kg}kg', 'matrix', 'NUL', load, command,
                                 *[f'{k}={v}' for k,v in runner.PACKAGES[package].items()]], OUT / 'logs' / (key + '.txt'))
        parsed = runner.parse(output)
        assert len(parsed) == 1 and f'{load}|{command}' in parsed, key
        return key, parsed

    with ThreadPoolExecutor(max_workers=3) as pool:
        result = dict(pool.map(job, jobs))
    (OUT / 'long_commands.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print('LONG COMMANDS:', len(result), 'completed', flush=True)


def plot_results():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    summary = json.loads((OUT / 'summary.json').read_text())
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.7), layout='constrained')
    x = np.arange(3)
    for ax, metric, title in zip(axes, ('out', 'ring', 'os_med'),
                               ('Final error > 0.5 deg (70 cases)', 'Post-reference target-side changes', 'Median overshoot (deg)')):
        for i, (package, label, color) in enumerate((('base0','Baseline','#607d8b'), ('T2','T2 (DOB off)','#168a85'), ('T3','T3 (DOB on)','#df843c'))):
            ax.bar(x + (i-1)*.24, [summary[package][str(kg)][metric] for kg in (15,20,30)], width=.23, label=label, color=color)
        ax.set_xticks(x, ['15kg','20kg','30kg'])
        ax.set_title(title, fontsize=10)
        ax.grid(axis='y',alpha=.2)
        ax.set_axisbelow(True)
    axes[0].legend(fontsize=8)
    fig.suptitle('Conservative prototype trade-offs (simulation, not hardware certification)', fontsize=12)
    fig.savefig(OUT/'conservative_tradeoffs.png', dpi=180)
    plt.close(fig)


if __name__ == '__main__':
    summarize()
    long_command_check()
    plot_results()
    target_check()
