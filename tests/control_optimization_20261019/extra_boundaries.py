"""Additional checks justified by remaining post-disturbance drift and model uncertainty."""
import json
import run_optimization as r


def run():
    result={}
    for kg in (15,20,30):
        for profile in ('baseline','optimized','handoff','opt_hold_i10','opt_hold_i30','opt_hold_i60'):
            exe=r.compile_host(profile,kg,extended=True)
            knobs=[f'HOLDKI={r.PROFILES[profile]["RUN_HOLD_KI"]}'] if 'RUN_HOLD_KI' in r.PROFILES[profile] else []
            key=f'{profile}_{kg}'
            result[key]=r.parse(r.run([exe,f'{kg}kg','matrix','NUL','push_r4','ext_hold',*knobs],r.OUT/'logs'/f'push_hold_{key}.txt'))
        print('PUSH HOLD',kg,flush=True)
    (r.OUT/'push_hold.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    result={}
    for kg in (15,20,30):
        for profile in ('baseline','handoff'):
            exe=r.OUT/f'{profile}_{kg}_extended.exe'
            for rw in (1.6,2.4,3.2,4.8):
                for load in ('inertia8','grav40%_r6'):
                    key=f'{profile}_{kg}_RW{rw}_{load}'
                    result[key]=r.parse(r.run([exe,f'{kg}kg','matrix','NUL',load,'reverse400',f'RW={rw}'],r.OUT/'logs'/f'sensitivity_{key}.txt'))
        print('RESISTANCE ASSUMPTION',kg,flush=True)
    (r.OUT/'sensitivity.json').write_text(json.dumps(result,indent=2),encoding='utf-8')


if __name__=='__main__':
    run()
