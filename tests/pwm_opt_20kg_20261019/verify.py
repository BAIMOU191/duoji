from pathlib import Path
import importlib.util,json,sys,subprocess
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
spec=importlib.util.spec_from_file_location('ex',HERE/'experiment.py')
ex=importlib.util.module_from_spec(spec);spec.loader.exec_module(ex)
OUT=ex.OUT
SELECT='m1_r0'

def main():
    matrices=json.loads((OUT/'matrix.json').read_text());base=matrices['base'];chosen=matrices[SELECT]
    assert not [k for k in base if chosen[k]['fault']>base[k]['fault']]
    assert not [k for k in base if abs(chosen[k]['final'])>.5 and abs(base[k]['final'])<=.5]
    assert all(r['hard_violations']==0 for r in chosen.values())
    results={}
    for label,p in (('before',HERE/'before'),('candidate',HERE/'candidates'/SELECT)):
        jolt=ex.build('jolt_'+label,p,ROOT/'tests/replan_jolt_20261019/jolt.c')
        rows=[]
        text=ex.s.run([jolt],OUT/(label+'_jolt.log'))
        for line in text.splitlines():
            if line.startswith('CASE,'):rows.append({k:float(v) for k,v in (x.split('=') for x in line.split(',')[1:])})
        assert len(rows)==486 and all(r['hard']==0 for r in rows)
        extra=ex.build('extended_'+label,p,ROOT/'tests/control_optimization_20261019/source/sim_extended.c')
        extended={}
        for ld in ('noload','inertia4','grav20%_r4','randload','grav30%_r4_bl0.6'):
            for command in ('ext_long2','ext_long60','ext_stream200','ext_gap','ext_pause','ext_stop','ext_release','ext_jitter','ext_hold','ext_timed90'):
                extended.update(ex.sim.parse(ex.s.run([extra,'20kg','matrix','NUL',ld,command],OUT/'extended'/f'{label}_{ld}_{command}.log')))
        assert len(extended)==50 and all(r['hard_violations']==0 for r in extended.values())
        results[label]={'jolt':rows,'extended':extended}
        (OUT/'verification.json').write_text(json.dumps(results,indent=2))
        print(label,'jolt faults',sum(r['fault'] for r in rows),'out',sum(abs(r['final'])>.5 for r in rows),'extended faults',sum(r['fault'] for r in extended.values()),flush=True)
    a=results['candidate'];b=results['before']
    newfaults=[('jolt',i) for i,(old,new) in enumerate(zip(b['jolt'],a['jolt'])) if new['fault']>old['fault']]
    newfaults += [('extended',k) for k in b['extended'] if a['extended'][k]['fault']>b['extended'][k]['fault']]
    newouts=[('jolt',i,old['final'],new['final']) for i,(old,new) in enumerate(zip(b['jolt'],a['jolt'])) if abs(new['final'])>.5 and abs(old['final'])<=.5]
    newouts += [('extended',k,b['extended'][k]['final'],a['extended'][k]['final']) for k in b['extended'] if abs(a['extended'][k]['final'])>.5 and abs(b['extended'][k]['final'])<=.5]
    print('new faults',newfaults,'new outside .5 deg',newouts,flush=True)
    (OUT/'additional_comparison.json').write_text(json.dumps({'new_faults':newfaults,'new_outside_0_5_deg':newouts},indent=2))
    assert not newfaults and not newouts
    for name,source,defs in (
        ('controller',ROOT/'tests/noncontrol_15kg_20261019/unit.c',()),
        ('regression',ROOT/'tests/noncontrol_15kg_20261019/regression.c',()),
        ('planner',ROOT/'tests/replan_20kg_impl_20261019/planner_unit.c',()),
        ('parameters',ROOT/'tests/replan_20kg_impl_20261019/parameters.c',()),
        ('protocol',ROOT/'tests/replan_20kg_impl_20261019/protocol_fixture.c',()),
        ('protection',ROOT/'tests/replan_20kg_impl_20261019/protocol_fixture.c',('-DALIGN_TEST_REAL_PROTECT',)),
        ('calibration',ROOT/'tests/replan_20kg_impl_20261019/protocol_fixture.c',('-DALIGN_TEST_CALIBRATION_RUN',)),
        ('continuity',HERE/'continuity.c',()),
    ):
        exe=ex.s.compile_host('check_'+name,source,HERE/'candidates'/SELECT,20,defs)
        print(name,ex.s.run([exe],exe.with_suffix('.run.log')).splitlines()[-1],flush=True)
    (OUT/'selected.json').write_text(json.dumps({'profile':SELECT,'checks_passed':True,'matrix':351,'retarget':486,'extended':50,'pwm_cases':40},indent=2))
    print('Completed independent checks',flush=True)
if __name__=='__main__':main()
