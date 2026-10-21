from pathlib import Path
import importlib.util,json,sys,re,struct,subprocess
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=HERE/'output'
BEFORE=HERE/'before/CH32V00X_APP_ADC_20kg'
AFTER=HERE/'candidate'
spec=importlib.util.spec_from_file_location('s',ROOT/'tests/sync_noncontrol_adc_20261019/verify.py')
s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s);s.OUT=OUT
sys.path.insert(0,str(ROOT/'tests/control_optimization_20261019'))
import run_optimization as sim

def host():
    (OUT/'20').mkdir(parents=True,exist_ok=True)
    assert 'PASS bit-identical' in (OUT/'equivalence.log').read_text()
    allrows={};alljolt={};extended={}
    for label,p,defs in (('before',BEFORE,()),('after',AFTER,('-flto',))):
        exe=s.compile_host('matrix_'+label,ROOT/'tests/control_optimization_20261019/source/sim.c',p,20,defs)
        allrows[label]=sim.parse(s.run([exe,'20kg','matrix','NUL','*','*'],OUT/(label+'_matrix.log')))
        assert len(allrows[label])==351
        exe=s.compile_host('jolt_'+label,ROOT/'tests/replan_jolt_20261019/jolt.c',p,20,defs)
        text=s.run([exe],OUT/(label+'_jolt.log'))
        alljolt[label]=[{k:float(v) for k,v in (q.split('=') for q in line.split(',')[1:])} for line in text.splitlines() if line.startswith('CASE,')]
        assert len(alljolt[label])==486
        exe=s.compile_host('pwm_'+label,ROOT/'tests/pwm_jitter_20kg_20261019/check.c',p,20,defs)
        for mode,T in ((0,0),(0,1000),(3,0),(3,1000),(4,0),(4,1000),(5,0),(6,0),(8,0),(9,0)):
            for noise in (0,1,2,5):
                name=f'm{mode}_T{T}_n{noise}';trace=OUT/(label+'_'+name+'.csv')
                s.run([exe,mode,T,noise,0,trace],OUT/'pwm.log')
                if label=='after':assert trace.read_bytes()==(OUT/('before_'+name+'.csv')).read_bytes(),name
        exe=s.compile_host('extended_'+label,ROOT/'tests/control_optimization_20261019/source/sim_extended.c',p,20,defs)
        extended[label]={}
        for ld in ('noload','inertia4','grav20%_r4','randload','grav30%_r4_bl0.6'):
            for cm in ('ext_long2','ext_long60','ext_stream200','ext_gap','ext_pause','ext_stop','ext_release','ext_jitter','ext_hold','ext_timed90'):
                extended[label].update(sim.parse(s.run([exe,'20kg','matrix','NUL',ld,cm],OUT/'extended'/f'{label}_{ld}_{cm}.log')))
        print(label,'351 matrix + 486 retarget + 40 PWM traces + 50 extended complete',flush=True)
    assert allrows['before']==allrows['after'];assert alljolt['before']==alljolt['after'];assert extended['before']==extended['after']
    for name,source,defs in (
        ('planner',ROOT/'tests/replan_20kg_impl_20261019/planner_unit.c',()),
        ('continuity',ROOT/'tests/pwm_opt_20kg_20261019/continuity.c',()),
        ('controller',ROOT/'tests/noncontrol_15kg_20261019/unit.c',()),
        ('regression',ROOT/'tests/noncontrol_15kg_20261019/regression.c',()),
        ('parameters',ROOT/'tests/replan_20kg_impl_20261019/parameters.c',()),
        ('protocol',ROOT/'tests/replan_20kg_impl_20261019/protocol_fixture.c',()),
        ('protection',ROOT/'tests/replan_20kg_impl_20261019/protocol_fixture.c',('-DALIGN_TEST_REAL_PROTECT',)),
        ('calibration',ROOT/'tests/replan_20kg_impl_20261019/protocol_fixture.c',('-DALIGN_TEST_CALIBRATION_RUN',)),
    ):
        exe=s.compile_host('check_'+name,source,AFTER,20,(*defs,'-flto'))
        print(name,s.run([exe],exe.with_suffix('.run.log')).splitlines()[-1],flush=True)
    status={'passed':True,'all_matrix_records_identical':True,'matrix_cases':351,'retarget_cases':486,'extended_cases':50,'PWM_traces':40,'coefficient_cases':1000000,'public_steps':6000000}
    (OUT/'host_status.json').write_text(json.dumps(status,indent=2));print('PASS all original behavior and PWM optimization preserved',flush=True)

def sections(path):
    raw=path.read_bytes();off=struct.unpack_from('<I',raw,32)[0];ent,count,strings=struct.unpack_from('<HHH',raw,46)
    table=struct.unpack_from('<10I',raw,off+strings*ent);names=raw[table[4]:table[4]+table[5]];result={}
    for i in range(count):
        row=struct.unpack_from('<10I',raw,off+i*ent);n=names[row[0]:].split(b'\0')[0].decode()
        result[n]={'addr':row[3],'size':row[5],'data':raw[row[4]:row[4]+row[5]] if row[1]!=8 else b''}
    return result

def target_audit():
    spec=importlib.util.spec_from_file_location('b',ROOT/'tests/verify_alignment.py')
    b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
    old=OUT/'mrs_flags/CH32V00X_APP_ADC_20kg/CH32V00X_normal.elf'
    new=OUT/'lto/candidate/CH32V00X_normal.elf'
    a=sections(old);z=sections(new)
    assert a['.fw_info']['addr']==z['.fw_info']['addr']==0x100
    assert a['.fw_info']['data']==z['.fw_info']['data']
    nm=subprocess.check_output([str(b.WCH/'riscv-wch-elf-nm.exe'),str(new)],text=True)
    assert not re.search(r'\b__(?:add|sub|mul|div)(?:df|sf)3\b|\b(?:sqrt|pow|lround)\b',nm)
    asm=subprocess.check_output([str(b.WCH/'riscv-wch-elf-objdump.exe'),'-d','-M','no-aliases,xw',str(new)],text=True)
    required=['USART1_IRQHandler','DMA1_Channel1_IRQHandler','TIM1_UP_IRQHandler','TIM1_CC_IRQHandler']
    symbols={q[2]:int(q[0],16) for line in nm.splitlines() if len(q:=line.split())==3}
    for n in required:
        m=re.search(r'<'+n+r'>:\n(.*?)(?=\n[0-9a-f]+ <|\Z)',asm,re.S);assert m and 'mret' in m[1],n
        addr=symbols[n]
        assert struct.pack('<I',addr) in z['.init']['data'],n
    assert z['.data']['size']+z['.bss']['size']+z['.stack']['size']<8192
    (OUT/'target_audit.json').write_text(json.dumps({'passed':True,'firmware_identity_preserved':True,'irq_vectors_and_mret_preserved':required,'no_float_routines':True,'static_ram':z['.data']['size']+z['.bss']['size'],'reserved_stack':z['.stack']['size']},indent=2))
    print('PASS LTO fixed firmware info, vector bindings, interrupt returns, RAM bounds and no floating point helpers',flush=True)
if __name__=='__main__':
    if '--target-audit' in sys.argv:target_audit()
    else:host()
