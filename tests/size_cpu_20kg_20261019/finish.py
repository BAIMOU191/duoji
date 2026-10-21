from pathlib import Path
import importlib.util,hashlib,json,re,shutil,sys,subprocess,difflib,xml.etree.ElementTree as ET
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=HERE/'output'
PROJECT=ROOT/'CH32V00X_APP_ADC_20kg'
FILES=('Common/C_Traj_Planner.c','.cproject','CH32V00X_APP.wvproj')

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def apply():
    assert json.loads((OUT/'host_status.json').read_text())['passed']
    assert json.loads((OUT/'target_audit.json').read_text())['passed']
    before=HERE/'before'/PROJECT.name
    for rel in FILES:assert digest(PROJECT/rel)==digest(before/rel),rel
    shutil.copy2(HERE/'candidate/Common/C_Traj_Planner.c',PROJECT/'Common/C_Traj_Planner.c')
    p=PROJECT/'CH32V00X_APP.wvproj';text=p.read_text(encoding='utf-8');data=json.loads(text)
    assert len(data['buildConfig']['configurations'])==1
    config=data['buildConfig']['configurations'][0]
    assert config['ccompiler']['optimization']['other_optimization_flags']==''
    assert config['clinker']['miscellaneous']['other_linker_flags']==''
    start=text.index('"ccompiler"');a=text[:start];b=text[start:]
    b=b.replace('"other_optimization_flags": ""','"other_optimization_flags": "-flto"',1);text=a+b
    start=text.index('"clinker"');a=text[:start];b=text[start:]
    b=b.replace('"other_linker_flags": ""','"other_linker_flags": "-flto"',1);text=a+b
    p.write_text(text,encoding='utf-8',newline='\n')
    p=PROJECT/'.cproject';text=p.read_text(encoding='utf-8')
    for option in ('c.compiler.otheroptimizations','c.linker.other'):
        pattern=r'(<option\b[^\n]*\bsuperClass="ilg\.gnumcueclipse\.managedbuild\.cross\.riscv\.option\.'+re.escape(option)+r'"[^\n]*\bvalue=")("[^\n]*/>)'
        text,count=re.subn(pattern,r'\g<1>-flto\g<2>',text);assert count==1,(option,count)
    p.write_text(text,encoding='utf-8',newline='\n')
    scope();print('PASS applied exact planner optimization and compile/link LTO options',flush=True)

def scope():
    manifest=json.loads((HERE/'source_hashes_before.json').read_text())
    changed=[n for n,h in manifest.items() if digest(ROOT/n)!=h]
    assert changed==['CH32V00X_APP_ADC_20kg/Common/C_Traj_Planner.c'],changed
    before=HERE/'before'/PROJECT.name
    for rel in ('Common/C_Pos_Ctrl.c','Common/C_Pos_Ctrl.h','Application/A_Config.h','Application/A_Servo.c'):
        assert digest(PROJECT/rel)==digest(before/rel),rel
    data=json.loads((PROJECT/'CH32V00X_APP.wvproj').read_text(encoding='utf-8'))['buildConfig']['configurations'][0]
    assert data['ccompiler']['optimization']['other_optimization_flags']=='-flto'
    assert data['clinker']['miscellaneous']['other_linker_flags']=='-flto'
    opts={x.attrib.get('superClass'):x.attrib.get('value') for x in ET.parse(PROJECT/'.cproject').iter('option')}
    for option in ('c.compiler.otheroptimizations','c.linker.other'):
        assert opts['ilg.gnumcueclipse.managedbuild.cross.riscv.option.'+option]=='-flto'
    patch=''.join(''.join(difflib.unified_diff((before/rel).read_text(encoding='utf-8').splitlines(True),(PROJECT/rel).read_text(encoding='utf-8').splitlines(True),fromfile='before/'+rel,tofile='after/'+rel)) for rel in FILES)
    (HERE/'implementation.diff').write_text(patch,encoding='utf-8')
    (OUT/'changed_sources.json').write_text(json.dumps(changed,indent=2))
    print('PASS only 20kg planner source and two project LTO settings changed; PWM, calibration, 15/30kg preserved',flush=True)

def package():
    scope()
    spec=importlib.util.spec_from_file_location('s',ROOT/'tests/sync_noncontrol_adc_20261019/verify.py')
    s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s);s.OUT=OUT
    # Confirm the production source exactly equals the validated candidate, independent of project flags.
    assert (PROJECT/'Common/C_Traj_Planner.c').read_bytes()==(HERE/'candidate/Common/C_Traj_Planner.c').read_bytes()
    exe=s.compile_host('final_planner',ROOT/'tests/replan_20kg_impl_20261019/planner_unit.c',PROJECT,20,('-flto',))
    print(s.run([exe],exe.with_suffix('.run.log')).splitlines()[-1],flush=True)
    spec=importlib.util.spec_from_file_location('b',ROOT/'tests/verify_alignment.py')
    b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b);b.FLAGS+=['-g','-flto'];b.OUT=OUT/'final'
    b.build(PROJECT);normal=b.OUT/PROJECT.name
    b.OUT=OUT/'diagnostic';b.FLAGS+=['-DTASK_FIREWATER_ENABLE=1'];b.build(PROJECT);debug=b.OUT/PROJECT.name
    spec=importlib.util.spec_from_file_location('audit',HERE/'validate.py')
    audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
    old=audit.sections(HERE/'output/mrs_flags/CH32V00X_APP_ADC_20kg/CH32V00X_normal.elf')
    for label,folder in [('normal',normal),('diagnostic',debug)]:
        sec=audit.sections(folder/'CH32V00X_normal.elf');assert sec['.fw_info']['data']==old['.fw_info']['data']
        symbols=s.run([b.WCH/'riscv-wch-elf-nm.exe',folder/'CH32V00X_normal.elf'],OUT/(label+'_symbols.log'))
        assert not re.search(r'\b__(?:add|sub|mul|div)(?:df|sf)3\b|\b(?:sqrt|pow|lround)\b',symbols)
        if label=='diagnostic':assert 'Task_Print' in symbols
    assert (normal/'CH32V00X_normal.bin').read_bytes()==(OUT/'lto/candidate/CH32V00X_normal.bin').read_bytes()
    for ext in ('bin','hex','elf','map'):shutil.copy2(normal/('CH32V00X_normal.'+ext),PROJECT/'obj'/('CH32V00X_APP.'+ext))
    with (PROJECT/'obj/CH32V00X_APP.lst').open('w',encoding='utf-8') as f:
        p=subprocess.run([str(b.WCH/'riscv-wch-elf-objdump.exe'),'-S','-d','-M','xw',str(PROJECT/'obj/CH32V00X_APP.elf')],stdout=f,stderr=subprocess.PIPE,text=True);assert p.returncode==0,p.stderr
    files=[PROJECT/'obj/CH32V00X_APP.bin',PROJECT/'obj/CH32V00X_APP.hex',debug/'CH32V00X_normal.bin',debug/'CH32V00X_normal.hex']
    result={str(p.relative_to(ROOT)):{'bytes':p.stat().st_size,'sha256':digest(p)} for p in files}
    (OUT/'firmware.json').write_text(json.dumps(result,indent=2));scope();print(json.dumps(result,indent=2),flush=True)
if __name__=='__main__':
    if '--apply' in sys.argv:apply()
    else:package()
