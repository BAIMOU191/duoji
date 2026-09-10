import pathlib,shutil,subprocess,json
from run import ROOT,HERE,build,run
report={}
for name,circular in [('motor_negative',False),('magnetic',True)]:
    dest=HERE/'results'/name
    for folder in ['Application','Common']:
        (dest/folder).mkdir(parents=True,exist_ok=True)
        for source in (ROOT/folder).glob('*'):
            if source.suffix in ['.c','.h']:shutil.copy2(source,dest/folder/source.name)
    p=dest/'Application/A_Parameter.h';s=p.read_text(encoding='utf-8')
    if circular:
        s=s.replace('#define CFG_WRAP_RANGE_CDEG 0 ', '#define CFG_WRAP_RANGE_CDEG 36000 ')
        sensor=dest/'Application/A_Sensor.h';sensor.write_text(sensor.read_text(encoding='utf-8').replace('#define ENCODER_MODE    0','#define ENCODER_MODE    1'),encoding='utf-8')
    else:s=s.replace('#define CAL_MOTOR_SIGN 1 ', '#define CAL_MOTOR_SIGN -1 ')
    p.write_text(s,encoding='utf-8');build(name,dest)
    if not circular:subprocess.run([str(HERE/'results'/f'{name}.exe'),'regression'],check=True)
    report[name]={m:run(name,m) for m in (['cycle'] if circular else ['cycle','boot-low','boot-high'])}
    assert abs(report[name]['cycle']['final_pos']-20250)<80
    assert report[name]['cycle']['tail_nonzero_pwm']==0
    if not circular:
        assert report[name]['boot-low']['escape_entries']==1 and report[name]['boot-high']['escape_entries']==1
        assert report[name]['boot-low']['tail_nonzero_pwm']==report[name]['boot-high']['tail_nonzero_pwm']==0
report['reverse_position']=run('fixed','cycle',(1,1,1,4.5,2,2),'_reverse')
assert abs(report['reverse_position']['final_pos']-6750)<80
assert report['reverse_position']['tail_nonzero_pwm']==0
(HERE/'results'/'variants.json').write_text(json.dumps(report,indent=2))
print('Variant builds and negative motor direction regression passed.')
