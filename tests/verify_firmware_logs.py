"""执行真实标定固件状态机，再回放Python辨识；与被控对象的已知参数对照。"""
import json
import subprocess
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
import calibrate_model as model

root=Path(sys.argv[1]).resolve()
result={}
for name,ripple,ka,delay,brake in (
    ('nominal',0,.0098,2,1),('ripple',15,.0098,2,1),
    ('other',0,.014,4,1),('weak_brake',0,.0098,2,.5)):
    logs=[]
    for voltage in (6000,8000):
        run=subprocess.run([str(root/'calibration_firmware_sim.exe'),str(voltage),str(ripple),str(ka),str(delay),str(brake)],capture_output=True)
        (root/f'{name}_{voltage}.log').write_bytes(run.stdout)
        if run.returncode: raise RuntimeError(run.stderr.decode(errors='replace'))
        logs.append(model.parse_result(run.stdout.decode('utf-8')))
    values,notes=model.calculate(*logs)
    assert abs(values['MOTOR_KV_UV']-402)/402<.05
    assert abs(values['MOTOR_KA_NV']-ka*1e6)/(ka*1e6)<.12,values
    if ripple==0: assert abs(values['MOTOR_DELAY_US']-delay*1000)<=500,values
    assert values['MOTOR_CAP_V1_VEL_DPS']<values['MOTOR_CAP_V2_VEL_DPS']
    result[name]=dict(values=values,notes=notes)
    (root/f'{name}_model.h').write_text(model.header(values,*logs),encoding='utf-8')
    print(name,{k:values[k] for k in ('MOTOR_KV_UV','MOTOR_KA_NV','MOTOR_DELAY_US','MOTOR_CAP_V2_DEC_DPS2')},flush=True)
assert result['weak_brake']['values']['MOTOR_CAP_V2_DEC_DPS2']<result['nominal']['values']['MOTOR_CAP_V2_DEC_DPS2']
(root/'verification.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
print('PASS: firmware state machine + offline identification, 4 plants x 2 voltages')
