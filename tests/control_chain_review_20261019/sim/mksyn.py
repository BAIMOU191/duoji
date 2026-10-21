import shutil,os,re,sys,subprocess
SRC='/c/Users/Admin/Desktop/duoji/CH32V00X_APP_ADC_30kg'
# Windows path for python
SRCW='C:/Users/Admin/Desktop/duoji/CH32V00X_APP_ADC_30kg'
cases={
 'fast_tau12':   dict(KV=300,KA=3600,FC=100,FS=300,DELAY=1500,NOISE=6,STEP=10,VEL=230,ACC=15000,DEC=15000),
 'slow_tau45':   dict(KV=250,KA=11250,FC=200,FS=500,DELAY=2500,NOISE=6,STEP=12,VEL=260,ACC=5000,DEC=5000),
 'high_fric':    dict(KV=397,KA=7063,FC=450,FS=1000,DELAY=3211,NOISE=6,STEP=20,VEL=150,ACC=7000,DEC=7000),
 'noisy_pot':    dict(KV=397,KA=7063,FC=127,FS=413,DELAY=3211,NOISE=14,STEP=13,VEL=162,ACC=7500,DEC=7500),
 'delay4500':    dict(KV=397,KA=7063,FC=127,FS=413,DELAY=4500,NOISE=6,STEP=13,VEL=162,ACC=7500,DEC=7500),
 'minstep40':    dict(KV=397,KA=7063,FC=127,FS=413,DELAY=3211,NOISE=6,STEP=40,VEL=162,ACC=7500,DEC=7500),
 'minstep34':    dict(KV=397,KA=7063,FC=127,FS=413,DELAY=3211,NOISE=6,STEP=34,VEL=162,ACC=7500,DEC=7500),
 'fast_acc30k':  dict(KV=397,KA=2400,FC=127,FS=413,DELAY=2000,NOISE=6,STEP=13,VEL=162,ACC=30000,DEC=30000),
}
names={'KV':'MOTOR_KV_UV','KA':'MOTOR_KA_NV','FC':'MOTOR_FC_MV','FS':'MOTOR_FS_MV','DELAY':'MOTOR_DELAY_US','NOISE':'MOTOR_NOISE_CDEG','STEP':'MOTOR_MIN_STEP_CDEG','VEL':'MOTOR_RATED_VEL_DPS','ACC':'MOTOR_RATED_ACC_DPS2','DEC':'MOTOR_RATED_DEC_DPS2'}
for nm,c in cases.items():
    dst=os.path.join('syn',nm)
    if os.path.exists(dst): shutil.rmtree(dst)
    for d in ['Application','Common','Drivers','System','SRC/Debug']:
        shutil.copytree(SRCW+'/'+d,os.path.join(dst,d))
    p=os.path.join(dst,'Application','A_Config.h')
    s=open(p,encoding='utf-8').read()
    for k,v in c.items():
        s=re.sub(r'(#define %s\s+)\d+'%names[k], lambda m: m.group(1)+str(v), s)
    open(p,'w',encoding='utf-8').write(s)
print('ok')
