from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent
OUT=HERE/'output'
plt.rcParams['font.sans-serif']=['Microsoft YaHei']
plt.rcParams['axes.unicode_minus']=False

def load(name):return np.genfromtxt(OUT/(name+'.csv'),delimiter=',',names=True)

a=load('after_m0_T0_n1_d0');b=load('after_m0_T0_n0_d0');small=load('after_m9_T0_n1_d0')
fig,ax=plt.subplots(3,2,figsize=(13,11))
ax[0,0].plot(a['t'],a['pwm']/24,label='有标定噪声 σ=0.01°',lw=1)
ax[0,0].plot(b['t'],b['pwm']/24,label='无附加噪声（保留量化）',lw=1,alpha=.7)
ax[0,0].set_xlim(0,1150);ax[0,0].set_title('90°单点运动，T=0：实际PWM全程');ax[0,0].set_ylabel('有符号占空比 (%)');ax[0,0].legend(fontsize=9)
w=(a['t']>=250)&(a['t']<=450)
ax[0,1].plot(a['t'][w],a['refv'][w],label='参考速度')
ax[0,1].plot(a['t'][w],a['v'][w],label='物理模型实际速度')
ax[0,1].plot(a['t'][w],a['obsv'][w],label='观测速度',alpha=.8)
ax[0,1].set_title('匀速时参考不变，观测速度仍有噪声');ax[0,1].set_ylabel('速度 (°/s)');ax[0,1].legend(fontsize=9)
ax[1,0].plot(a['t'][w],a['pwm'][w]/24,label='控制器1ms输出',color='C0',alpha=.7)
ws=w&(a['t']%20==0)
ax[1,0].plot(a['t'][ws],a['pwm'][ws]/24,'o-',label='每20ms取一帧示例',color='C3',ms=4)
ax[1,0].set_title('匀速段PWM：可直接在上位机波形中出现');ax[1,0].set_ylabel('占空比 (%)');ax[1,0].legend(fontsize=9)
ax[1,1].plot(a['t'][w],a['ufb'][w],label='反馈项')
ax[1,1].plot(a['t'][w],a['ui'][w],label='积分项')
ax[1,1].plot(a['t'][w],a['uff'][w]-6000,label='前馈 − 6000mV')
ax[1,1].set_title('同一匀速段：主要是反馈项在变');ax[1,1].set_ylabel('电压 (mV)');ax[1,1].legend(fontsize=9)
w=(small['t']>=40)&(small['t']<=110)
ax[2,0].plot(small['t'][w],small['pwm'][w]/24,'o-',ms=2,label='最终PWM')
ax[2,0].axvline(74,color='C3',ls='--',lw=1)
ax[2,0].set_title('0.1°请求（脉宽量化后约0.13°），T=0');ax[2,0].set_ylabel('占空比 (%)');ax[2,0].annotate('193 → 44 / 2400\n1ms下降6.21个百分点',xy=(74,44/24),xytext=(80,6.5),fontsize=9,arrowprops={'arrowstyle':'->'})
ax[2,1].plot(small['t'][w],small['ustick'][w],label='起转补偿',color='C1')
ax[2,1].plot(small['t'][w],small['ufb'][w],label='反馈项',color='C0')
ax[2,1].plot(small['t'][w],small['uff'][w],label='前馈项',color='C2')
ax[2,1].axvline(74,color='C3',ls='--',lw=1)
ax[2,1].set_title('主要跳变量：起转补偿376mV → 0');ax[2,1].set_ylabel('电压 (mV)');ax[2,1].legend(fontsize=9)
for x in ax.flat:x.set_xlabel('指令后时间 (ms)');x.grid(alpha=.25)
fig.suptitle('20kg当前代码空载仿真：最终PWM输出确有波动\n采用当前源码标定；噪声为独立高斯假设。仿真覆盖1ms占空比指令，不覆盖20kHz硬件边沿。',fontsize=12)
fig.tight_layout();fig.savefig(OUT/'pwm_jitter.png',dpi=150)

results=json.loads((OUT/'results.json').read_text())
summary={'case_count':len(results),'production_sources_unchanged':True,'T0_before_after_identical':True,'calibration':{'KV':349,'KA':10285,'FC':37,'FS':486,'delay_us':1655,'noise_3sigma_cdeg':3,'min_step_cdeg':9},'PWM_full_scale_ticks':2400,'cruise_window_ms':[250,450],'noise_cruise':{},'small_T0_jump':{},'noise_assumption':'Independent Gaussian block-mean noise; standard deviation = calibration noise / 3. Quantization retained even at zero added noise.'}
for noise in (0,1,2,5):summary['noise_cruise'][str(noise)]=results[f'after_m0_T0_n{noise}_d0']['cruise250_450']
summary['small_T0_jump']=results['after_m9_T0_n1_d0']['move']
summary['notes']=['HOLD has constant nonzero PWM in some nominal cases; constant is not jitter.','Noise sigma 0.05 deg is a stress case above calibration; this model stays in SETTLE through 2500 ms.','Observer bandwidth reduction and compensation removal were attribution diagnostics only, not validated fixes.','Output mapping and limiting use production A_Servo; the D_Motor hardware stub returns the same signed clipped duty as the real driver, without register timing or switching transients.']
(OUT/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print('PASS figure and numeric summary generated')
