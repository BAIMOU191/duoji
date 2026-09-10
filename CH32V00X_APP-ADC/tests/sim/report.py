import csv,json,pathlib,hashlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from run import HERE,ROOT
OUT=HERE/'results'
summary=json.loads((OUT/'summary.json').read_text());sweep=json.loads((OUT/'sweep.json').read_text())
def rows(v,m):
    with (OUT/f'{v}_{m}.csv').open() as f:return [{k:float(x) for k,x in r.items()} for r in csv.DictReader(f)]
stress={}
for v,cases in sweep.items():
    quiet=0;last_drive=0
    for i,case in enumerate(cases):
        data=rows(v,f'cycle_sweep{i}')
        for start in range(0,12000,3000):
            b=data[start:start+3000]
            quiet+=not any(r['pwm'] for r in b[-100:])
            active=[k for k,r in enumerate(b) if r['pwm']]
            if active:last_drive=max(last_drive,active[-1])
            if v=='fixed':
                assert all(-1000<=r['pos']<=28000 and abs(r['pwm'])<=2400 for r in b)
                assert not any(r['pwm'] for r in b[-100:])
    stress[v]={'cases':len(cases),'moves':len(cases)*4,'quiet_moves':quiet,'last_drive_ms':last_drive,
               'max_overshoot_deg':max(m['overshoot'] for r in cases for m in r['moves'])/100,
               'max_post_done_travel_deg':max(m['post_done_travel'] for r in cases for m in r['moves'])/100,
               'max_final_error_deg':max(abs(m['final_error']) for r in cases for m in r['moves'])/100}
(OUT/'stress_summary.json').write_text(json.dumps(stress,indent=2))
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(3,2,figsize=(12,10),constrained_layout=True)
for v,color in [('baseline','#d55e00'),('fixed','#0072b2')]:
    for row,mode in enumerate(['cycle','jitter','boot-low']):
        data=rows(v,mode)
        if row==0:data=[r for r in data if 9700<=r['ms']<=10400]
        if row==1:data=[r for r in data if r['ms']>=10000]
        t=[r['ms']/1000 for r in data]
        p=[(r['pos']-(20250 if row==0 else 13500 if row==1 else 0))/100 for r in data]
        axes[row,0].plot(t,p,label=v,color=color,lw=1.4)
        axes[row,1].plot(t,[r['pwm'] for r in data],label=v,color=color,lw=1)
for row,title in enumerate(['Large move: final approach to 2000 us','Fixed 1500 us input with +/-3 us jitter','Boot below potentiometer low edge']):
    axes[row,0].set_title(title);axes[row,1].set_title('Applied motor PWM')
    axes[row,0].set_ylabel('Position error (deg)' if row<2 else 'Position (deg)')
    axes[row,1].set_ylabel('PWM counts')
    for ax in axes[row]:ax.set_xlabel('Time (s)');ax.grid(alpha=.2);ax.legend()
fig.suptitle('Host simulation of production C firmware — model results, not hardware measurements',fontsize=13)
fig.savefig(OUT/'servo_comparison.png',dpi=160);plt.close(fig)
a=summary['baseline'];b=summary['fixed'];sa=stress['baseline'];sb=stress['fixed']
report=f'''# 舵机到位回摆、静止噪声及低端死区修复

日期：2026-09-09。已修改工作区源码并生成固件；没有连接实机烧录。本报告中的角度与速度均沿用工程的电位器标定尺度。

## 原因与修复

1. **500侧死区往复是确定的坐标错误。** 电位器可输出负角度，原舵机层却把它强转为uint16_t。例如-500厘度变成65036，行程上报又投影到27000；观测器和位置环仍强制按36000回绕。已有CFG_WRAP_RANGE_CDEG=0参数原先没有在算法中使用。现在反馈全程保留int32_t符号，并在电位器模式关闭回绕；目标/上报才夹到行程范围。磁编码器仍保留圆周处理，新增编译期一致性检查。

2. **静止PWM输入会反复重规划。** 原32厘度目标死区约相当于2.37us；±3us的输入抖动足以跨过它，轨迹反复被打断，保持静音来不及生效。加入三帧中值去除孤立毛刺，以及仅对PWM输入生效的6us目标变化门限；比较基准是上次接受的目标，小幅连续移动会累积后生效。端点先夹到真实规划范围，避免几个不同脉宽实际指向同一端点却重复规划。

3. **到位后的细小修正缺少静音滞环，低于起转力的输出还会继续发。** 保留32厘度运动修正死区，把静音捕获窗适度放宽至48厘度，退出窗为80厘度。静止补偿门槛从起转标定值的3/4提高到完整235 PWM，并保留运动时的小输出制动。静音锁存时清除残余积分；真实持续负载下保留闭环与保持力。末端低速、仍在捕获窗外时加快积分建立，缩短摩擦变大后等待最后一步的时间。

4. **最快大角度动作对惯性/延迟误差敏感。** 相对速度预测从1ms改为8ms；减速平滑度25→60，减速加速度比例189/256→128/256。最大速度仍为84%，T=0仍表示可实现的最快规划。标称整段往复运动规划时间增加约30ms，给末端制动留出余量。

5. **附带修复。** 暂停保留原目的地；脱困期间保留最新目标和时间，普通运动脱困后恢复原目标；脱困所有拍均计入超时，有效反馈出现后根据物理所在端选择内推方向并确认进入测量安全区；脱困输出遵守电机安装方向。中位标定拒绝把完整行程移出电位器可测范围。MounRiver工程明确排除tests目录，防止测试桩和历史代码参加固件编译。

## 仿真方法

直接编译工程的A_Servo、A_Sensor、C_Traj_Planner、C_Speed_Observer、C_Pos_Ctrl，替换ADC、输入捕获和电机硬件接口；不是另外写一套控制器。物理模型使用已记录的斜率4716、惯性52ms、延迟2ms、动摩擦57，以及正反向起转门槛180/236；加入Q4 ADC量化和随机测量噪声。

标称序列先从中位到500，然后2000→500→2000，每3秒发一次，所有指令T=0。500的实际目标保留原有2°保护余量。另测每450ms提前反向、静止PWM抖动、两端死区上电、3°外力位置扰动、300 PWM等效恒定负载及卸载。

压力测试为速度增益0.7/1.0/1.3、惯性0.7/1.0/1.3、摩擦1/1.5倍、延迟1/4ms的36种组合，每种4次动作，共144次；位置噪声±9厘度，均使用固定随机种子。压力测试覆盖范围不等于实机的全部工作条件。

## 对比结果

| 项目 | 修改前 | 修改后 |
|---|---:|---:|
| 标称4次动作最大越过目标量 | {max(m['overshoot'] for m in a['cycle']['moves'])/100:.3f}° | {max(m['overshoot'] for m in b['cycle']['moves'])/100:.3f}° |
| 标称轨迹结束后最大累计位移 | {max(m['post_done_travel'] for m in a['cycle']['moves'])/100:.3f}° | {max(m['post_done_travel'] for m in b['cycle']['moves'])/100:.3f}° |
| 标称每次动作最后1秒驱动输出 | 全部为0 | 全部为0 |
| ±3us输入抖动：最后1秒轴摆幅 | {a['jitter']['tail_motion_pp']/100:.3f}° | {b['jitter']['tail_motion_pp']/100:.3f}° |
| ±3us输入抖动：最后1秒非零驱动拍数 | {a['jitter']['tail_nonzero_pwm']}/1000 | {b['jitter']['tail_nonzero_pwm']}/1000 |
| 低端上电：12秒内脱困进入次数 | {a['boot-low']['escape_entries']} | {b['boot-low']['escape_entries']} |
| 高端上电：12秒内脱困进入次数 | {a['boot-high']['escape_entries']} | {b['boot-high']['escape_entries']} |
| 144次压力动作：最后100ms全程无驱动 | {sa['quiet_moves']}/144 | {sb['quiet_moves']}/144 |
| 压力测试最大越过目标量 | {sa['max_overshoot_deg']:.3f}° | {sb['max_overshoot_deg']:.3f}° |
| 压力测试最晚仍有驱动的时刻（指令后） | {sa['last_drive_ms']}ms | {sb['last_drive_ms']}ms |

“轨迹结束”指规划器done，不表示实物已完全停住；累计位移不等同于单次回摆幅度。修改前压力测试的最大越位还叠加了负角度坐标错误，不能把全部改进归因于减速调参。

持续负载下修改后最终误差{abs(b['load']['final_pos']-13500)/100:.3f}°，最后1秒无位置摆动，但仍输出保持力；这类工况不应承诺电机完全无电流、无声音。卸载后测试恢复零输出。

![仿真对比]({(OUT/'servo_comparison.png').as_posix()})

## 验证、产物与边界

- 回归断言通过：负角度链路、超过180°的线性误差方向、负位置观测器、暂停/同目标继续、中位/自定义行程标定、脱困新旧指令保留、反馈间歇恢复时总超时、PWM毛刺过滤/真实阶跃、输出限幅。
- 电机安装方向-1运行同一回归测试通过；反向位置模式、磁编码器模式分别构建并检查到位和最终静止。
- WCH GCC12已完整编译并链接当前工程，Flash占用28416字节，静态RAM占用3184字节，另预留2048字节栈。仅有原调试库的fd未使用参数警告。新HEX/BIN已同步到工程obj目录，原生成文件备份在results/original_build。宿主UBSan检查因本机MinGW没有libubsan不能链接，不计为已通过检查。
- 6us过滤在270°模式对应最多约0.81°的未接受目标变化；持续小步会累积。三帧中值使正常PWM阶跃通常增加一帧约20ms延迟。串口目标仍使用32厘度门限。
- 保持捕获0.48°、退出0.80°是阈值，不是全工况精度保证。惯性和摩擦偏差下仍可能出现短暂越位：本次压力测试最大{sb['max_overshoot_deg']:.2f}°。代码错误已修复，真实机构的齿隙、弹性回弹、负载变化、电源和ADC时序需烧录后复核；仿真不能证明肉眼回摆与声音在所有工况下消失。

复现：在工程根目录依次运行 `python tests/sim/run.py baseline fixed`、`tests/sim/results/fixed.exe regression`、`python tests/sim/check_variants.py`、`python tests/sim/sweep.py`、`python tests/sim/report.py`。完整固件重建使用 `python tests/sim/build_firmware.py`，避免旧obj/makefile中的绝对路径指向另一份工程。

建议实机先空载验证500→2000→500、T=0多轮，再测PWM固定输入、两端死区上电和轻载保持。对比ref_pos/meas_pos/obs_vel/PWM，确认末端不再因噪声重复启动；若只在带载时有声，应区分必要保持力与反复修正。
'''
(HERE/'REPORT.md').write_text(report,encoding='utf-8')
hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'Application/A_Servo.c',ROOT/'Application/A_Parameter.h',ROOT/'Application/A_Sensor.h',ROOT/'Common/C_Pos_Ctrl.c',ROOT/'Common/C_Speed_Observer.c',ROOT/'CH32V00X_APP.wvproj']}
(OUT/'source_hashes.json').write_text(json.dumps(hashes,indent=2))
print(json.dumps(stress,indent=2))
