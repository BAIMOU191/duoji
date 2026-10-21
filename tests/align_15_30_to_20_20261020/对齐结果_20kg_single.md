# 补充：CH32V00X_APP_ADC_20kg_single 同步对齐（2026-10-20）

按同一规则把 `CH32V00X_APP_ADC_20kg_single` 也对齐到 20kg 基准：只保留型号/电机标定参数，其余照搬。
20kg 基准源码零改动。

## 一、保留的标定参数（原值未动）

| 项目 | single | 20kg(基准) |
|---|---:|---:|
| `FW_TORQUE` | 20U | 20U |
| `MOTOR_CAL_MV` / `MOTOR_KV_UV` / `MOTOR_KA_NV` | 0 / 0 / 0 | 7683 / 349 / 10285 |
| `MOTOR_FC_MV` / `MOTOR_FS_MV` | 0 / 0 | 37 / 486 |
| `MOTOR_DELAY_US` | 2000 | 1655 |
| `MOTOR_NOISE_CDEG` / `MOTOR_MIN_STEP_CDEG` | 0 / 40 | 3 / 9 |
| `MOTOR_SIGN` | **-1** | 1 |
| `MOTOR_RATED_VEL/ACC/DEC` | 0 / 0 / 0 | 191 / 6000 / 5000 |

即 single 的生产标定**尚未完成**（多数 `MOTOR_*` 为 0），旋向为 `-1`，最小可靠位移 40 厘度。全部原样保留。

## 二、对齐掉的差异（非标定，共 22 个源码/工程文件）

- **新增 4 个模块**：`A_Debug.c/.h`、`A_Servo_Ctrl.c/.h`、`A_StatusLed.c/.h`（20kg 的模块拆分，single 原先缺失）
- **旧版实现换新**：`A_Servo.c/.h`、`A_Tasks.c`、`A_UartCmd.c`、`A_Config.c`、`A_Protect.c/.h`、
  `Common/C_Pos_Ctrl.c/.h`、`Common/C_Traj_Planner.c/.h`、`User/main.c`
- **`A_Config.h` 第 3/4/5 节**：`TUNE_OBS_BW_HZ` 21→16、`TUNE_VEL_BW_RADS` 120→70、
  `TUNE_POS_BW_RADS` 固定 20→按 ω_v/4 收敛、补回缺失的 `TUNE_LOAD_RATIO_X10=20U`、
  `CFG_DEF_HOLD_KP` 30→80、`CFG_DEF_AMAX/DMAX` 80%→70%、`CFG_DEF_SMOOTH_MS` 0→50、
  `CFG_DEF_MOVE_MIN_MS` 120→150、`CFG_SMOOTH_MIN_MS` 公式→`1U`
- **构建清单**：`obj/*/subdir.mk` 补上 `A_Debug / A_Servo_Ctrl / A_StatusLed`
- **工程描述**：`.cproject` / `CH32V00X_APP.wvproj` 补上基准的 `-flto` 编译与链接选项（single 原先没有）

## 三、⚠️ single 仍无法出固件 —— 对齐前就如此

`A_Config.h` 的编译期标定断言在 `MOTOR_*=0` 时必然失败，**对齐前后完全一样**：

| 断言 | 对齐前 | 对齐后 |
|---|---|---|
| `Invalid motor calibration` | 失败 | 失败 |
| `Invalid default motion/hold parameters` | 失败 | 失败 |
| `Invalid smoothing time range`（`MOTOR_KV_UV=0` 除零） | 失败 | 失败 |

用 `A_Calib_Min` 标定固件实测结果替换 `A_Config.h` 第 2 节后即可正常出固件。
我没有把任何测试/假设标定值写入生产源码（本项目既有约定）。

合成标定的隔离测试副本验证了对齐后的代码本身可正常编译链接（正常/标定两种模式均通过）：

| 模式 | text | data | bss |
|---|---:|---:|---:|
| 合成标定 正常 | 38156 | 128 | 3728 |
| 合成标定 标定 | 43256 | 212 | 5192 |

与 20kg 基准完全相同 —— 因为测试副本借用了基准标定，反过来证明三机差异只来自标定常数。

## 四、验证（`final_check.py` 全 7 项 PASS）

| 检查项 | 15kg | 30kg | 20kg_single |
|---|:--:|:--:|:--:|
| [3] 除 `A_Config.h` 外源码逐字节一致 | 是 | 是 | 是 |
| [4] `A_Config.h` 归一化标定值后与 20kg 一致 | 是 | 是 | 是 |
| [5] 型号/电机标定 13 项原值保留 | 是 | 是 | 是 |
| [6] 构建清单与 20kg 一致 | 是 | 是 | 是 |
| [7] IDE/工程描述文件与 20kg 一致 | 是 | 是 | 是 |

## 五、产物

- 对齐前备份：`backup_before_align/CH32V00X_APP_ADC_20kg_single/`
- 测试副本：`output/build/CH32V00X_APP_ADC_20kg_single_synthetic_model_TARGET_CHECK_ONLY/`
- 编译日志：`output/build/CH32V00X_APP_ADC_20kg_single/`、`output/build/single_synthetic_build/`
- 脚本：`build_single_synthetic.py`（合成标定编译）、`udiff.py`、`bytecmp_single.py`

## 六、其它

`CH32V00X_APP_ADC_20kg/obj` 下 50 个 `.o/.map` 构建产物在本次工作期间被外部重新构建过
（非我所为，应为 MRS 工程构建产生）。**20kg 生产源码零改动**，`final_check.py` [1] 已按"源码 vs 构建产物"
分开核对。
