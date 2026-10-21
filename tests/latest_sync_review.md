# 2026-10-16 最新同步核对

以当前30kg版本为基准同步其余四个版本；保留各型号硬件标识、旋向、电机标定与传感器几何参数。30kg基准代码未修改。

- 保持P默认30，单位‰/°；保持I默认0，单位‰/(°·s)；两者上限100。
- 千分比相对于本机标定电压；保持窗与到位窗共用，保持D和保持齿隙地址均已移除。
- 上电位置规划请求仍为1500ms，IIC保留磁编码器和多圈功能。
- 18组主机仿真、10项目标编译通过；恢复默认指令与保存回归通过；只有厂商debug.c现有fd未使用警告。
- 参数总表核对53个可读地址、35个普通可写参数及19个二进制示例通过，重复生成无变化。
- ADC 20kg_single与IIC 20kg生产标定尚未完整，使用隔离测试模型完成编译和仿真；未把测试标定写入生产源码。

本次备份：`C:\Users\Admin\Desktop\duoji\tests\alignment_output\before_resync_20261016_170122.zip`

## CH32V00X_APP-IIC_20kg

- Application/A_Config.h
- Application/A_Servo.c
- Application/A_UartCmd.c
- Common/C_Pos_Ctrl.c
- Common/C_Pos_Ctrl.h

## CH32V00X_APP_ADC_15kg

- Application/A_Config.h
- Application/A_Servo.c
- Application/A_UartCmd.c
- Common/C_Pos_Ctrl.c
- Common/C_Pos_Ctrl.h

## CH32V00X_APP_ADC_20kg

- Application/A_Config.h
- Application/A_Servo.c
- Application/A_UartCmd.c
- Common/C_Pos_Ctrl.c
- Common/C_Pos_Ctrl.h

## CH32V00X_APP_ADC_20kg_single

- Application/A_Config.h
- Application/A_Servo.c
- Application/A_UartCmd.c
- Common/C_Pos_Ctrl.c
- Common/C_Pos_Ctrl.h

## CH32V00X_APP_ADC_30kg

基准源码未变动。

