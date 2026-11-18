/* servo_drv.h —— 总线舵机主控端驱动库（二进制协议）
 *
 * 适用：电位器版 / 磁编码版串口舵机，单线半双工 UART，8N1，默认 115200。
 * 帧格式：AA 66 | ID | CMD | LEN | DATA | CRC8 | 66 AA，CRC-8/ATM(多项式 07，初值 00)，范围 ID~DATA 末字节。
 * 特点：纯 C99、无 malloc、不依赖芯片；移植只需填 Servo_Port_t 里的几个回调（见 servo_port_example.c）。
 *
 * 分层：
 *   1) 底层(不阻塞)：Servo_Crc8 / Servo_BuildFrame / Servo_ParseByte —— DMA、RTOS 自己调度收发时用
 *   2) 通用读写(阻塞带超时)：Servo_Transfer / Servo_ReadParam / Servo_WriteParam
 *   3) 运动控制、状态读取、设备信息、参数设置、校准与系统动作
 *
 * 返回值：int 型。0 = 成功；1~7 = 舵机应答的状态码(SERVO_ST_*)；负数 = 驱动自身错误(SERVO_ERR_*)。
 * 广播(ID=0xFF)：舵机延时 100ms+ID×2ms 应答，驱动发完后等够这段时间、丢弃所有应答，然后返回 0；
 *   广播读没有意义(会收到多份)，驱动拒绝；舵机本身允许广播修改 ID 和全部恢复默认，本驱动为防误操作也拒绝。
 */
#ifndef SERVO_DRV_H
#define SERVO_DRV_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== 常量 ============================== */

#define SERVO_BROADCAST_ID     0xFFU   /* 广播ID */
#define SERVO_ID_MAX           254U    /* 单播ID上限 */
#define SERVO_DATA_MAX         32U     /* 一帧 DATA 最大字节数 */
#define SERVO_FRAME_MAX        (8U + SERVO_DATA_MAX)   /* 一帧最大字节数 */
#define SERVO_DEFAULT_TIMEOUT  50U     /* 默认应答超时 ms(9600 波特率下也够读 C9) */

/* 位置码：0~4095 对应 500~2500us(2048≈1500us) */
#define SERVO_POS_MAX          4095U
#define SERVO_US_TO_POS(us)    ((uint16_t)((((uint32_t)(us) - 500U) * 4095U + 1000U) / 2000U))   /* us 须在 500~2500 */
#define SERVO_POS_TO_US(p)     ((uint16_t)(500U + ((uint32_t)(p) * 2000U + 2047U) / 4095U))

/* 命令字 */
#define SERVO_CMD_STATE        0x01U   /* 查询简要状态 */
#define SERVO_CMD_READ         0x02U   /* 读参数 */
#define SERVO_CMD_WRITE        0x03U   /* 写参数 */
#define SERVO_CMD_SLAVE        0x04U   /* 主/从：DATA 1B，0=主(正常应答) 1=从(不应答，存Flash) */
#define SERVO_CMD_RESET_KEEP   0x07U   /* 恢复默认(保留ID) */
#define SERVO_CMD_RESET_ALL    0x08U   /* 全部恢复默认 */

/* 舵机应答状态码 */
#define SERVO_OK               0       /* 成功 */
#define SERVO_ST_CMD           1       /* 命令不存在 */
#define SERVO_ST_LENGTH        2       /* 数据长度错误 */
#define SERVO_ST_VALUE         3       /* 参数值越界 */
#define SERVO_ST_STATE         4       /* 当前状态不允许 */
#define SERVO_ST_FAILED        5       /* 操作失败 */
#define SERVO_ST_READONLY      6       /* 只读地址 */
#define SERVO_ST_UNSUP         7       /* 功能未开放 */

/* 驱动自身错误 */
#define SERVO_ERR_TIMEOUT      (-1)    /* 超时无应答 */
#define SERVO_ERR_REPLY        (-2)    /* 应答格式/长度不符 */
#define SERVO_ERR_ARG          (-3)    /* 参数越界或指针为空，帧没发出 */
#define SERVO_ERR_BCAST        (-5)    /* 该操作不允许广播 */

/* ============================== 地址 ============================== */

/* 设备信息(只读) */
#define SERVO_ADDR_HW_TYPE       0x00U  /* 舵机类型 1B：1/3 电位器，2/4 磁编码 */
#define SERVO_ADDR_HW_VERSION    0x01U  /* 硬件版本 1B，0.1版 */
#define SERVO_ADDR_HW_VOLTAGE    0x02U  /* 标称电压 1B，V */
#define SERVO_ADDR_HW_TORQUE     0x03U  /* 扭力型号 1B，kg */
#define SERVO_ADDR_HW_VMAX       0x04U  /* 额定最大速度 2B，°/s */
#define SERVO_ADDR_HW_AMAX       0x05U  /* 额定最大加速度 2B，°/s² */
#define SERVO_ADDR_HW_DMAX       0x06U  /* 额定最大减速度 2B，°/s² */
#define SERVO_ADDR_HW_SMOOTH     0x07U  /* 自动平滑时间 1B，ms */
#define SERVO_ADDR_HW_MIN_STEP   0x08U  /* 最小可靠位移 1B，0.01° */
#define SERVO_ADDR_FW_VERSION    0x10U  /* 固件版本 3B：主/次/修订 */
/* 动作命令(只写) */
#define SERVO_ADDR_MOVE          0x80U  /* 运动目标：位置码2B + 时间2B */
#define SERVO_ADDR_MOTION        0x81U  /* 运动动作：1继续 2暂停 3停止 */
#define SERVO_ADDR_TORQUE_MODE   0x82U  /* 扭矩模式 1B：0释放 1正常 2自定义阻尼(RAM) */
#define SERVO_ADDR_DAMPING       0x83U  /* 阻尼比例 1B，0~100%(RAM) */
#define SERVO_ADDR_CALIB         0x8BU  /* 校准动作 1B，见 Servo_Calib_t */
#define SERVO_ADDR_SYSTEM        0x8CU  /* 系统动作 1B：1重载RAM参数 2恢复默认保留ID 3全部恢复默认 */
#define SERVO_ADDR_MOVE_TURNS    0x90U  /* 多圈运动：位置码2B + 整圈数1B + 每圈时间2B(仅磁编码版) */
#define SERVO_ADDR_CAL_TRIM      0x91U  /* 中点微调 int16 us(±999)：转到1500+n，到位后实测位置记为新中点 */
/* 实时状态(只读) */
#define SERVO_ADDR_WORK_STATE    0xC0U  /* 工作状态 1B，见 Servo_WorkState_t */
#define SERVO_ADDR_FAULT         0xC1U  /* 故障位掩码 2B，见 SERVO_FAULT_* */
#define SERVO_ADDR_POSITION      0xC2U  /* 当前位置 2B，位置码 */
#define SERVO_ADDR_SPEED         0xC3U  /* 速度 int16，0.1°/s */
#define SERVO_ADDR_TEMPERATURE   0xC4U  /* 温度 int16，0.1℃ */
#define SERVO_ADDR_VOLTAGE       0xC5U  /* 电压 2B，0.01V */
#define SERVO_ADDR_CURRENT       0xC6U  /* 电流 int16，mA */
#define SERVO_ADDR_TORQUE        0xC7U  /* 扭矩 int16，% */
#define SERVO_ADDR_STALLED       0xC8U  /* 堵转状态 1B：1=堵转保持中 */
#define SERVO_ADDR_TELEMETRY     0xC9U  /* 七项状态集合 14B */

/* 故障位(C1 / CMD01) */
#define SERVO_FAULT_OVERTEMP     0x0001U  /* 过温 */
#define SERVO_FAULT_OVERCUR      0x0002U  /* 过流 */
#define SERVO_FAULT_UNDERVOLT    0x0008U  /* 欠压 */
#define SERVO_FAULT_OVERVOLT     0x0010U  /* 过压 */
#define SERVO_FAULT_VOLT_INVALID 0x0020U  /* 电压采样无效 */

/* 保护开关位(参数 46) */
#define SERVO_PROT_TEMP          0x01U  /* 过温保护 */
#define SERVO_PROT_CURRENT       0x02U  /* 过流保护 */
#define SERVO_PROT_STALL         0x04U  /* 堵转保护 */
#define SERVO_PROT_VOLTAGE       0x08U  /* 电压保护 */

/* ============================== 枚举 ============================== */

typedef enum {
    SERVO_STATE_IDLE = 0,      /* 空闲 */
    SERVO_STATE_MOVING,        /* 运动 */
    SERVO_STATE_PAUSED,        /* 暂停 */
    SERVO_STATE_RELEASED,      /* 自由释放 */
    SERVO_STATE_DAMPED,        /* 阻尼释放 */
    SERVO_STATE_RECOVER,       /* 反馈恢复 */
    SERVO_STATE_FAULT          /* 故障 */
} Servo_WorkState_t;

typedef enum {
    SERVO_TORQUE_FREE = 0,     /* 自由释放 */
    SERVO_TORQUE_NORMAL,       /* 正常 */
    SERVO_TORQUE_DAMPING       /* 自定义阻尼(比例见 Servo_SetDamping) */
} Servo_TorqueMode_t;

typedef enum {
    SERVO_CALIB_MID = 1,       /* 当前位置标定为 1500us(SCK) */
    SERVO_CALIB_ZERO,          /* 当前位置标定为 500us(SCZ) */
    SERVO_CALIB_TRAVEL_500,    /* 当前位置设为行程 500 端，进入模式13(AMI) */
    SERVO_CALIB_TRAVEL_2500,   /* 当前位置设为行程 2500 端，进入模式13(AMX) */
    SERVO_CALIB_LIMIT_MIN,     /* 当前位置设为目标下限(SMI) */
    SERVO_CALIB_LIMIT_MAX,     /* 当前位置设为目标上限(SMX) */
    SERVO_CALIB_SAVE_STARTUP   /* 当前位置保存为上电位置(CSD) */
} Servo_Calib_t;

typedef enum {
    SERVO_STORE_FLASH = 0,     /* 写 Flash：掉电保存，同时更新当前运行值 */
    SERVO_STORE_RAM            /* 写 RAM：仅本次上电有效 */
} Servo_Store_t;

/* 可设置参数(值 = Flash 地址)。带 [RAM] 的有对应 RAM 地址，可选 SERVO_STORE_RAM；其余只在 Flash */
typedef enum {
    SERVO_P_ID              = 0x20,  /* 舵机ID 1B，0~254，改后用新ID通信 */
    SERVO_P_BAUD            = 0x21,  /* 波特率档位 1B，1~8，见 Servo_BaudOf */
    SERVO_P_MODE            = 0x22,  /* 工作模式 1B，1~12(电位器版无7/8)，13由行程标定进入 */
    SERVO_P_BOOT_ACTION     = 0x23,  /* 上电动作 1B：1转到上电位置 2原地保持 3自由释放 */
    SERVO_P_TORQUE_LIMIT    = 0x24,  /* 扭矩上限 1B，0~100%，0=禁止驱动 [RAM 84] */
    SERVO_P_STARTUP_POS     = 0x25,  /* 上电位置 2B，位置码 0~4095 */
    SERVO_P_MAX_SPEED       = 0x30,  /* 最大速度 2B，°/s，0=额定，上限[04] [RAM 85] */
    SERVO_P_MAX_ACCEL       = 0x31,  /* 最大加速度 2B，°/s²，0=额定，上限[05] [RAM 86] */
    SERVO_P_MAX_DECEL       = 0x32,  /* 最大减速度 2B，°/s²，0=额定，上限[06] [RAM 87] */
    SERVO_P_SMOOTH_TIME     = 0x33,  /* S曲线平滑时间 1B，0~200ms，0=自动 [RAM 88] */
    SERVO_P_MIN_MOVE_TIME   = 0x34,  /* 最短运动时间 2B，0~500ms，0=不限 [RAM 89] */
    SERVO_P_HOLD_P          = 0x35,  /* 保持比例P 1B，0~100 ‰/° [RAM 8A] */
    SERVO_P_ARRIVE_PREC     = 0x37,  /* 到位精度 2B，[08]×1.5~500，0.01° [RAM 8E] */
    SERVO_P_HOLD_I          = 0x38,  /* 保持积分I 1B，0~100 ‰/(°·s) [RAM 8F] */
    SERVO_P_OC_LIMIT        = 0x40,  /* 过流阈值 2B，500~5000mA */
    SERVO_P_OC_TIME         = 0x41,  /* 过流持续时间 2B，10~1000ms */
    SERVO_P_OT_LIMIT        = 0x42,  /* 过温阈值 2B，300~1000(0.1℃) */
    SERVO_P_OT_TIME         = 0x43,  /* 过温持续时间 2B，250~10000ms */
    SERVO_P_STALL_THRESHOLD = 0x44,  /* 堵转位移阈值 2B，10~[04]/2，0.01°/20ms */
    SERVO_P_STALL_COUNT     = 0x45,  /* 堵转连续次数 1B，1~100 */
    SERVO_P_PROTECT_MASK    = 0x46,  /* 保护开关 1B，0~15，见 SERVO_PROT_* */
    SERVO_P_VOLTAGE_MIN     = 0x47,  /* 最低工作电压 1B，0.1V，范围随型号 */
    SERVO_P_VOLTAGE_MAX     = 0x48,  /* 最高工作电压 1B，0.1V，须比最低高≥0.5V */
    SERVO_P_MIN_START_TORQUE= 0x49   /* 最小启动扭矩 1B，0~100%，0=关 */
} Servo_Param_t;

/* ============================== 结构体 ============================== */

/* 移植接口：用户实现 */
typedef struct {
    void     (*write)(const uint8_t *buf, uint16_t len);  /* 必填：发送并阻塞到最后一字节发完(一帧内字节须连续) */
    int      (*read_byte)(uint8_t *b);                    /* 必填：非阻塞取1字节，有数据返回1，无数据返回0 */
    uint32_t (*millis)(void);                             /* 必填：毫秒计时，允许回绕 */
    void     (*set_dir)(uint8_t tx);                      /* 可选：半双工方向，1=发送 0=接收；NULL=不需要 */
    void     (*set_baud)(uint32_t bps);                   /* 可选：舵机波特率改变后主控同步切换；NULL=不切换 */
} Servo_Port_t;

/* 帧解析器(底层，可单独使用) */
typedef struct {
    uint8_t buf[SERVO_FRAME_MAX];  /* 正在收的帧 */
    uint8_t len;                   /* 已收字节数 */
} Servo_Parser_t;

/* 一条总线 */
typedef struct {
    Servo_Port_t   port;           /* 移植回调 */
    uint16_t       timeout_ms;     /* 应答超时，Servo_Init 设为 SERVO_DEFAULT_TIMEOUT */
    uint8_t        bcast_max_id;   /* 广播等待按的最大ID，Servo_Init 设为 254，按实际总线改小可缩短等待 */
    Servo_Parser_t parser;         /* 内部用 */
} Servo_Bus_t;

/* 设备信息(00~08、10) */
typedef struct {
    uint8_t  type;          /* 1/3 电位器，2/4 磁编码 */
    uint8_t  hw_version;    /* 0.1版 */
    uint8_t  voltage;       /* 标称电压 V */
    uint8_t  torque_kg;     /* 扭力型号 kg */
    uint16_t vmax;          /* 额定最大速度 °/s */
    uint16_t amax;          /* 额定最大加速度 °/s² */
    uint16_t dmax;          /* 额定最大减速度 °/s² */
    uint8_t  smooth_ms;     /* 自动平滑时间 ms */
    uint8_t  min_step;      /* 最小可靠位移 0.01° */
    uint8_t  fw[3];         /* 固件版本 主/次/修订 */
} Servo_Info_t;

/* 状态集合(C9)，方向均为"位置值增大为正" */
typedef struct {
    uint16_t target;        /* 目标位置码 */
    uint16_t position;      /* 实际位置码 */
    int16_t  torque;        /* 扭矩 %，测不到为0 */
    int16_t  speed;         /* 速度 0.1°/s */
    int16_t  current;       /* 电流 mA */
    int16_t  temperature;   /* 温度 0.1℃ */
    uint16_t voltage;       /* 电压 0.01V */
} Servo_Telemetry_t;

/* ============================== 1. 底层 ============================== */

uint8_t  Servo_Crc8(const uint8_t *p, uint16_t n);                         /* CRC-8/ATM */
uint16_t Servo_BuildFrame(uint8_t *out, uint8_t id, uint8_t cmd,
                          const uint8_t *data, uint8_t len);               /* 组帧，返回帧长；len>32 返回0 */
void     Servo_ParserReset(Servo_Parser_t *ps);                            /* 清解析器 */
int      Servo_ParseByte(Servo_Parser_t *ps, uint8_t b);                   /* 喂1字节；收到一帧完整且CRC正确返回1，帧在 ps->buf，长 ps->len */

/* ============================== 2. 初始化与通用读写 ============================== */

void Servo_Init(Servo_Bus_t *bus, const Servo_Port_t *port);              /* 填默认超时与广播最大ID */

/* 收发一帧：发 id/cmd/data，等 CMD|0x80 的应答，返回状态码；应答中状态码后面的数据拷到 rx(最多 rx_cap 字节)，长度放 *rx_len。
 * 广播：等够延时后返回 0，rx 不填。rx/rx_len 可为 NULL */
int Servo_Transfer(Servo_Bus_t *bus, uint8_t id, uint8_t cmd, const uint8_t *data, uint8_t len,
                   uint8_t *rx, uint8_t rx_cap, uint8_t *rx_len);

int Servo_ReadParam(Servo_Bus_t *bus, uint8_t id, uint8_t addr, uint8_t *val, uint8_t cap, uint8_t *len); /* 读任意地址，值小端 */
int Servo_WriteParam(Servo_Bus_t *bus, uint8_t id, uint8_t addr, const uint8_t *val, uint8_t len);        /* 写任意地址，值小端 */
int Servo_ReadU8(Servo_Bus_t *bus, uint8_t id, uint8_t addr, uint8_t *v);    /* 读1字节参数 */
int Servo_ReadU16(Servo_Bus_t *bus, uint8_t id, uint8_t addr, uint16_t *v);  /* 读2字节参数(有符号量强转 int16_t) */
int Servo_WriteU8(Servo_Bus_t *bus, uint8_t id, uint8_t addr, uint8_t v);    /* 写1字节参数 */
int Servo_WriteU16(Servo_Bus_t *bus, uint8_t id, uint8_t addr, uint16_t v);  /* 写2字节参数 */

/* ============================== 3. 运动控制 ============================== */

int Servo_Move(Servo_Bus_t *bus, uint8_t id, uint16_t pos, uint16_t time_ms);   /* 运动到位置码 pos，time_ms=0 最快 */
int Servo_MoveUs(Servo_Bus_t *bus, uint8_t id, uint16_t us, uint16_t time_ms);  /* 同上，脉宽 500~2500us */
int Servo_MoveTurns(Servo_Bus_t *bus, uint8_t id, uint16_t pos, uint8_t turns,
                    uint16_t turn_ms);                                          /* 多圈(仅磁编码版)：额外 0~9 圈，turn_ms 为每圈时间；电位器版返回 7 */
int Servo_Pause(Servo_Bus_t *bus, uint8_t id);                                  /* 平滑暂停，保留目标 */
int Servo_Resume(Servo_Bus_t *bus, uint8_t id);                                 /* 继续暂停前的目标 */
int Servo_Stop(Servo_Bus_t *bus, uint8_t id);                                   /* 停止并清除目标 */
int Servo_SetTorqueMode(Servo_Bus_t *bus, uint8_t id, Servo_TorqueMode_t mode); /* 释放/正常/阻尼 */
int Servo_SetDamping(Servo_Bus_t *bus, uint8_t id, uint8_t percent);           /* 阻尼比例 0~100%，扭矩模式2时生效 */

/* ============================== 4. 状态读取 ============================== */

int Servo_GetState(Servo_Bus_t *bus, uint8_t id, uint8_t *work_state, uint8_t *fault_lo); /* CMD01：工作状态 + 故障低8位 */
int Servo_ReadTelemetry(Servo_Bus_t *bus, uint8_t id, Servo_Telemetry_t *t);  /* 一次读七项(C9) */
int Servo_ReadPosition(Servo_Bus_t *bus, uint8_t id, uint16_t *pos);          /* C2 位置码 */
int Servo_ReadSpeed(Servo_Bus_t *bus, uint8_t id, int16_t *speed);            /* C3 0.1°/s */
int Servo_ReadTemperature(Servo_Bus_t *bus, uint8_t id, int16_t *temp);       /* C4 0.1℃ */
int Servo_ReadVoltage(Servo_Bus_t *bus, uint8_t id, uint16_t *cv);            /* C5 0.01V */
int Servo_ReadCurrent(Servo_Bus_t *bus, uint8_t id, int16_t *ma);             /* C6 mA */
int Servo_ReadTorque(Servo_Bus_t *bus, uint8_t id, int16_t *pct);             /* C7 % */
int Servo_ReadFault(Servo_Bus_t *bus, uint8_t id, uint16_t *fault);           /* C1 故障位掩码 */
int Servo_ReadStalled(Servo_Bus_t *bus, uint8_t id, uint8_t *stalled);        /* C8 1=堵转保持中 */

/* ============================== 5. 设备信息 ============================== */

int Servo_ReadInfo(Servo_Bus_t *bus, uint8_t id, Servo_Info_t *info);         /* 读 00~08 与 10 */

/* ============================== 6. 参数设置 ============================== */

/* 写参数：store 选 Flash/RAM；只在 Flash 的参数选 RAM 返回 SERVO_ERR_ARG。
 * 驱动先按固定范围预检(越界不发帧)，随型号变化的上限(速度/到位精度/电压等)由舵机校验，越界返回 3 */
int Servo_SetParam(Servo_Bus_t *bus, uint8_t id, Servo_Param_t p, uint16_t value, Servo_Store_t store);
int Servo_GetParam(Servo_Bus_t *bus, uint8_t id, Servo_Param_t p, uint16_t *value, Servo_Store_t store);

/* 常用参数的便捷写法(都转到 Servo_SetParam/Servo_GetParam) */
static inline int Servo_SetId(Servo_Bus_t *b, uint8_t id, uint8_t new_id)               { return Servo_SetParam(b, id, SERVO_P_ID, new_id, SERVO_STORE_FLASH); }
static inline int Servo_SetBaud(Servo_Bus_t *b, uint8_t id, uint8_t code)               { return Servo_SetParam(b, id, SERVO_P_BAUD, code, SERVO_STORE_FLASH); }
static inline int Servo_SetMode(Servo_Bus_t *b, uint8_t id, uint8_t mode)               { return Servo_SetParam(b, id, SERVO_P_MODE, mode, SERVO_STORE_FLASH); }
static inline int Servo_SetBootAction(Servo_Bus_t *b, uint8_t id, uint8_t act)          { return Servo_SetParam(b, id, SERVO_P_BOOT_ACTION, act, SERVO_STORE_FLASH); }
static inline int Servo_SetStartupPos(Servo_Bus_t *b, uint8_t id, uint16_t pos)         { return Servo_SetParam(b, id, SERVO_P_STARTUP_POS, pos, SERVO_STORE_FLASH); }
static inline int Servo_SetTorqueLimit(Servo_Bus_t *b, uint8_t id, uint8_t pct, Servo_Store_t s)   { return Servo_SetParam(b, id, SERVO_P_TORQUE_LIMIT, pct, s); }
static inline int Servo_SetMaxSpeed(Servo_Bus_t *b, uint8_t id, uint16_t dps, Servo_Store_t s)     { return Servo_SetParam(b, id, SERVO_P_MAX_SPEED, dps, s); }
static inline int Servo_SetMaxAccel(Servo_Bus_t *b, uint8_t id, uint16_t dps2, Servo_Store_t s)    { return Servo_SetParam(b, id, SERVO_P_MAX_ACCEL, dps2, s); }
static inline int Servo_SetMaxDecel(Servo_Bus_t *b, uint8_t id, uint16_t dps2, Servo_Store_t s)    { return Servo_SetParam(b, id, SERVO_P_MAX_DECEL, dps2, s); }
static inline int Servo_SetSmoothTime(Servo_Bus_t *b, uint8_t id, uint8_t ms, Servo_Store_t s)     { return Servo_SetParam(b, id, SERVO_P_SMOOTH_TIME, ms, s); }
static inline int Servo_SetMinMoveTime(Servo_Bus_t *b, uint8_t id, uint16_t ms, Servo_Store_t s)   { return Servo_SetParam(b, id, SERVO_P_MIN_MOVE_TIME, ms, s); }
static inline int Servo_SetHoldP(Servo_Bus_t *b, uint8_t id, uint8_t v, Servo_Store_t s)           { return Servo_SetParam(b, id, SERVO_P_HOLD_P, v, s); }
static inline int Servo_SetHoldI(Servo_Bus_t *b, uint8_t id, uint8_t v, Servo_Store_t s)           { return Servo_SetParam(b, id, SERVO_P_HOLD_I, v, s); }
static inline int Servo_SetArrivePrecision(Servo_Bus_t *b, uint8_t id, uint16_t cdeg, Servo_Store_t s) { return Servo_SetParam(b, id, SERVO_P_ARRIVE_PREC, cdeg, s); }
static inline int Servo_SetProtectMask(Servo_Bus_t *b, uint8_t id, uint8_t mask)        { return Servo_SetParam(b, id, SERVO_P_PROTECT_MASK, mask, SERVO_STORE_FLASH); }

/* 工作电压上下限一起写(0.1V)：自动按"先放宽再收紧"的顺序写，避免中间态不合法被拒 */
int Servo_SetVoltageRange(Servo_Bus_t *bus, uint8_t id, uint8_t min_dv, uint8_t max_dv);

/* ============================== 7. 校准与系统动作 ============================== */

int Servo_Calibrate(Servo_Bus_t *bus, uint8_t id, Servo_Calib_t act);        /* 校准，结果保存到 Flash */
/* 中点微调(同明文 SCK±nnn)：舵机先最快转到 1500+us，到位后把实测位置记为新中点(存 Flash)。
 * 返回 0 只表示已受理；到位校准是异步的，可轮询 Servo_GetState 到 IDLE 后读位置确认(应≈2048) */
int Servo_TrimMid(Servo_Bus_t *bus, uint8_t id, int16_t us);
/* 主/从(CMD 04，存 Flash)：slave=1 后该舵机明文/二进制都不再应答(本条也不应答，驱动只发不等)；
 * slave=0 恢复正常并等应答。用途：两台同ID并联，只留主舵机应答。07 恢复默认保留此设置，08 清除，换固件回主 */
int Servo_SetSlave(Servo_Bus_t *bus, uint8_t id, uint8_t slave);
int Servo_ReloadRamParams(Servo_Bus_t *bus, uint8_t id);                     /* RAM 参数重新从 Flash 载入(8C=1) */
int Servo_FactoryReset(Servo_Bus_t *bus, uint8_t id, uint8_t keep_id);       /* 恢复默认：keep_id=1 保留ID；波特率回 115200 */

uint32_t Servo_BaudOf(uint8_t code);                                         /* 档位 1~8 → bps，非法返回0 */

#ifdef __cplusplus
}
#endif

#endif /* SERVO_DRV_H */
