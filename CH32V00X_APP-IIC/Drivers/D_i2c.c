/* D_i2c.c I2C1总线驱动：400kHz，含超时、错误检测与总线恢复 */

#include "D_i2c.h"

static uint8_t s_i2c_initialized; /* 初始化完成标志 */

#define I2C_STAR1_ERROR_MASK  (I2C_STAR1_BERR | I2C_STAR1_ARLO | \
                               I2C_STAR1_AF   | I2C_STAR1_OVR)
#define I2C_BUS_PINS          (GPIO_Pin_1 | GPIO_Pin_2)
#define I2C_SDA_PIN           GPIO_Pin_1
#define I2C_SCL_PIN           GPIO_Pin_2

/*
 * @fn      I2C_ApplyConfig
 * @brief   配置I2C时序，供初始化和软件复位恢复共用
 * @param   I2Cx I2C外设
 * @return  无
 */
static void I2C_ApplyConfig(I2C_TypeDef *I2Cx)
{
    I2C_InitTypeDef config = {0};

    config.I2C_ClockSpeed          = 400000;
    config.I2C_Mode                = I2C_Mode_I2C;
    config.I2C_DutyCycle           = I2C_DutyCycle_16_9;
    config.I2C_OwnAddress1         = 0x00;
    config.I2C_Ack                 = I2C_Ack_Enable;
    config.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    I2C_Init(I2Cx, &config);
    I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current);
    I2C_Cmd(I2Cx, ENABLE);
}

/*
 * @fn      I2C_WaitStar1
 * @brief   等待STAR1标志并检查总线错误和超时
 * @param   I2Cx I2C外设
 * @param   mask 等待的标志掩码
 * @return  0=成功，1=错误或超时
 */
static uint8_t I2C_WaitStar1(I2C_TypeDef *I2Cx, uint16_t mask)
{
    uint32_t timeout = I2C_TIMEOUT;

    while ((I2Cx->STAR1 & mask) == 0U)
    {
        if ((I2Cx->STAR1 & I2C_STAR1_ERROR_MASK) != 0U) return 1;
        if (--timeout == 0U) return 1;
    }
    return 0;
}

/*
 * @fn      I2C_ClearADDR
 * @brief   按硬件要求依次读取STAR1和STAR2清除ADDR
 * @param   I2Cx I2C外设
 * @return  无
 */
static void I2C_ClearADDR(I2C_TypeDef *I2Cx)
{
    volatile uint16_t dummy;

    dummy = I2Cx->STAR1;
    dummy = I2Cx->STAR2;
    (void)dummy;
}

/*
 * @fn      I2C_BusDelay
 * @brief   GPIO总线恢复使用的短延时
 * @param   无
 * @return  无
 */
static void I2C_BusDelay(void)
{
    volatile uint8_t count = 24U;

    while (count-- != 0U) __NOP();
}

/*
 * @fn      I2C_ConfigPins
 * @brief   切换I2C引脚的复用或GPIO开漏模式
 * @param   mode GPIO工作模式
 * @return  无
 */
static void I2C_ConfigPins(GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef gpio = {0};

    /* 开漏输出切换前先把输出锁存器置1，避免主动拉低总线。 */
    GPIOC->BSHR = I2C_BUS_PINS;
    gpio.GPIO_Pin   = I2C_BUS_PINS;
    gpio.GPIO_Mode  = mode;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOC, &gpio);
}

/*
 * @fn      I2C_BusClear
 * @brief   输出最多9个SCL并生成STOP以释放卡住的从机
 * @param   无
 * @return  无
 */
static void I2C_BusClear(void)
{
    uint8_t pulse;

    I2C_Cmd(I2C1, DISABLE);
    I2C_ConfigPins(GPIO_Mode_Out_OD);
    GPIOC->BSHR = I2C_BUS_PINS;
    I2C_BusDelay();

    for (pulse = 0U;
         pulse < 9U && GPIO_ReadInputDataBit(GPIOC, I2C_SDA_PIN) == Bit_RESET;
         pulse++)
    {
        GPIOC->BCR = I2C_SCL_PIN;
        I2C_BusDelay();
        GPIOC->BSHR = I2C_SCL_PIN;
        I2C_BusDelay();
    }

    /* SDA低、SCL高、SDA释放，构造一个明确的STOP。 */
    GPIOC->BCR = I2C_SCL_PIN | I2C_SDA_PIN;
    I2C_BusDelay();
    GPIOC->BSHR = I2C_SCL_PIN;
    I2C_BusDelay();
    GPIOC->BSHR = I2C_SDA_PIN;
    I2C_BusDelay();

    I2C_ConfigPins(GPIO_Mode_AF_OD);
}

/*
 * @fn      D_Bus_I2C_Init
 * @brief   初始化I2C1总线与GPIO，已初始化则直接返回
 * @param   无
 * @return  无
 */
void D_Bus_I2C_Init(void)
{
    if (s_i2c_initialized) return;

    GPIO_InitTypeDef  GPIO_InitStructure = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_I2C1, ENABLE);

    /* 官方示例同样使用PC2=SCL、PC1=SDA、30MHz复用开漏。 */
    GPIO_InitStructure.GPIO_Pin   = I2C_BUS_PINS;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    I2C_ApplyConfig(I2C1);
    s_i2c_initialized = 1;
}

/*
 * @fn      I2C_SoftReset
 * @brief   GPIO释放从机并完整重配主机，总线卡死时兜底恢复
 * @param   I2Cx I2C外设
 * @return  无
 */
static void I2C_SoftReset(I2C_TypeDef *I2Cx)
{
    I2C_BusClear();

    /* 主机和从机均已回到空闲电平后，再复位主机外设。 */
    I2C_SoftwareResetCmd(I2Cx, ENABLE);
    I2C_SoftwareResetCmd(I2Cx, DISABLE);

    /* SWRST会清除400kHz时序配置，必须无条件完整重配。 */
    I2C_ApplyConfig(I2Cx);
}

/*
 * @fn      D_I2C_Write
 * @brief   向从机指定寄存器写入len字节
 * @param   devAddr 从机7位地址
 * @param   regAddr 寄存器地址
 * @param   pData   数据指针
 * @param   len     数据长度(字节)
 * @return  0=成功，1=总线错误或超时
 */
uint8_t D_I2C_Write(uint8_t devAddr, uint8_t regAddr, uint8_t *pData, uint8_t len)
{
    uint32_t timeout;

    if ((pData == NULL) || (len == 0)) return 1;

    /* 等待总线空闲 */
    timeout = I2C_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) != RESET)
        if (--timeout == 0) { I2C_SoftReset(I2C1); return 1; }

    I2C_GenerateSTART(I2C1, ENABLE);

    /* 等待起始条件已发送 */
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
        if (--timeout == 0) { I2C_SoftReset(I2C1); return 1; }

    I2C_Send7bitAddress(I2C1, devAddr << 1, I2C_Direction_Transmitter);

    /* 等待从机应答 */
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        if (--timeout == 0) { I2C_SoftReset(I2C1); return 1; }

    I2C_SendData(I2C1, regAddr);

    /* 等待寄存器地址字节发送完成 */
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        if (--timeout == 0) { I2C_SoftReset(I2C1); return 1; }

    /* 逐字节发送数据 */
    while (len--)
    {
        timeout = I2C_TIMEOUT;
        while (I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE) == RESET)
            if (--timeout == 0) { I2C_SoftReset(I2C1); return 1; }
        I2C_SendData(I2C1, *pData++);
    }

    /* 等待最后一字节发送完成 */
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        if (--timeout == 0) { I2C_SoftReset(I2C1); return 1; }

    I2C_GenerateSTOP(I2C1, ENABLE);
    return 0;
}

/*
 * @fn      D_I2C_Read
 * @brief   从从机指定寄存器读取len字节
 * @param   devAddr 从机7位地址
 * @param   regAddr 寄存器地址
 * @param   pData   输出数据指针
 * @param   len     数据长度(字节)
 * @return  0=成功，1=总线错误或超时
 */
uint8_t D_I2C_Read(uint8_t devAddr, uint8_t regAddr, uint8_t *pData, uint8_t len)
{
    uint32_t timeout;
    uint32_t irq_state;

    if ((pData == NULL) || (len == 0)) return 1;

    /* 每笔事务从已知ACK/POS状态开始，并清除上一次错误标志。 */
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
    I2C1->STAR1 &= (uint16_t)~I2C_STAR1_ERROR_MASK;

    /* 阶段一：写入寄存器地址 */
    timeout = I2C_TIMEOUT;
    while ((I2C1->STAR2 & I2C_STAR2_BUSY) != 0U)
        if (--timeout == 0U) goto read_failed;

    I2C_GenerateSTART(I2C1, ENABLE);

    if (I2C_WaitStar1(I2C1, I2C_STAR1_SB)) goto read_failed;

    I2C_Send7bitAddress(I2C1, (uint8_t)(devAddr << 1),
                       I2C_Direction_Transmitter);

    if (I2C_WaitStar1(I2C1, I2C_STAR1_ADDR)) goto read_failed;
    I2C_ClearADDR(I2C1);

    I2C_SendData(I2C1, regAddr);

    if (I2C_WaitStar1(I2C1, I2C_STAR1_BTF)) goto read_failed;

    /* 阶段二：重复起始 + 切换为接收方向 */
    I2C_GenerateSTART(I2C1, ENABLE);

    if (I2C_WaitStar1(I2C1, I2C_STAR1_SB)) goto read_failed;

    /* 两字节接收必须在ADDR清除前设置POS。 */
    if (len == 2U)
        I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Next);
    I2C_Send7bitAddress(I2C1, (uint8_t)(devAddr << 1),
                       I2C_Direction_Receiver);

    /* 不能用I2C_CheckEvent：它会读取STAR2并提前清除ADDR。 */
    if (I2C_WaitStar1(I2C1, I2C_STAR1_ADDR)) goto read_failed;

    if (len == 1U)
    {
        /* 单字节：清ADDR与STOP之间不能被中断打断。 */
        irq_state = __get_MSTATUS();
        __disable_irq();
        I2C_AcknowledgeConfig(I2C1, DISABLE);
        I2C_ClearADDR(I2C1);
        I2C_GenerateSTOP(I2C1, ENABLE);
        __set_MSTATUS(irq_state);

        if (I2C_WaitStar1(I2C1, I2C_STAR1_RXNE)) goto read_failed;
        *pData = I2C_ReceiveData(I2C1);
    }
    else if (len == 2U)
    {
        /* 两字节：NACK第二字节，BTF后一次读出两个字节再发STOP。 */
        irq_state = __get_MSTATUS();
        __disable_irq();
        I2C_AcknowledgeConfig(I2C1, DISABLE);
        I2C_ClearADDR(I2C1);
        __set_MSTATUS(irq_state);

        if (I2C_WaitStar1(I2C1, I2C_STAR1_BTF)) goto read_failed;

        irq_state = __get_MSTATUS();
        __disable_irq();
        I2C_GenerateSTOP(I2C1, ENABLE);
        *pData++ = I2C_ReceiveData(I2C1);
        *pData   = I2C_ReceiveData(I2C1);
        __set_MSTATUS(irq_state);
    }
    else
    {
        /* 通用多字节路径：最后3字节按BTF时序收尾。 */
        I2C_ClearADDR(I2C1);

        while (len > 3U)
        {
            if (I2C_WaitStar1(I2C1, I2C_STAR1_RXNE)) goto read_failed;
            *pData++ = I2C_ReceiveData(I2C1);
            len--;
        }

        if (I2C_WaitStar1(I2C1, I2C_STAR1_BTF)) goto read_failed;
        I2C_AcknowledgeConfig(I2C1, DISABLE);
        *pData++ = I2C_ReceiveData(I2C1);

        if (I2C_WaitStar1(I2C1, I2C_STAR1_BTF)) goto read_failed;
        I2C_GenerateSTOP(I2C1, ENABLE);
        *pData++ = I2C_ReceiveData(I2C1);
        *pData   = I2C_ReceiveData(I2C1);
    }

    /* 等STOP真正释放总线后再恢复ACK/POS，避免影响最后一个NACK。 */
    timeout = I2C_TIMEOUT;
    while ((I2C1->STAR2 & I2C_STAR2_BUSY) != 0U)
        if (--timeout == 0U) goto read_failed;

    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
    return 0;

read_failed:
    I2C_SoftReset(I2C1);
    return 1;
}
