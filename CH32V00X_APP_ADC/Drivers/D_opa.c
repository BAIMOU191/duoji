/* D_opa.c OPA1运算放大器驱动：PA2正输入，8x增益，PGA基准VB=VDD/4 */

#include "D_opa.h"

/*
 * @fn      D_OPA1_Init
 * @brief   初始化OPA1：解锁→配置GPIO→配置运放参数→使能→锁定
 * @param   无
 * @return  无
 *
 * 为什么必须开PGA基准电压VB：
 *   检流电阻只有60mΩ，空载约90mA时压降仅5.6mV，输入几乎贴着地。实测
 *   VB关闭时读数只有真值的36%，而且提高增益无效——4x到32x增益涨8倍，
 *   信号只涨2.9倍，说明失效的是近地端的线性度而不是增益。开VB把工作点
 *   抬到约760mV后达成率回到105%，1A处与万用表也吻合到5%以内。
 *   VB选VDD/4而不是VDD/2：台阶低，留给信号的量程大(5.29A vs 3.56A)，
 *   正好覆盖到AT8236自身5.5A的过流点。
 *   不要用32x：实测两种VB下零点都钳在同一个值，已经不线性了。
 *
 * VB带来的零点台阶由 D_ADC 的上电零点标定自动测出并扣除，对上层透明。
 */
void D_OPA1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    OPA_InitTypeDef  OPA_InitStructure  = {0};

    OPA_Unlock();

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; /* PA2模拟输入 */
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    OPA_InitStructure.Mode      = OUT_IO_OUT0;
    OPA_InitStructure.PGADIF    = PGADIF_OFF;
    OPA_InitStructure.PSEL      = CHP0;              /* 正输入：CHP0=PA2 */
    OPA_InitStructure.NSEL      = CHN_PGA_8xIN;      /* 单端PGA 8x       */
    OPA_InitStructure.FB        = FB_ON;             /* 内部反馈使能     */
    OPA_InitStructure.OPA_HS    = HS_ON;             /* 高速模式         */
    OPA_InitStructure.PGA_VBEN  = PGA_VBEN_ON;       /* 启用PGA基准电压  */
    OPA_InitStructure.PGA_VBSEL = PGA_VBSEL_VDD_DIV4;/* VB = VDD/4       */
    OPA_Init(&OPA_InitStructure);

    OPA_Cmd(ENABLE);
    OPA_Lock();
}
