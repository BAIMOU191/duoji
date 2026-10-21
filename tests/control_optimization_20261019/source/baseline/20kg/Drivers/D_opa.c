/* D_opa.c OPA1运算放大器驱动：PA2正输入，8x增益，PGA基准VB=VDD/4 */

#include "D_opa.h"

/* 初始化OPA1：8x增益并开PGA基准VB=VDD/4。
 * 检流压降只有几毫伏，VB关闭时近地端不线性(实测读数仅真值36%)；VB带来的零点台阶由上电零点标定扣除。
 * 不要用32x增益(已不线性)。 */
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
