/********************************** (C) COPYRIGHT  *******************************
 * File Name          : iap.c
 * Author             : WCH
 * Version            : V1.0.1
 * Date               : 2025/01/13
 * Description        : IAP
 *******************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "iap.h"
#include "D_uart.h"
#include "string.h"
#include "flash.h"
#include "core_riscv.h"
#include <stdlib.h>
/******************************************************************************/

u32 Program_addr = FLASH_Base;
u32 Verify_addr = FLASH_Base;
u8 Verify_Star_flag = 0;
u8 Fast_Program_Buf[390];
u16 CodeLen = 0;
u8 End_Flag = 0;
u8 EP2_Rx_Buffer[USBD_DATA_SIZE+4];
#define  isp_cmd_t   ((isp_cmd  *)EP2_Rx_Buffer)


/*********************************************************************
 * @fn      RecData_Deal
 *
 * @brief   UART (deal jump IAP command)
 *
 * @return  ERR_ERROR - ERROR
 *          ERR_SUCCESS - SUCCESS
 *          ERR_End - End
 */
u8 RecData_Deal(void)
{
    u8 s;

    switch ( isp_cmd_t->UART.Cmd) {
        case CMD_IAP_ERASE:
            s = ERR_ERROR;
            break;

        case CMD_IAP_PROM:
            s = ERR_ERROR;
            break;

        case CMD_IAP_VERIFY:
            s = ERR_ERROR;
            break;

        case CMD_IAP_END:
            s = ERR_ERROR;
            break;

        case CMD_JUMP_IAP:
            FLASH_Unlock_Fast();
            FLASH_ErasePage_Fast(CalAddr & 0xFFFFFF00);
            FLASH->CTLR |= ((uint32_t)0x00008000);
            FLASH->CTLR |= ((uint32_t)0x00000080);
            s = ERR_SUCCESS;
            break;

        default:
            s = ERR_ERROR;
            break;
    }

    return s;
}


/* UART Rx state machine states */
#define RX_STATE_IDLE          0
#define RX_STATE_WAIT_HEAD2    1
#define RX_STATE_WAIT_CMD      2
#define RX_STATE_WAIT_LEN      3
#define RX_STATE_WAIT_ADDR     4
#define RX_STATE_WAIT_DATA     5
#define RX_STATE_WAIT_CKSUM_LO 6
#define RX_STATE_WAIT_CKSUM_HI 7
#define RX_STATE_WAIT_TAIL2    8
#define RX_STATE_WAIT_TAIL1    9

static u8  rx_state = RX_STATE_IDLE;
static u8  rx_idx = 0;
static u16 rx_cksum = 0;

/*********************************************************************
 * @fn      IAP_Rx_Deal
 *
 * @brief   Byte-by-byte UART Rx data deal (called from the UART task).
 *          Sniffs the same byte stream as the servo ASCII protocol and
 *          only reacts to a complete 0xAA/0x55 framed IAP command.
 *
 * @param   data - the received byte
 *
 * @return  none
 */
void IAP_Rx_Deal(uint8_t data)
{
    u8 s;

    switch (rx_state)
    {
        case RX_STATE_IDLE:
            if (data == Uart_Sync_Head1)
            {
                rx_state = RX_STATE_WAIT_HEAD2;
            }
            break;

        case RX_STATE_WAIT_HEAD2:
            if (data == Uart_Sync_Head2)
            {
                rx_state = RX_STATE_WAIT_CMD;
                rx_cksum = 0;
            }
            else
            {
                rx_state = RX_STATE_IDLE;
            }
            break;

        case RX_STATE_WAIT_CMD:
            isp_cmd_t->UART.Cmd = data;
            rx_cksum += data;
            rx_state = RX_STATE_WAIT_LEN;
            break;

        case RX_STATE_WAIT_LEN:
            isp_cmd_t->UART.Len = data;
            rx_cksum += data;
            if (isp_cmd_t->UART.Cmd == CMD_IAP_ERASE
                    || isp_cmd_t->UART.Cmd == CMD_IAP_VERIFY)
            {
                rx_state = RX_STATE_WAIT_ADDR;
                rx_idx = 0;
            }
            else if (isp_cmd_t->UART.Cmd == CMD_IAP_PROM
                    && isp_cmd_t->UART.Len > 0)
            {
                rx_state = RX_STATE_WAIT_DATA;
                rx_idx = 0;
            }
            else
            {
                rx_state = RX_STATE_WAIT_CKSUM_LO;
            }
            break;

        case RX_STATE_WAIT_ADDR:
            isp_cmd_t->other.buf[2 + rx_idx] = data;
            rx_cksum += data;
            rx_idx++;
            if (rx_idx >= 4)
            {
                if (isp_cmd_t->UART.Cmd == CMD_IAP_VERIFY
                        && isp_cmd_t->UART.Len > 0)
                {
                    rx_state = RX_STATE_WAIT_DATA;
                    rx_idx = 0;
                }
                else
                {
                    rx_state = RX_STATE_WAIT_CKSUM_LO;
                }
            }
            break;

        case RX_STATE_WAIT_DATA:
            isp_cmd_t->UART.data[rx_idx] = data;
            rx_cksum += data;
            rx_idx++;
            if (rx_idx >= isp_cmd_t->UART.Len)
            {
                rx_state = RX_STATE_WAIT_CKSUM_LO;
            }
            break;

        case RX_STATE_WAIT_CKSUM_LO:
            if (data == (uint8_t)(rx_cksum & 0xFF))
            {
                rx_state = RX_STATE_WAIT_CKSUM_HI;
            }
            else
            {
                rx_state = RX_STATE_IDLE;
            }
            break;

        case RX_STATE_WAIT_CKSUM_HI:
            if (data == (uint8_t)(rx_cksum >> 8))
            {
                rx_state = RX_STATE_WAIT_TAIL2;
            }
            else
            {
                rx_state = RX_STATE_IDLE;
            }
            break;

        case RX_STATE_WAIT_TAIL2:
            if (data == Uart_Sync_Head2)
            {
                rx_state = RX_STATE_WAIT_TAIL1;
            }
            else
            {
                rx_state = RX_STATE_IDLE;
            }
            break;

        case RX_STATE_WAIT_TAIL1:
            if (data == Uart_Sync_Head1)
            {
                /* Complete frame received - process it */
                s = RecData_Deal();

                if (s != ERR_End)
                {
                    /* 应答帧交给串口驱动的发送队列，不再逐字节阻塞等TC。
                     * 原实现在接收中断里死等发送完成，会把1ms控制任务顶掉。 */
                    uint8_t ack[6];   /* AA 55 00 <err> 55 AA */

                    ack[0] = Uart_Sync_Head1;
                    ack[1] = Uart_Sync_Head2;
                    ack[2] = 0x00;
                    ack[3] = (s == ERR_ERROR) ? 0x01 : 0x00;
                    ack[4] = Uart_Sync_Head2;
                    ack[5] = Uart_Sync_Head1;
                    (void)D_UART1_Tx_Write(ack, sizeof(ack));
                }
            }
            rx_state = RX_STATE_IDLE;
            break;

        default:
            rx_state = RX_STATE_IDLE;
            break;
    }
}

/*********************************************************************
 * @fn      APP_2_IAP
 *
 * @brief   APP_2_IAP program.
 *
 * @return  none
 */
void APP_2_IAP(void)
{
    RCC_ClearFlag();
    SystemReset_StartMode(Start_Mode_BOOT);
    NVIC_SystemReset();
}

/*==========================================================================
 * IAP升级检测 — 上电时校验升级标志, 决定是否进入Bootloader
 *==========================================================================*/
void IAP_Check(void)
{
    if (*(uint32_t*)CalAddr != CheckNum)
    {
        APP_2_IAP();
        while (1);
    }
}