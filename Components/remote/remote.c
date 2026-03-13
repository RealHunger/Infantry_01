#include "remote.h"

#include "cmsis_os.h"
#include "usart.h"
#include "string.h"


#include "../../Bsp/LED/bsp_LED.h"

/* 声明外部句柄，请确保 CubeMX 中 USART6 和 DMA 已配置 */
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

static uint8_t rc_rx_buf[2][RC_RX_BUF_SIZE]; // DMA 双缓冲区
RC_ctrl_t remote_ctrl;                       // 映射后的逻辑数据

/* --- 官方 CRC16 校验表 --- */
static const uint16_t crc16_tab[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/**
 * @brief CRC16 校验计算
 */
static uint16_t RC_CRC16_Check(uint8_t *p_msg, uint16_t len)
{
    uint16_t crc16 = 0xFFFF;
    while (len--)
    {
        crc16 = (crc16 >> 8) ^ crc16_tab[(crc16 ^ *p_msg++) & 0x00ff];
    }
    return crc16;
}

/**
 * @brief 遥控器数据解析
 * @param p_frame 接收到的 21 字节原始数据包
 */
static void RC_Data_Parse(volatile const uint8_t *p_frame)
{
    remote_raw_t *raw = (remote_raw_t *)p_frame;

    //基础校验 (帧头 + CRC)
    if (raw->sof_1 != 0xA9 || raw->sof_2 != 0x53) return;
    if (RC_CRC16_Check((uint8_t *)p_frame, RC_FRAME_LENGTH - 2) != raw->crc16) return;

    // 1. 映射遥控器摇杆与拨轮
    remote_ctrl.rc.ch[0] = (int16_t)raw->ch_0 - RC_CH_VALUE_OFFSET;
    remote_ctrl.rc.ch[1] = (int16_t)raw->ch_1 - RC_CH_VALUE_OFFSET;
    remote_ctrl.rc.ch[2] = (int16_t)raw->ch_2 - RC_CH_VALUE_OFFSET;
    remote_ctrl.rc.ch[3] = (int16_t)raw->ch_3 - RC_CH_VALUE_OFFSET;
    remote_ctrl.rc.wheel = (int16_t)raw->wheel - RC_CH_VALUE_OFFSET;

    // 2. 映射遥控器按键与挡位
    remote_ctrl.rc.sw       = (uint8_t)raw->mode_sw;
    remote_ctrl.rc.pause    = (uint8_t)raw->btn_pause;
    remote_ctrl.rc.custom_l = (uint8_t)raw->btn_custom_l;
    remote_ctrl.rc.custom_r = (uint8_t)raw->btn_custom_r;
    remote_ctrl.rc.trigger  = (uint8_t)raw->btn_trigger;

    // 3. 映射鼠标
    remote_ctrl.mouse.x = raw->mouse_x;
    remote_ctrl.mouse.y = raw->mouse_y;
    remote_ctrl.mouse.z = raw->mouse_z;
    remote_ctrl.mouse.press_l = (uint8_t)raw->mouse_left;
    remote_ctrl.mouse.press_r = (uint8_t)raw->mouse_right;
    remote_ctrl.mouse.press_m = (uint8_t)raw->mouse_middle;

    // 4. 映射键盘
    remote_ctrl.key.v = raw->key_v;

    remote_ctrl.last_update_tick = osKernelSysTick();
}

/**
 * @brief USART6 中断服务函数
 * 需在 stm32xxxx_it.c 中调用或直接定义
 */
////  现在空闲中断函数被裁判系统接收占用了喵
// void USART6_IRQHandler(void)
// {
//     if (huart6.Instance->SR & UART_FLAG_IDLE)
//     {
//         __HAL_UART_CLEAR_IDLEFLAG(&huart6);
//
//         uint16_t rx_len;
//         // 检查当前 DMA 正在向哪个 Memory 写入
//         uint8_t current_mem = (hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) ? 1 : 0;
//
//         // 停止 DMA 以重置计数器
//         __HAL_DMA_DISABLE(&hdma_usart6_rx);
//         rx_len = RC_RX_BUF_SIZE - hdma_usart6_rx.Instance->NDTR;
//         hdma_usart6_rx.Instance->NDTR = RC_RX_BUF_SIZE;
//
//         // 切换缓冲区目标
//         if (current_mem == 0) hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
//         else hdma_usart6_rx.Instance->CR &= ~DMA_SxCR_CT;
//
//         __HAL_DMA_ENABLE(&hdma_usart6_rx);
//
//         // 只有长度匹配才解析 (21字节)
//         if (rx_len == RC_FRAME_LENGTH)
//         {
//             RC_Data_Parse(rc_rx_buf[current_mem]);
//         }
//     }
// }

/**
 * @brief 遥控器接收初始化 (USART6 + DMA 双缓冲)
 */
//把防止错误调用usart6
// void RC_Init(void)
// {
//     // 使能串口 DMA 接收和空闲中断
//     SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
//     __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
//
//     // 配置 DMA
//     __HAL_DMA_DISABLE(&hdma_usart6_rx);
//     hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
//     hdma_usart6_rx.Instance->M0AR = (uint32_t)(rc_rx_buf[0]);
//     hdma_usart6_rx.Instance->M1AR = (uint32_t)(rc_rx_buf[1]);
//     hdma_usart6_rx.Instance->NDTR = RC_RX_BUF_SIZE;
//
//     // 开启双缓冲模式
//     SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
//     __HAL_DMA_ENABLE(&hdma_usart6_rx);
// }

void RC_Unable(void) { __HAL_UART_DISABLE(&huart6); }
const RC_ctrl_t *RC_Get_Handle(void) { return &remote_ctrl; }