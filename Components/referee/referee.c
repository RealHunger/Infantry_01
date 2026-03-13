#include "referee.h"
#include "usart.h"
#include "string.h"
#include "cmsis_os.h"
#include "D:/all_code/clion/infantry/Bsp//uart/bsp_uart.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern CAN_HandleTypeDef hcan2; // 声明外部的 CAN2 句柄

referee_info_t referee_data;
uint8_t ref_rx_buf[2][REF_RX_BUF_SIZE];
uint8_t ref_tx_buf[REF_RX_BUF_SIZE]; // 【新增】发送缓冲区
static uint8_t tx_seq = 0; // 【新增】发送包序号
uint32_t uart6_rx_count = 0; // 全局测试变量
uint8_t  raw_data_dump[10] = {0}; // 【新增】用来保存前 10 个字节的生肉数据
uint16_t raw_data_len = 0;        // 【新增】记录这次到底收到多长的数据

// ==================== 官方 CRC8 和 CRC16 校验代码 (来自协议附录一) ====================

// CRC8 校验部分
//crc8 generator polynomial:G(x)=x8+x5+x4+1
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
{
0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

/**
  * @brief 计算 CRC8 校验值
  * @param pchMessage: 数据指针
  * @param dwLength: 数据长度
  * @param ucCRC8: CRC8 初始值
  * @return 计算出的 CRC8 值
  */
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8)
{
    unsigned char ucIndex;
    while (dwLength--)
    {
        ucIndex = ucCRC8^(*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return(ucCRC8);
}

/**
  * @brief 验证 CRC8
  * @param pchMessage: 数据指针
  * @param dwLength: 数据长度 (包含 CRC8 字节)
  * @return 1: 校验通过, 0: 校验失败
  */
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2)) return 0;
    ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
    return ( ucExpected == pchMessage[dwLength-1] );
}

/**
  * @brief 追加 CRC8 到数据末尾
  * @param pchMessage: 数据指针
  * @param dwLength: 数据总长度 (需要留出最后1个字节放 CRC8)
  */
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2)) return;
    ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
    pchMessage[dwLength-1] = ucCRC;
}

// CRC16 校验部分
uint16_t CRC16_INIT = 0xffff;
const uint16_t wCRC_Table[256] = {
0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/**
  * @brief 计算 CRC16 校验值
  * @param pchMessage: 数据指针
  * @param dwLength: 数据长度
  * @param wCRC: CRC16 初始值
  * @return 计算出的 CRC16 值
  */
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL)
    {
        return 0xFFFF;
    }
    while(dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}

/**
  * @brief 验证 CRC16
  * @param pchMessage: 数据指针
  * @param dwLength: 数据长度 (包含 CRC16 字节)
  * @return 1: 校验通过, 0: 校验失败
  */
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC16_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/**
  * @brief 追加 CRC16 到数据末尾
  * @param pchMessage: 数据指针
  * @param dwLength: 数据总长度 (需要留出最后2个字节放 CRC16)
  */
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return;
    }
    wCRC = Get_CRC16_Check_Sum ( (uint8_t *)pchMessage, dwLength-2, CRC16_INIT);
    pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}
// ==================================================================================
void Referee_Init(void) {
    memset(&referee_data, 0, sizeof(referee_info_t));

    // 开启接收 DMA 和空闲中断
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    __HAL_DMA_DISABLE(&hdma_usart6_rx);

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(ref_rx_buf[0]);
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(ref_rx_buf[1]);
    hdma_usart6_rx.Instance->NDTR = REF_RX_BUF_SIZE;
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    __HAL_DMA_ENABLE(&hdma_usart6_rx);
}

void USART6_IRQHandler(void) {

    uart6_rx_count++; // 只要硬件产生中断，这个数就会疯涨


    if (huart6.Instance->SR & UART_FLAG_IDLE) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);
        uint16_t rx_len;
        uint8_t current_mem = (hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) ? 1 : 0;

        __HAL_DMA_DISABLE(&hdma_usart6_rx);
        rx_len = REF_RX_BUF_SIZE - hdma_usart6_rx.Instance->NDTR;
        hdma_usart6_rx.Instance->NDTR = REF_RX_BUF_SIZE;

        if (current_mem == 0) hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
        else hdma_usart6_rx.Instance->CR &= ~DMA_SxCR_CT;

        __HAL_DMA_ENABLE(&hdma_usart6_rx);

        if (rx_len > 0) {
            Referee_Data_Parse(ref_rx_buf[current_mem], rx_len);
        }
    }
}

void Referee_Data_Parse(uint8_t *rx_buf, uint16_t len) {
    raw_data_len = len;
    for (int i = 0; i < 10 && i < len; i++) {
        raw_data_dump[i] = rx_buf[i];
    }

    uint16_t parsed_index = 0;

    while (parsed_index < len) {
        if (rx_buf[parsed_index] == REF_HEADER_SOF) {
            frame_header_struct_t *p_header = (frame_header_struct_t *)(&rx_buf[parsed_index]);

            // 确保调用官方校验库时传入总长度 5。官方库底层会自动算前 4 字节去比对第 5 字节。
            if (Verify_CRC8_Check_Sum(&rx_buf[parsed_index], 5) == 0) {
                parsed_index++; continue;
            }

            uint16_t data_len = p_header->data_length;
            uint16_t frame_len = data_len + 9;
            if (parsed_index + frame_len > len) break;

            // 传入 frame_len，官方库自动取前 frame_len-2 进行计算，并与最后2字节比对
            if (Verify_CRC16_Check_Sum(&rx_buf[parsed_index], frame_len) == 0) {
                parsed_index++; continue;
            }

            uint16_t cmd_id = (rx_buf[parsed_index + 6] << 8) | rx_buf[parsed_index + 5];
            uint8_t *data_ptr = &rx_buf[parsed_index + 7];

            // 解析逻辑
            switch (cmd_id) {
            case 0x0001: // 比赛状态
                memcpy(&referee_data.game_status, data_ptr, sizeof(ext_game_status_t));
                break;

            case 0x0101:
                memcpy(&referee_data.place_status, data_ptr, sizeof(ext_place_status_t));
                break;

            case 0x0201: // 机器人性能状态 (血量、上限等)
                memcpy(&referee_data.robot_status, data_ptr, sizeof(ext_game_robot_status_t));
                break;

            case 0x0202: // 实时功率热量
                memcpy(&referee_data.power_heat_data, data_ptr, sizeof(ext_power_heat_data_t));
                break;

            case 0x0203: // 机器人绝对位置
                memcpy(&referee_data.robot_pos, data_ptr, sizeof(ext_game_robot_pos_t));
                break;

            case 0x0206:
                memcpy(&referee_data.huart_robot, data_ptr, sizeof(ext_huart_robot_data_t));
                break;

            case 0x0208:
                memcpy(&referee_data.allow_robot, data_ptr, sizeof(ext_allow_robot_data_t));
                break;

            default:
                break;
            }
            referee_data.last_update_tick = osKernelSysTick();
            parsed_index += frame_len;
        } else {
            parsed_index++;
        }
    }
}

// 主动发送函数，补全双向通信环路
void Referee_Send_Packet(uint16_t cmd_id, uint8_t *data, uint16_t data_len) {
    uint16_t frame_len = data_len + 9;
    if (frame_len > 256) return; // 防止越界


    uint32_t wait_timeout = 0;
    while (huart6.gState != HAL_UART_STATE_READY) {
        osDelay(1); // 挂起 1ms，不占用系统资源
        wait_timeout++;
        if (wait_timeout > 10) {
            // 超时 10ms DMA 依然被占用，直接丢弃本包数据，保护后续逻辑
            return;
        }
    }

    // 1. 填充帧头
    ref_tx_buf[0] = REF_HEADER_SOF;
    ref_tx_buf[1] = data_len & 0xFF;
    ref_tx_buf[2] = (data_len >> 8) & 0xFF;
    ref_tx_buf[3] = tx_seq++;

    // 2. 追加帧头 CRC8 (传参 5，官方库算前 4 位写入第 5 位)
    Append_CRC8_Check_Sum(ref_tx_buf, 5);

    // 3. 填充 CMD ID
    ref_tx_buf[5] = cmd_id & 0xFF;
    ref_tx_buf[6] = (cmd_id >> 8) & 0xFF;

    // 4. 填充数据段
    memcpy(&ref_tx_buf[7], data, data_len);

    // 5. 追加整帧 CRC16
    Append_CRC16_Check_Sum(ref_tx_buf, frame_len);

    // 6. 使用 DMA 阻塞/非阻塞发送
    HAL_UART_Transmit_DMA(&huart6, ref_tx_buf, frame_len);
}

// 裁判系统调试打印函数 (仪表盘风格)
void Referee_Debug_Print(void) {
    static struct uart_device *uart1 = NULL;
    static uint32_t last_print_tick = 0;

    // 1. 获取封装好的 USART1 DMA 实例
    if (uart1 == NULL) {
        uart1 = uart_get_device("uart1_dma");
    }

    // 2. 降频打印：500ms 打印一次 (2Hz)
    if (osKernelSysTick() - last_print_tick > 500) {
        if (uart1 != NULL) {
            if (referee_data.last_update_tick != 0) {

                // 提取场地状态的 bit23-24
                uint8_t place_bits = (referee_data.place_status.place_t >> 23) & 0x03;

                // 如果接收到了数据，打印多行仪表盘
                uart1->Print(uart1,
                "========== REF GATEWAY DASHBOARD ==========\r\n"
                "[Game] Prog: %d | Time: %d s | Place: %d\r\n"
                "[Stat] HP: %d | Heat17: %d | Buf: %d J\r\n"
                "[Cmbt] Allow17: %d | ArmorID: %d | Hurt: %d\r\n"
                "-------------------------------------------\r\n"
                "[Test] UART_Cnt: %d | Raw0-3: %02X %02X %02X %02X\r\n"
                "===========================================\r\n\r\n",
                referee_data.game_status.game_progress,
                referee_data.game_status.stage_remain_time,
                place_bits,
                referee_data.robot_status.current_HP,
                referee_data.power_heat_data.shooter_17mm_barrel_heat,
                referee_data.power_heat_data.buffer_energy,
                referee_data.allow_robot.allow_bullet_17,
                referee_data.huart_robot.armor_id,
                referee_data.huart_robot.HP_deducation_reason,
                // 底层测试数据保留：
                uart6_rx_count,
                raw_data_dump[0], raw_data_dump[1], raw_data_dump[2], raw_data_dump[3]
                );
            } else {
                uart1->Print(uart1, "[REF RAW] Cnt: %d | Len: %d | Data: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                uart6_rx_count, raw_data_len,
                raw_data_dump[0], raw_data_dump[1], raw_data_dump[2], raw_data_dump[3],
                raw_data_dump[4], raw_data_dump[5], raw_data_dump[6], raw_data_dump[7]);
            }
        }
        last_print_tick = osKernelSysTick();
    }
}

void Referee_CAN_Forward(void) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t send_mail_box;

    // ==========================================
    // 第一帧 (ID: 0x101) - 核心高频数据 (8 字节满载)
    // ==========================================

    // 1. 配置 CAN 发送头
    tx_header.StdId = 0x101;           // 我们自定义的“车牌号” ID
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;        // 标准帧
    tx_header.RTR = CAN_RTR_DATA;      // 数据帧
    tx_header.DLC = 8;                 // 数据长度：严格 8 字节
    tx_header.TransmitGlobalTime = DISABLE;

    // 2. 数据拆解打包 (利用位移操作，把 16位 拆成两个 8位)
    // 注意：这里采用大端序(高位在前)，接收端也必须按照这个顺序还原
    tx_data[0] = (referee_data.robot_status.current_HP >> 8) & 0xFF;
    tx_data[1] = referee_data.robot_status.current_HP & 0xFF;

    tx_data[2] = (referee_data.power_heat_data.shooter_17mm_barrel_heat >> 8) & 0xFF;
    tx_data[3] = referee_data.power_heat_data.shooter_17mm_barrel_heat & 0xFF;

    tx_data[4] = (referee_data.power_heat_data.buffer_energy >> 8) & 0xFF;
    tx_data[5] = referee_data.power_heat_data.buffer_energy & 0xFF;

    tx_data[6] = (referee_data.game_status.stage_remain_time >> 8) & 0xFF;
    tx_data[7] = referee_data.game_status.stage_remain_time & 0xFF;

    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &send_mail_box);


    // ==========================================
    // 第二帧 (ID: 0x102) - 弹药与事件数据 (4 字节有效，其余补0)
    // ==========================================
    tx_header.StdId = 0x102;
    uint8_t tx_data2[8] = {0}; // 必须清零，保证后4字节干净

    // Byte 0-1: 17mm 允许发弹量
    tx_data2[0] = (referee_data.allow_robot.allow_bullet_17 >> 8) & 0xFF;
    tx_data2[1] = referee_data.allow_robot.allow_bullet_17 & 0xFF;

    // Byte 2: 【碎片合并】装甲板ID(高4位) | 扣血原因(低4位)
    tx_data2[2] = ((referee_data.huart_robot.armor_id & 0x0F) << 4) |
                  (referee_data.huart_robot.HP_deducation_reason & 0x0F);

    // Byte 3: 【碎片合并】场地bit23-24 (高2位) | 比赛进度(低4位)
    // 先把 place_t 右移23位，只取最后两位的状态 (0x03)
    uint8_t place_status_bits = (referee_data.place_status.place_t >> 23) & 0x03;
    tx_data2[3] = (place_status_bits << 4) |
                  (referee_data.game_status.game_progress & 0x0F);

    // 塞进 CAN2 邮箱
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data2, &send_mail_box);

}