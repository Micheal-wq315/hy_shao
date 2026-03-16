/*
 * @Description: 
 * @Author: changfeng
 * @brief: 
 * @version: 
 * @Date: 2025-03-13 06:34:23
 * @LastEditors:  
 * @LastEditTime: 2025-03-16 17:17:08
 */
#include "referee_transport.h"

#define START_BYTE 0xAA
#define END_BYTE 0x55
#define REFEREE_DATA_CONTROL_FRAME_SIZE sizeof(referee_info_t)

static referee_info_t referee_data;
static uint8_t referee_init_flag;
static uint8_t game_state;
uint8_t test_buff[128];
static USARTInstance *referee_data_usart_instance;  // 裁判系统数据转发串口实例
static DaemonInstance *referee_data_daemo_instance; // 裁判系统数据转发进程守护实例


/**
 * @brief 裁判系统数据转发解析函数
 * @return 
 */
static void RefereeDataParse(const uint8_t *referee_data_buf)
{   
    if(referee_data_buf[0] != START_BYTE || referee_data_buf[150] != END_BYTE)
    {
        LOGWARNING("[referee] Packet format error");
        return;
    }

    // 计算校验和（前149字节）
    uint8_t checksum = 0;
    for(int i =0; i < 149; i++)
    {
        checksum ^= referee_data_buf[i];
    }

    if(checksum == referee_data_buf[149])   // 校验正确
    {
        memcpy(&referee_data, &referee_data_buf[1], REFEREE_DATA_CONTROL_FRAME_SIZE);
    }
}

/**
 * @brief 裁判系统数据转发回调函数
 * @return 
 */
static void RefereeDataRxCallback()
{
    DaemonReload(referee_data_daemo_instance);   //先喂狗
    RefereeDataParse(referee_data_usart_instance->recv_buff);
    game_state = referee_data.GameState.game_progress;
   
    for(int i = 0; i < 125; i++)
    {
        test_buff[i] = game_state;
    }
    HAL_UART_Transmit_DMA(&huart7, test_buff, sizeof(test_buff));
}

/**
 * @brief 裁判系统数据转发离线回调函数 
 * @return
 */
static void RefereeDataLostCallback()
{
    memset(&referee_data, 0, sizeof(referee_data)); //清空cmd_vel数据
    USARTServiceInit(referee_data_usart_instance);   //尝试重新启动

    LOGWARNING("[referee] referee data lost");
}

/**
 * @brief 初始化裁判系统数据转发串口通信 
 * @return
 */
referee_info_t *RefereeDataTransportInit(UART_HandleTypeDef *referee_data_usart_handle)
{
    USART_Init_Config_s config;
    config.module_callback = RefereeDataRxCallback;
    config.usart_handle = referee_data_usart_handle;
    config.recv_buff_size = (REFEREE_DATA_CONTROL_FRAME_SIZE + 12);
    referee_data_usart_instance = USARTRegister(&config);
    // 进行进程守护的注册，用于定时检查串口是否正常工作
    Daemon_Init_Config_s daemo_conf = {
        .reload_count = 10, //100ms未接收到数据视为离线
        .callback = RefereeDataLostCallback,
        .owner_id = NULL,   //只有一个cmd_vel不需要id
    };
    // cmd_vel_daemo_instance = DaemonRegister(&daemo_conf);
    referee_init_flag = 1;

    return &referee_data;

}

/**
 * @brief 检查裁判系统数据转发是否在线
 * @return 1 表示在线, 0 表示离线
 */
uint8_t RefereeIsOnline()
{
    if(referee_init_flag == 1)
        return DaemonIsOnline(referee_data_daemo_instance);
    return 0;
}


