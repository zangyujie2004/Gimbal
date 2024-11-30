//
// Created by Administrator on 24-11-23.
//

#include "User_RemoteDevice.h"
#include "usart.h"
#include "User_LinearMapping.h"

uint8_t rcRxBuf[RC_RX_BUF_SIZE];
uint8_t rcRxData[RC_RX_DATA_SIZE];

Remote remote;  // 创建 Remote 类对象

void RemoteDeviceInit() {
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);  // 启用UART接收空闲中断
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rcRxBuf, RC_RX_BUF_SIZE);  // 启动DMA接收
    remote.init();  // 初始化遥控器
    memset(rcRxBuf, 0, sizeof(rcRxBuf));  // 清空缓冲区
    memset(rcRxData, 0, sizeof(rcRxData));  // 清空接收数据
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart3)
    {
        remote.frameHandle();
    }
}

void Remote::init()
{
    channel_.l_col = 0;
    channel_.l_row = 0;
    channel_.r_col = 0;
    channel_.r_row = 0;
    switch_.l = MID_POS;
    switch_.r = MID_POS;
}

void Remote::frameHandle()
{
    channel_.r_row = linearMappingInt2Float((rcRxData[0] | (rcRxData[1] << 8)) & 0x07ff, 364, 1684, -1.0, 1.0);
    channel_.r_col = linearMappingInt2Float(((rcRxData[1] >> 3) | (rcRxData[2] << 5)) & 0x07ff, 364, 1684, -1.0,
                                            1.0);
    channel_.l_row = linearMappingInt2Float(((rcRxData[2] >> 6) | (rcRxData[3] << 2) | (rcRxData[4] << 10)) &0x07ff, 364, 1684, -1.0, 1.0);
    channel_.l_col = linearMappingInt2Float( ((rcRxData[4] >> 1) | (rcRxData[5] << 7)) & 0x07ff, 364, 1684, -1.0, 1.0);

    switch_.r = RCSwitchStates[((rcRxData[5] >> 4) & 0x0003) - 1];
    switch_.l = RCSwitchStates[((rcRxData[5] >> 4) & 0x000C) - 1];
}