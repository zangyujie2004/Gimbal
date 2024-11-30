//
// Created by Administrator on 24-11-23.
//
#include "User_RemoteDevice.h"
#include "usart.h"
#include "main.h"
#include "tim.h"
#include "can.h"

extern uint8_t rcRxBuf[RC_RX_BUF_SIZE];
extern uint8_t rcRxData[RC_RX_DATA_SIZE];
extern Remote remote;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if(htim == &htim6)
    {
        mainDeviceRoutine();
        mainTaskRoutine();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart == &huart3)
    {
        std::memcpy(rcRxData, rcRxBuf, RC_RX_BUF_SIZE);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rcRxBuf, RC_RX_BUF_SIZE);
        if (Size == RC_RX_BUF_SIZE)
        {
            remote.frameHandle();
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart3)
    {
        remote.frameHandle();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    if (hcan == &hcan1 || hcan == &hcan2)  // 判断是哪个 CAN 总线触发的回调
    {
        CAN_RxHeaderTypeDef header;
        uint8_t data[8];  // CAN 消息数据缓存

        // 获取接收到的消息
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) == HAL_OK)
        {
            // 处理接收到的数据
            uint8_t canLine = (hcan == &hcan1) ? 1 : 2;  // 判断是 hcan1 还是 hcan2
            uint8_t controllerId = header.StdId - 0x200;  // 根据标准 ID 计算控制器 ID

            // 这里假设你有一个 motorSet 或其他结构来处理数据
            // 将接收到的数据传递给相应的处理函数
            motorSet.getMotorById(canLine, controllerId)->controllerRxHandle(data);
        }
    }
}