//
// Created by Administrator on 24-11-23.
//

#include "usart.h"
#include "main.h"
#include "tim.h"
#include "can.h"
#include "User_Main.h"
#include "User_RemoteDevice.h"
#include "User_CanDevice.h"

extern uint8_t rcRxBuf[RC_RX_BUF_SIZE];
extern uint8_t rcRxData[RC_RX_DATA_SIZE];
extern Remote remote;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if(htim == &htim6)
    {
        MainDeviceRoutine();
        MainTaskRoutine();
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
    if(hcan == &hcan1 || hcan == &hcan2)
    {
        CanRx::handle(hcan);
    }
}

