//
// Created by Administrator on 24-11-23.
//
#include "User_CanDevice.h"
#include "cstring"
#include "User_MotorDevice.h"

void CanDeviceInit()
{
    // CAN过滤器配置
    CAN_FilterTypeDef filterConfig = {
        .FilterIdHigh = 0x0000,
        .FilterIdLow = 0x0000,
        .FilterMaskIdHigh = 0x0000,
        .FilterMaskIdLow = 0x0000,
        .FilterFIFOAssignment = CAN_FILTER_FIFO0,
        .FilterBank = 0,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_32BIT,
        .FilterActivation = ENABLE
    };

    // 配置CAN1
    HAL_CAN_ConfigFilter(&hcan1, &filterConfig);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    // 配置CAN2
    HAL_CAN_ConfigFilter(&hcan2, &filterConfig);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

//Flow: 1.读取CAN消息 2.判断CAN总线 3.解析CAN消息ID:控制器的 ID 从 0x200 开始 4.传递数据给电机
namespace CanRx
{
    //CAN_RxHeaderTypeDef header：保存接收到的 CAN 帧的头信息（标准ID、扩展ID、数据长度等）
    CAN_RxHeaderTypeDef header;
    //data[8]：保存接收到的数据（最大 8 字节，CAN 数据字段长度为 8 字节）
    uint8_t data[8];

    //CAN 接收数据的处理函数，它会读取接收到的 CAN 消息，并通过motorSet到对应的电机并调用其controllerRxHandle方法来处理接收到的数据。
    void handle(CAN_HandleTypeDef* hcan)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data);
        uint8_t canLine;
        if(hcan == &hcan1) canLine = 1;
        if(hcan == &hcan2) canLine = 2;

        uint8_t controllerId = header.StdId - 0x200;
        motorSet.getMotorById(canLine, controllerId)->controllerRxHandle(data);
    }
}


//1初始化数据(Inline和headerID) 2.填充数据 3.发送数据HAL_CAN_AddTxMessage(lineList[lineInd], &headerTemplate, data[lineInd][headerInd], &mailbox);
namespace CanTx
{
    //headerIdList保存了我们要发送的 CAN 消息的 ID 列表
    const uint8_t headerIdCnt = 5;
    uint16_t headerIdList[headerIdCnt] = {0x200, 0x1FF, 0x1FE, 0x2FE, 0x2FF};

    // 查找目标ID的索引
    inline uint8_t findHeaderIndById(uint16_t targetId)
    {
        for (int i = 0; i < headerIdCnt; i++)
            if (headerIdList[i] == targetId)
                return i;
        return headerIdCnt;  // 如果没找到，返回 headerIdCnt
    }
    //headerTemplate定义了 CAN 消息的基本结构（ID、数据长度、传输时间等
    CAN_TxHeaderTypeDef headerTemplate = {
        .StdId = 0x000,
        .ExtId = 0x000,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE
    };

    CAN_HandleTypeDef* lineList[] = { &hcan1, &hcan2 };
    uint32_t mailbox;
    uint8_t data[2][headerIdCnt][8]; // can line index, header index, data index
    uint8_t frameActivated[2][headerIdCnt]; //标记每个帧是否被激活，即是否需要发送

    // 初始化数据
    inline void init()
    {
        memset(data, 0, sizeof(data));
        memset(frameActivated, 0, sizeof(frameActivated));
    }

    // 发送CAN数据
    void transmit()
    {
        for (uint8_t headerInd = 0; headerInd < headerIdCnt; headerInd++)
        {
            for (uint8_t lineInd = 0; lineInd < 2; lineInd++)
            {
                if (!frameActivated[lineInd][headerInd])
                    continue;

                //等待空闲：确保 CAN 邮箱不忙碌
                uint32_t timeout = 1000;  // 设置一个超时值（ms）
                while (HAL_CAN_IsTxMessagePending(lineList[lineInd], mailbox) && timeout > 0)
                {
                    timeout--;
                    HAL_Delay(1); // 等待 1 毫秒
                }

                if (timeout == 0) {
                    // 超时，进行错误处理
                    return;
                }

                headerTemplate.StdId = headerIdList[headerInd];
                HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(lineList[lineInd], &headerTemplate, data[lineInd][headerInd], &mailbox);
                if (status != HAL_OK) {
                    // 发送失败，处理错误
                }
            }
        }
    }

}

void CanDeviceRoutine()
{
    CanTx::init();  // 清空数据缓冲区
    for (auto motorPtr : motorSet)
    {
        uint8_t lineInd = motorPtr->hardwareInfo.canLine - 1;
        uint8_t controllerInd = motorPtr->hardwareInfo.controllerId - 1;
        uint8_t headerInd = CanTx::findHeaderIndById(motorPtr->motorType->canTxIdList[controllerInd]);
        uint8_t dataPos = motorPtr->motorType->canTxPosInd[controllerInd];

        // 缓存数据计算，减少重复访问
        int16_t controlData = motorPtr->outputIntensity * motorPtr->motorType->intensityDataRatio;
        CanTx::data[lineInd][headerInd][dataPos] = controlData >> 8;
        CanTx::data[lineInd][headerInd][dataPos + 1] = controlData & 0xff;

        // 激活当前帧，准备发送
        CanTx::frameActivated[lineInd][headerInd] = true;
    }
    CanTx::transmit();  // 发送所有激活的 CAN 帧
}
