//
// Created by Administrator on 24-11-23.
//

#ifndef USER_CANDEVICE_H
#define USER_CANDEVICE_H

#include "can.h"

namespace CanRx
{
    void handle(CAN_HandleTypeDef* hcan);
}

void CanDeviceInit();
void CanDeviceRoutine();
#endif //USER_CANDEVICE_H
