//
// Created by Sunny on 24-11-23.
//

// User_RemoteDevice.h

#ifndef USER_REMOTEDEVICE_H
#define USER_REMOTEDEVICE_H

#include "main.h"
#include <cstring>

const int RC_RX_BUF_SIZE = 18;
const int RC_RX_DATA_SIZE = 18;

// 遥控器开关的状态
typedef enum { UP_POS, DOWN_POS, MID_POS } RCSwitchState_e;
const RCSwitchState_e RCSwitchStates[] = { UP_POS, DOWN_POS, MID_POS };

// 左右摇杆和开关的控制数据
struct RCChannel {
    float r_row;  // 右摇杆竖直方向
    float r_col;  // 右摇杆水平方向
    float l_row;  // 左摇杆竖直方向
    float l_col;  // 左摇杆水平方向
};

// 开关状态
struct RCSwitch {
    RCSwitchState_e l;  // 左开关
    RCSwitchState_e r;  // 右开关
};

class Remote {
public:
    RCChannel channel_;
    RCSwitch switch_;
    uint8_t keys_;

    void init();           // 初始化遥控器
    void frameHandle();    // 处理接收到的数据
};

// 初始化遥控器设备
void RemoteDeviceInit();

#endif // USER_REMOTEDEVICE_H
