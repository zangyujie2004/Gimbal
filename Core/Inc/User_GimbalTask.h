//
// Created by Administrator on 24-11-30.
//

#ifndef USER_GIMBALTASK_H
#define USER_GIMBALTASK_H

#include "User_MotorDevice.h"
#include "User_Main.h"

class MotorAnglePitch : public MotorAngle
{
public:
    float angleMin;   // 最小角度
    float angleMax;   // 最大角度

    // 构造函数，去除前馈控制参数
    MotorAnglePitch(MotorType_t* pMotorType, PID* pPidSpeed, PID* pPidAngle, float pAngleMin, float pAngleMax);

    // 更新控制
    void updateControl() override;

    // 设置角度
    void setAngle(float angle) override;

    // 增加角度
    void addToAngle(float deltaAngle);
};


void GimbalTaskInit();
void GimbalTaskRoutine();

#endif //USER_GIMBALTASK_H

