//
// Created by Administrator on 24-11-30.
//

#include "User_GimbalTask.h"
#include "User_MotorDevice.h"
#include "User_Main.h"

// 电机俯仰角度控制类的构造函数
MotorAnglePitch::MotorAnglePitch(MotorType_t* pMotorType, PID* pPidSpeed, PID* pPidAngle, float pAngleMin, float pAngleMax)
    : MotorAngle(pMotorType, pPidSpeed, pPidAngle)
{
    angleMin = pAngleMin;
    angleMax = pAngleMax;
}

// 更新电机控制输出
void MotorAnglePitch::updateControl()
{
    // 如果电机停止，输出强度为 0
    if (stopFlag)
    {
        outputIntensity = 0;
        return;
    }

    // 计算角度反馈的PID输出
    controlSpeed.targetValue = controlAngle.compute(state.angle);  // 设置速度目标
    outputIntensity = controlSpeed.compute(state.speed);  // 只计算PID控制输出，无前馈
}

// 辅助函数：限制数值在指定范围内
inline float clampInRange(float in, float in_min, float in_max)
{
    if (in < in_min) return in_min;
    if (in > in_max) return in_max;
    return in;
}

// 设置目标角度，限制在最小和最大角度之间
void MotorAnglePitch::setAngle(float angle)
{
    // 限制角度在软件限位范围内
    if (angle >= angleMin && angle <= angleMax)
        controlAngle.targetValue = angle;
    else
        controlAngle.targetValue = clampInRange(angle, angleMin, angleMax);
}

// 增加角度并限制在最小和最大角度之间
void MotorAnglePitch::addToAngle(float deltaAngle)
{
    // 调整目标角度，并进行限位
    controlAngle.targetValue = clampInRange(controlAngle.targetValue + deltaAngle, angleMin, angleMax);
}

// 定义 PID 控制器对象，使用合适的参数进行初始化
PID pidPitchSpeed(10.0f, 0.0f, 0.0f, 25000.0f, 25000.0f);  // 速度控制的 PID
PID pidPitchAngle(10.0f, 0.0f, 0.0f, 25000.0f, 25000.0f);  // 角度控制的 PID

MotorAnglePitch motorPitch(&gm6020_v, &pidPitchSpeed, &pidPitchAngle, -62.5, 0);

// 电机初始化函数
void GimbalTaskInit()
{
    // 初始化 motorPitch，设置电机控制器和CAN ID
    motorSet.Append(&motorPitch, 1, 5);  // 假设 motorPitch 是预定义的电机对象
}

// 电机控制任务
void GimbalTaskRoutine()
{
    // 执行电机控制更新
    for (auto motorPtr : motorSet)
    {
        // 更新电机控制
        motorPtr->updateControl();

        // 如果电机需要停止，则调用 Stop()
        if (motorPtr->stopFlag)
        {
            motorPtr->Stop();
        }
        else
        {
            // 启动电机
            motorPtr->Start();
        }
    }
}

