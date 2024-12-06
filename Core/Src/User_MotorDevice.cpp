//
// Created by Administrator on 24-11-30.
//

#include "User_MotorDevice.h"

// 电机构造函数
Motor::Motor(MotorType_t* pMotorType)
{
    motorType = pMotorType;
    outputIntensity = 0;
    stopFlag = true;
}

MotorSpeed::MotorSpeed(MotorType_t* pMotorType, PID* pPidSpeed) : Motor(pMotorType), controlSpeed(*pPidSpeed) {
    // controlSpeed 通过构造函数初始化，不需要再赋值
}

MotorAngle::MotorAngle(MotorType_t* pMotorType, PID* pPidSpeed, PID* pPidAngle)
    : MotorSpeed(pMotorType, pPidSpeed), controlAngle(*pPidAngle) {
    // controlAngle 通过构造函数初始化，不需要再赋值
}


// 设置硬件信息
void Motor::setHardwareInfo(uint8_t pCanLine, uint8_t pControllerId)
{
    hardwareInfo.canLine = pCanLine;
    hardwareInfo.controllerId = pControllerId;
}

// 处理控制器反馈数据
void Motor::controllerRxHandle(uint8_t* data)
{
    feedback.angle = data[0]<<8 | data[1];
    feedback.speed = data[2]<<8 | data[3];
    feedback.moment = data[4]<<8 | data[5];
    feedback.temperature = data[6];
}

// 计算角度变化，考虑溢出情况
int32_t calculateAngleDifference(int16_t thisAngle, int16_t lastAngle)
{
    if (thisAngle <= lastAngle) {
        return (lastAngle - thisAngle > 4096) ? (thisAngle + 8192 - lastAngle) : -(lastAngle - thisAngle);
    } else {
        return (thisAngle - lastAngle > 4096) ? -(lastAngle + 8192 - thisAngle) : (thisAngle - lastAngle);
    }
}

// 更新电机状态（包括角度、速度、温度等）
void Motor::updateState()
{
    state.speed = feedback.speed / motorType->reductionRatio;
    state.temperature = feedback.temperature;

    // 计算角度变化，考虑角度溢出
    state.angleInt += calculateAngleDifference(feedback.angle, feedback.lastFeedbackAngle);
    feedback.lastFeedbackAngle = feedback.angle;

    state.angle = state.angleInt * 0.0439453125f / motorType->reductionRatio;
}

// 电机控制更新
void Motor::updateControl()
{
    outputIntensity = 0;   // 基类默认不做控制
}

// 电机速度更新
void MotorSpeed::updateControl()
{
    if (stopFlag)
    {
        outputIntensity = 0;
        return;
    }
    outputIntensity = controlSpeed.compute(state.speed);
}

// 设置目标速度
void MotorSpeed::setSpeed(float speed)
{
    controlSpeed.targetValue = speed;
}

// 电机角度更新
void MotorAngle::updateControl()
{
    if (stopFlag)
    {
        outputIntensity = 0;
        return;
    }
    controlSpeed.targetValue = controlAngle.compute(state.angle);
    outputIntensity = controlSpeed.compute(state.speed);
}

// 设置目标角度
void MotorAngle::setAngle(float angle)
{
    controlAngle.targetValue = angle;
}

// 电机归零
void MotorAngle::zeroSet()
{
    state.angleInt = 0;
    state.angle = 0;
}


// 电机集合构造函数
MotorSet::MotorSet() : size(0) {}

// 追加电机到集合
void MotorSet::Append(Motor* motorPtr, uint8_t canLine, uint8_t controllerId)
{
    if (motorMap[canLine-1][controllerId-1] != nullptr)
    {
        return; // 如果已经存在，返回
    }
    motorPtr->setHardwareInfo(canLine, controllerId);
    motorMap[canLine-1][controllerId-1] = motorPtr;
    motorList[size++] = motorPtr;
}

// 根据 CAN 总线和控制器 ID 获取电机
Motor* MotorSet::getMotorById(uint8_t canLine, uint8_t controllerId)
{
    return motorMap[canLine-1][controllerId-1];
}

// 电机集合迭代器实现
MotorSet::Iterator::Iterator(Motor** pPtr) : ptr(pPtr) {}
Motor*& MotorSet::Iterator::operator * () const
{
    return *ptr;
}
MotorSet::Iterator& MotorSet::Iterator::operator ++ ()
{
    ++ptr;
    return *this;
}
bool MotorSet::Iterator::operator == (const Iterator& other) const
{
    return ptr == other.ptr;
}
bool MotorSet::Iterator::operator != (const Iterator& other) const
{
    return ptr != other.ptr;
}
MotorSet::Iterator MotorSet::begin()
{
    return Iterator(motorList);
}

MotorSet::Iterator MotorSet::end()
{
    return Iterator(motorList + size);
}

MotorSet motorSet;

// 电机设备初始化
void MotorDeviceInit()
{
    // 可以在此添加设备初始化代码
}

// 电机设备例行操作
void MotorDeviceRoutine()
{
    for (auto motorPtr : motorSet)
    {
        motorPtr->updateState();
        motorPtr->updateControl();
    }
}


//电机实例参考
MotorType_t m3508 {
    .reductionRatio = 19.203209f,
    .intensityLimit = 20.0f,
    .intensityDataRatio = 819.2f,
    .canTxPosInd = { 0, 2, 4, 6, 0, 2, 4, 6, 0, 0, 0, 0},
    .canTxIdList = { 0x200, 0x200, 0x200, 0x200, 0x1FF, 0x1FF, 0x1FF, 0x1FF, 0, 0, 0, 0 }
};
MotorType_t m2006 {
    .reductionRatio = 36.0f,
    .intensityLimit = 10.0f,
    .intensityDataRatio = 1000.0f,
    .canTxPosInd = { 0, 2, 4, 6, 0, 2, 4, 6, 0, 0, 0, 0},
    .canTxIdList = { 0x200, 0x200, 0x200, 0x200, 0x1FF, 0x1FF, 0x1FF, 0x1FF, 0, 0, 0, 0 }
};
MotorType_t gm6020_i {
    .reductionRatio = 1.0f,
    .intensityLimit = 3.0f,
    .intensityDataRatio = 5461.3333f,
    .canTxPosInd = { 0, 0, 0, 0, 0, 2, 4, 6, 0, 2, 4, 6},
    .canTxIdList = { 0, 0, 0, 0, 0x1FE, 0x1FE, 0x1FE, 0x1FE, 0X2FE, 0X2FE, 0X2FE, 0 }
};
MotorType_t gm6020_v {
    .reductionRatio = 1.0f,
    .intensityLimit = 25000.0f,
    .intensityDataRatio = 1.0f,
    .canTxPosInd = { 0, 0, 0, 0, 0, 2, 4, 6, 0, 2, 4, 6},
    .canTxIdList = { 0, 0, 0, 0, 0x1FF, 0x1FF, 0x1FF, 0x1FF, 0x2FF, 0x2FF, 0x2FF, 0 }
};

