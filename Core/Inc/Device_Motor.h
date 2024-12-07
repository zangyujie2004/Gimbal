//
// Created by Administrator on 24-12-7.
//

#ifndef DEVICE_MOTOR_H
#define DEVICE_MOTOR_H

#include "can.h"
#include "User_PID.h"
#include "stm32f4xx.h"

// 电机类型选择
enum class DJIMotorType : uint8_t { None = 0, M3508, M2006, GM6020 };
enum class MotorID : uint8_t {
    None = 0,
    Motor1 = 1,
    Motor2 = 2,
    Motor3 = 3,
    Motor4 = 4,
};

class DJIMotor {
public:
    DJIMotor();
    DJIMotor(
        DJIMotorType motorType,
        MotorID id,
        PID speedPID,
        PID anglePID,
        float initAngle
    );

    void dataProcess(uint8_t data[8]);
    void setSpeed(uint16_t speed);
    void setAngle(float angle);
    void stop();
    void handle();
    uint16_t getRxId() const;

private:
    DJIMotorType type;
    MotorID id;

    MotorPIDType pidType;

    PID speedPIDController; // 速度环PID
    PID anglePIDController; // 角度环PID

    struct {
        int16_t speedRef;
        int16_t angleRef;
    } pidRef;

    float reductionRatio; // 电机减速比

    uint16_t encoderAngle; // 当前电机编码器角度 range:[0,8191]
    uint16_t lastEncoderAngle; // 上次电机编码器角度 range:[0,8191]
    int16_t deltaEncoderAngle; // 编码器端新转动的角度
    int32_t totalEncoderAngle; // 编码器转过的总角度
    int32_t roundCount; // 转过的总圈数

    float rotateSpeed; // rpm 反馈转子转速
    float current; // A 反馈转矩电流
    float temperature; // °C 反馈电机温度

    CAN_TxHeaderTypeDef txHeader;
    uint16_t rxMessageId;
};


#endif //DEVICE_MOTOR_H
