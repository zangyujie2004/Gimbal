//
// Created by Administrator on 24-12-7.
//

#ifndef USER_MOTOR_H
#define USER_MOTOR_H

#include "can.h"
#include "User_PID.h"
#include "stm32f4xx.h"

enum DJIMotorType { MOTOR_TYPE_NONE = 0, M3508, M2006, GM6020 };
enum MotorPIDType { PID_TYPE_NONE = 0, SINGLE_SPEED, DOUBLE_ANGLE };
enum MotorID {
    MOTOR_ID_NONE = 0,
    MOTOR_ID_1 = 1,
    MOTOR_ID_2,
    MOTOR_ID_3,
    MOTOR_ID_4,
    MOTOR_ID_5,
    MOTOR_ID_6,
    MOTOR_ID_7,
    MOTOR_ID_8
};

enum MotorStatus { STOP = 0, RUNNING };

class DJIMotor {
public:
    DJIMotor();
    DJIMotor(
        DJIMotorType DJI_motor_type,
        MotorID id,
        PID speed_pid,
        PID angle_pid,
        MotorPIDType motor_pid_type,
        float init_angle
    );

    void data_process(uint8_t data[8]);

    void set_speed(uint16_t speed);

    void set_angle(float angle);

    void stop();

    void handle();

    uint16_t rx_id() const;

private:
    MotorStatus motor_status;

    DJIMotorType motor_type;
    MotorID motor_id;

    MotorPIDType pid_type;

    PID motor_speed_pid; // 速度环PID

    PID motor_angle_pid; // 角度环PID

    struct {
        int16_t speed_ref;
        int16_t angle_ref;
    } pid_ref;

    float reduction_ratio; // 电机减速比

    uint16_t encoder_angle; // 当前电机编码器角度 range:[0,8191]
    uint16_t last_encoder_angle; // 上次电机编码器角度 range:[0,8191]
    int16_t delta_encoder_angle; // 编码器端新转动的角度
    int32_t total_encoder_angle; // 编码器转过的总角度
    int32_t round_cnt; // 转过的总圈数

    float rotate_speed; // rpm 反馈转子转速
    float current; // A 反馈转矩电流
    float temp; // °C 反馈电机温度

    CAN_TxHeaderTypeDef tx_header;
    uint16_t rx_message_id;
};

#endif //USER_MOTOR_H
