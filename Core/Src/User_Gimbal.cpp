//
// Created by Administrator on 24-12-7.
//
#include "User_Gimbal.h"
#include "User_PID.h"
#include "User_Motor.h"
#define PITCH_MOTOR_TYPE GM6020
#define YAW_MOTOR_TYPE GM6020
#define PITCH_MOTOR_ID MOTOR_ID_1
#define YAW_MOTOR_ID MOTOR_ID_3

PID Gimbal::create_pid(float kp, float ki, float kd, float i_max, float out_max) {
    PIDInitStruct pid_init_struct;
    pid_init_struct._kp = kp;
    pid_init_struct._ki = ki;
    pid_init_struct._kd = kd;
    pid_init_struct._i_max = i_max;
    pid_init_struct._out_max = out_max;
    return PID(pid_init_struct);
}

Gimbal::Gimbal() {
    PID pitch_speed_pid = create_pid(150, 0, 0, 0, 10000);
    PID pitch_angle_pid = create_pid(1, 0.08, 0.6, 100, 180);

    PID yaw_speed_pid = create_pid(150, 0, 0, 0, 10000);
    PID yaw_angle_pid = create_pid(0.3, 0.03, 0, 100, 180);

    pitch_motor = DJIMotor(
        PITCH_MOTOR_TYPE, PITCH_MOTOR_ID,
        pitch_speed_pid, pitch_angle_pid,
        DOUBLE_ANGLE, 0x1650
    );

    yaw_motor = DJIMotor(
        YAW_MOTOR_TYPE, YAW_MOTOR_ID,
        yaw_speed_pid, yaw_angle_pid,
        DOUBLE_ANGLE, 0
    );

    pitch_angle = 0;
    yaw_angle = 0;
}

void Gimbal::set_pitch_angle(float angle) {
    pitch_angle = angle;
}

void Gimbal::set_yaw_angle(float angle) {
    yaw_angle = angle;
}

uint16_t Gimbal::pitch_rx_id() {
    return pitch_motor.rx_id();
}

uint16_t Gimbal::yaw_rx_id() {
    return yaw_motor.rx_id();
}

void Gimbal::pitch_data_process(uint8_t data[8]) {
    pitch_motor.data_process(data);
}

void Gimbal::yaw_data_process(uint8_t data[8]) {
    yaw_motor.data_process(data);
}

void Gimbal::stop() {
    pitch_motor.stop();
    yaw_motor.stop();
}

void Gimbal::handle() {
    pitch_motor.set_angle(pitch_angle);
    yaw_motor.set_angle(yaw_angle);

    pitch_motor.handle();
    yaw_motor.handle();
}

float Gimbal::get_pitch_feedforward_float() const {
    return pitch_motor.get_feedforward_float();
}

float Gimbal::get_yaw_feedforward_float() const {
    return yaw_motor.get_feedforward_float();
}
