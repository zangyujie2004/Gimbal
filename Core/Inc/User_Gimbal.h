//
// Created by Administrator on 24-12-7.
//

#ifndef USER_GIMBAL_H
#define USER_GIMBAL_H

#include "User_Motor.h"
#include "User_PID.h"

class Gimbal {
public:
    Gimbal();

    void set_pitch_angle(float angle);
    void set_yaw_angle(float angle);

    uint16_t pitch_rx_id();
    uint16_t yaw_rx_id();

    void pitch_data_process(uint8_t data[8]);
    void yaw_data_process(uint8_t data[8]);

    void stop();

    void handle();

    float get_pitch_feedforward_float() const;
    float get_yaw_feedforward_float() const;

private:
    DJIMotor pitch_motor, yaw_motor;
    float pitch_angle, yaw_angle;

    PID create_pid(float kp, float ki, float kd, float i_max, float out_max);
};
#endif //USER_GIMBAL_H
