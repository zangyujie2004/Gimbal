//
// Created by Administrator on 24-12-7.
//

#ifndef USER_IMU_H
#define USER_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

    // 函数声明
    void BMI088_Init(void);
    void BMI088_ACCEL_NS_L(void);
    void BMI088_ACCEL_NS_H(void);
    void BMI088_GYRO_NS_L(void);
    void BMI088_GYRO_NS_H(void);
    void BMI088_UserInit(uint8_t *return_data, uint8_t *range);
    void BMI088_ReadAccelData(int16_t *accelData, float *accelValue);
    void BMI088_ReadGyroData(int16_t *gyroData, float *gyroValue);
    void compute_attitude(void); // 姿态计算函数

    // 获取姿态角度
    float get_pitch(void);
    float get_roll(void);
    float get_yaw(void);

#ifdef __cplusplus
}
#endif


#endif //USER_IMU_H
