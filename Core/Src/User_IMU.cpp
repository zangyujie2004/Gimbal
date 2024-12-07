//
// Created by Administrator on 24-12-7.
//

#include "User_IMU.h"
#include "gpio.h"
#include "spi.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

// 定义常量
#define PI 3.1416f

// IMU 数据缓存
static int16_t accelData_buffer[3];
static float accelValue_buffer[3];
static int16_t gyroData_buffer[3];
static float gyroValue_buffer[3];

// 姿态角变量
static float pitch = 0.0f; // 俯仰角
static float roll = 0.0f;  // 横滚角
static float yaw = 0.0f;   // 偏航角（不使用加速度计校正）
// 时间步长（秒）
static float dt = 0.1f; // 根据实际定时器配置调整
// 互补滤波器系数
static float alpha = 0.98f;

// 获取姿态角度函数实现
extern "C" float get_pitch(void) {
    return pitch;
}

extern "C" float get_roll(void) {
    return roll;
}

extern "C" float get_yaw(void) {
    return yaw;
}

extern "C" {

// 配置函数实现
void BMI088_ACCEL_NS_L() {
    HAL_GPIO_WritePin(Acc_GPIO_Port, Acc_Pin, GPIO_PIN_RESET);
}

void BMI088_ACCEL_NS_H() {
    HAL_GPIO_WritePin(Acc_GPIO_Port, Acc_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L() {
    HAL_GPIO_WritePin(Gyro_GPIO_Port, Gyro_Pin, GPIO_PIN_RESET);
}

void BMI088_GYRO_NS_H() {
    HAL_GPIO_WritePin(Gyro_GPIO_Port, Gyro_Pin, GPIO_PIN_SET);
}

// 读取和写入寄存器实现
void BMI088_ReadReg_ACCEL(uint8_t reg, uint8_t *return_data, uint8_t length) {
    BMI088_ACCEL_NS_L();
    reg = reg | 0x80; // 设置最高位为1，表示读取操作
    HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    HAL_SPI_Receive(&hspi1, return_data, length, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
    BMI088_ACCEL_NS_H();
}

void BMI088_ReadReg_GYRO(uint8_t reg, uint8_t *return_data, uint8_t length) {
    BMI088_GYRO_NS_L();
    reg = reg | 0x80; // 设置最高位为1，表示读取操作
    HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    HAL_SPI_Receive(&hspi1, return_data, length, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
    BMI088_GYRO_NS_H();
}

void BMI088_WriteReg(uint8_t reg, uint8_t write_data) {
    BMI088_ACCEL_NS_L();
    reg = reg & 0x7F; // 写操作，确保最高位为0
    HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    HAL_SPI_Transmit(&hspi1, &write_data, 1, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    BMI088_ACCEL_NS_H();
}

// 初始化 BMI088 实现
void BMI088_Init() {
    // Soft Reset ACCEL
    BMI088_ACCEL_NS_L();
    BMI088_WriteReg(0x7E, 0xB6); // 写入 0xB6 到 ACC_SOFTRESET(0x7E)
    HAL_Delay(1);
    BMI088_ACCEL_NS_H();

    // Soft Reset GYRO
    BMI088_GYRO_NS_L();
    BMI088_WriteReg(0x14, 0xB6); // 写入 0xB6 到 GYRO_SOFTRESET(0x14)
    HAL_Delay(30);
    BMI088_GYRO_NS_H();

    // 切换 ACCEL 到正常模式
    BMI088_ACCEL_NS_L();
    HAL_Delay(1);
    BMI088_WriteReg(0x7D, 0x04); // 写入 0x04 到 ACC_PWR_CTRL(0x7D)
    HAL_Delay(1);
    BMI088_ACCEL_NS_H();
}

// 用户初始化实现
void BMI088_UserInit(uint8_t *return_data, uint8_t *range) {
    BMI088_ReadReg_GYRO(0x00, return_data, 1);
    BMI088_WriteReg(0x41, 0x02);
    HAL_Delay(10);
    BMI088_ReadReg_ACCEL(0x41, range, 1);
}

// 读取加速度数据实现
void BMI088_ReadAccelData(int16_t *accelData_out, float *accelValue_out) {
    uint8_t rawData[6];
    BMI088_ReadReg_ACCEL(0x12, rawData, 6); // 0x12 是 ACCEL 数据寄存器地址

    accelData_out[0] = (rawData[1] << 8) | rawData[0];
    accelData_out[1] = (rawData[3] << 8) | rawData[2];
    accelData_out[2] = (rawData[5] << 8) | rawData[4];

    uint8_t rangeReg;
    BMI088_ReadReg_ACCEL(0x41, &rangeReg, 1); // 读取加速度计范围寄存器
    float scaleFactor = 3.0f / 32768.0f; // 默认 3g
    switch (rangeReg) {
        case 0x00: scaleFactor = 3.0f / 32768.0f; break;
        case 0x01: scaleFactor = 6.0f / 32768.0f; break;
        case 0x02: scaleFactor = 12.0f / 32768.0f; break;
        case 0x03: scaleFactor = 24.0f / 32768.0f; break;
        default: scaleFactor = 3.0f / 32768.0f; break; // 默认 3g
    }

    accelValue_out[0] = accelData_out[0] * scaleFactor;
    accelValue_out[1] = accelData_out[1] * scaleFactor;
    accelValue_out[2] = accelData_out[2] * scaleFactor;
}

// 读取陀螺仪数据实现
void BMI088_ReadGyroData(int16_t *gyroData_out, float *gyroValue_out) {
    uint8_t rawData[6];
    BMI088_ReadReg_GYRO(0x02, rawData, 6); // 0x02 是 GYRO 数据寄存器地址

    gyroData_out[0] = (rawData[1] << 8) | rawData[0];
    gyroData_out[1] = (rawData[3] << 8) | rawData[2];
    gyroData_out[2] = (rawData[5] << 8) | rawData[4];

    float gyroScaleFactor = 2000.0f / 32768.0f * PI / 180.0f; // 转换到弧度系统
    gyroValue_out[0] = gyroData_out[0] * gyroScaleFactor;
    gyroValue_out[1] = gyroData_out[1] * gyroScaleFactor;
    gyroValue_out[2] = gyroData_out[2] * gyroScaleFactor;
}

}

// 姿态计算函数实现
void compute_attitude(void) {
    // 计算加速度计的俯仰角和横滚角
    float accel_pitch = atan2(accelValue_buffer[1], sqrt(accelValue_buffer[0] * accelValue_buffer[0] + accelValue_buffer[2] * accelValue_buffer[2])) * 180.0f / PI;
    float accel_roll = atan2(-accelValue_buffer[0], accelValue_buffer[2]) * 180.0f / PI;

    // 使用陀螺仪数据进行角度积分
    pitch += gyroValue_buffer[0] * dt * 180.0f / PI; // 假设 gyroValue[0] 是绕 X 轴的角速度
    roll += gyroValue_buffer[1] * dt * 180.0f / PI;  // 假设 gyroValue[1] 是绕 Y 轴的角速度
    yaw += gyroValue_buffer[2] * dt * 180.0f / PI;   // 假设 gyroValue[2] 是绕 Z 轴的角速度

    // 应用互补滤波器
    pitch = alpha * pitch + (1.0f - alpha) * accel_pitch;
    roll = alpha * roll + (1.0f - alpha) * accel_roll;
}
