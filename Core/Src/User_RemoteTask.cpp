//
// Created by Administrator on 24-11-30.
//

#include "User_RemoteTask.h"
#include "User_RemoteDevice.h"
#include "User_MotorDevice.h"
#include "User_RemoteTask.h"
#include "vector"

// 假设有一个存储电机对象的集合
extern Remote remote;  // 外部引用遥控器对象
//extern MotorAnglePitch motorPitch;   // 外部引用pitch电机
//extern MotorYawPitch motorYaw;  // 外部引用yaw电机
//extern std::vector<Motor*> motorSet; // 电机集合

void RemoteTaskInit()
{
    // 初始化遥控器任务，如果有需要可以在此处扩展
}

void RemoteTaskRoutine()
{
    // 1. 判断右开关状态，控制电机启停
    if (remote.switch_.r == DOWN_POS) {
        // 如果右开关为 DOWN_POS，停止所有电机
        for (auto motorPtr : motorSet) {
            motorPtr->Stop();
        }
    } else {
        // 2. 如果右开关不为 DOWN_POS，根据左开关和摇杆的状态来控制电机
        if (remote.switch_.l == UP_POS && remote.switch_.r == UP_POS) {
            // 如果左开关为 UP_POS 且右开关为 UP_POS，启动所有电机
            for (auto motorPtr : motorSet) {
                motorPtr->Start();
            }

            // 3. 使用左摇杆来控制云台的 pitch 和 yaw 角度
            // 左摇杆上下控制 pitch 角度变化
            //motorPitch.addToAngle(remoteControl.channel_.l_row * (-0.05f)); // 根据上下摇杆控制 pitch

            // 左摇杆水平拨动控制 yaw 角度变化
            //motorYaw.addToAngle(remoteControl.channel_.l_col * (-0.05f)); // 根据左右摇杆控制 yaw
        }
    }
}