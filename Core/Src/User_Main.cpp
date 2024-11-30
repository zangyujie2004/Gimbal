//
// Created by Administrator on 24-11-23.
//
#include "User_Main.h"
#include "User_RemoteDevice.h"
#include "User_RemoteTask.h"
#include "User_CanDevice.h"
#include "User_MotorDevice.h"
#include "User_GimbalTask.h"
#include "tim.h"
#include "iwdg.h"


void MainInit()
{
    MainDeviceInit();
    MainTaskInit();
    HAL_TIM_Base_Start_IT(&htim6);
    while(1)
    {

    }
}
void MainDeviceInit()
{
    RemoteDeviceInit();
    CanDeviceInit();
}
void MainTaskInit()
{
    RemoteTaskInit();
    GimbalTaskInit();
}

void MainRoutine()
{
    MainDeviceRoutine();
    MainTaskRoutine();
}

void MainDeviceRoutine()
{
    MotorDeviceRoutine();
    CanDeviceRoutine();
    HAL_IWDG_Refresh(&hiwdg);
}

void MainTaskRoutine()
{
    RemoteTaskRoutine();
    GimbalTaskRoutine();
}
