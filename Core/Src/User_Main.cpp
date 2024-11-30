//
// Created by Administrator on 24-11-23.
//
#include "User_Main.h"
#include "User_RemoteDevice.h"
#include "User_CanDevice.h"
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
    CANDeviceInit();
}
void MainTaskInit()
{
    //remoteControlTaskInit();
    //gimbalTaskInit();
}

void mainDeviceRoutine()
{
    canDeviceRoutine();
    HAL_IWDG_Refresh(&hiwdg);
}

