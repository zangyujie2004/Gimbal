//
// Created by Administrator on 24-11-23.
//

#ifndef USER_MAIN_H
#define USER_MAIN_H

#ifdef __cplusplus
extern "C"{
#endif

#include "main.h"

    void MainInit();
    void MainDeviceInit();
    void mainDeviceRoutine();
    void MainTaskInit();
    void mainTaskRoutine();

#ifdef __cplusplus
}
#endif

#endif //USER_MAIN_H
