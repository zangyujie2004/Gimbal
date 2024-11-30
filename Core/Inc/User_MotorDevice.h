//
// Created by Administrator on 24-11-30.
//

#ifndef USER_MOTORDEVICE_H
#define USER_MOTORDEVICE_H

#include "main.h"
#include "User_PID.h"

// 电机类型结构体
struct MotorType_t
{
    float reductionRatio;       // 减速比
    float intensityLimit;       // 强度限制
    float intensityDataRatio;   // 强度数据比

    uint8_t canTxPosInd[12];    // CAN发送位置索引
    uint16_t canTxIdList[12];   // CAN发送ID列表
};

// 电机硬件信息
struct MotorHardwareInfo_t
{
    uint8_t canLine;           // CAN总线编号
    uint8_t controllerId;      // 控制器ID
};

// 控制器接收数据
struct ControllerRx_t
{
    int16_t angle;             // 当前角度
    int16_t speed;             // 当前速度
    int16_t moment;            // 扭矩
    int16_t temperature;       // 温度
    int16_t lastFeedbackAngle;
};

// 电机状态
struct MotorState_t
{
    float speed;              // 速度 (度/秒)
    int32_t angleInt;         // 角度 (原始值)
    float angle;              // 角度 (度)
    float temperature;        // 温度
};

// 电机控制单元 (包含 PID 控制器)
struct MotorControlUnit_t
{
    float targetValue;        // 目标值
    PID pid;                  // PID 控制器

    // 计算控制输出
    float compute(float actualValue)
    {
        return pid.calc(targetValue, actualValue);
    }
};

// 电机基类
class Motor
{
public:
    MotorType_t* motorType;
    MotorHardwareInfo_t hardwareInfo{};   // 硬件信息
    ControllerRx_t feedback{};            // 控制器反馈
    MotorState_t state{};                 //电机目前状态
    float outputIntensity;                 // 输出强度
    uint8_t stopFlag;                      // 停止标志

    //虚析构函数
    virtual ~Motor() = default;
    explicit Motor(MotorType_t* pMotorType);

    void setHardwareInfo(uint8_t pCanLine, uint8_t pControllerId);
    void controllerRxHandle(uint8_t* data);

    void updateState();
    virtual void updateControl();

    void setMotorState(bool stop)
    {
        stopFlag = stop;
    }

    void Stop() { setMotorState(true); }
    void Start() { setMotorState(false); }
};

// 电机速度类（继承自 Motor）
// 电机速度类（继承自 Motor）
class MotorSpeed : public Motor
{
public:
    MotorControlUnit_t controlSpeed;    // 控制速度的 PID 控制器

    MotorSpeed(MotorType_t* pMotorType, PID* pPidSpeed);
    void updateControl() override;
    void setSpeed(float speed);         // 设置目标速度
};

// 电机角度类（继承自 MotorSpeed）
class MotorAngle : public MotorSpeed
{
public:
    MotorControlUnit_t controlAngle;    // 控制角度的 PID 控制器

    MotorAngle(MotorType_t* pMotorType, PID* pPidSpeed, PID* pPidAngle);
    void updateControl() override;
    void setAngle(float angle);         // 设置目标角度
    void zeroSet();                     // 归零
private:
    using MotorSpeed::setSpeed;
};

// 电机集合类（用于管理多个电机）
class MotorSet
{
public:
    MotorSet();
    ~MotorSet();
    void Append(Motor* motorPtr, uint8_t canLine, uint8_t controllerId);
    Motor* getMotorById(uint8_t canLine, uint8_t controllerId);

    class Iterator
    {
    public:
        explicit Iterator(Motor** pPtr);
        Motor*& operator * () const;
        Iterator& operator ++ ();
        bool operator == (const Iterator& other) const;
        bool operator != (const Iterator& other) const;
    private:
        Motor** ptr;
    };
    Iterator begin();
    Iterator end();

protected:
    Motor* motorMap[2][12] = {nullptr};
    Motor* motorList[24] = {nullptr};
    uint8_t size;
};

void MotorDeviceInit();
void MotorDeviceRoutine();

extern MotorSet motorSet;

extern MotorType_t m3508;
extern MotorType_t m2006;
extern MotorType_t gm6020_i;
extern MotorType_t gm6020_v;

#endif //USER_MOTORDEVICE_H
