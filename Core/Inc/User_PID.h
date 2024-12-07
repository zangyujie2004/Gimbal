//
// Created by Administrator on 24-12-7.
//

#ifndef USER_PID_H
#define USER_PID_H

struct PIDInitStruct {
    float _kp;
    float _ki;
    float _kd;
    float _i_max; // error_sum限幅
    float _out_max; // 总输出限幅
};

class PID {
public:
    PID();
    PID(const PIDInitStruct& pid_init_struct);
    float calculate(float ref, float fdb);

private:
    float error_[2]; // 0 for current, 1 for last
    float error_sum_;
    float ref_, fdb_;
    float kp_, ki_, kd_;
    float p_out_, i_out_, d_out_;
    float i_max_;
    float out_max_;
    float output_;
};

#endif //USER_PID_H
