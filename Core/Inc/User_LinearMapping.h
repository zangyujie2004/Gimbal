//
// Created by Administrator on 24-11-23.
//

#ifndef USER_LINEARMAPPING_H
#define USER_LINEARMAPPING_H

// 标准 C++ 风格的线性映射函数，没有使用 inline
inline float linearMappingInt2Float(int in, int in_min, int in_max, float out_min, float out_max)
{
    return (out_max - out_min) * static_cast<float>(in - in_min) / static_cast<float>(in_max - in_min) + out_min;
}

#endif //USER_LINEARMAPPING_H
