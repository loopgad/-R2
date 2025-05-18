/*
Copyright (c) 2024 loopgad 9th_R2_Member

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


/*
Copyright (c) 2024 loopgad

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


/**
 * @file tool.h
 * @author Yang JianYi(2807643517@qq.com)
 * @brief 该文件用于定义一些通用的工具函数，包括绝对值，限幅，数据转换等。考虑把类去掉，直接使用函数。
 * 
 * @version 0.1
 * @date 2024-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <cstdint>

// 绝对值

inline float Abs(float x) {
    return (x > 0) ? x : -x;
}

// 限幅

inline void Constrain(float *x, float Min, float Max) {
    if (*x < Min) 
        *x = Min;
    else if (*x > Max) 
        *x = Max;
}

// 16位int数据填充
inline void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

// 16位unsigned int数据填充
inline void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

// 32位int数据填充
inline void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

// 32位unsigned int数据填充
inline void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

/**
 * @brief 浮点转换函数, 16位float数据填充
 * @param buffer 数据缓存数组
 * @param number 数据
 * @param scale 数量级
 * @param index 数据索引
 * @param unsign_flag 是否使用unsigned, 使用置1，否则置0
 */
inline void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index, bool unsign_flag) {
    if (unsign_flag)
        buffer_append_int16(buffer, (int16_t)(number * scale), index);
    else
        buffer_append_uint16(buffer, (uint16_t)(number * scale), index);
}

/**
 * @brief 浮点转换函数, 32位float数据填充
 * @param buffer 数据缓存数组
 * @param number 数据
 * @param scale 数量级
 * @param index 数据索引
 * @param unsign_flag 是否使用unsigned, 使用置1，否则置0
 */
inline void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index, bool unsign_flag) {
    if (unsign_flag)
        buffer_append_uint32(buffer, (uint32_t)(number * scale), index);
    else
        buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

/**
 * @brief 16位int数据获取
 * @param buffer 数据缓存数组
 * @param index 数据索引
 * @return 16位int数据
 */
inline int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
    int16_t res = ((uint16_t)buffer[*index]) << 8 |
                  ((uint16_t)buffer[*index + 1]);
    *index += 2;
    return res;
}

/**
 * @brief 16位uint数据获取
 * @param buffer 数据缓存数组
 * @param index 数据索引
 * @return 16位uint数据
 */
inline uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index) {
    uint16_t res = ((uint16_t)buffer[*index]) << 8 |
                   ((uint16_t)buffer[*index + 1]);
    *index += 2;
    return res;
}

/**
 * @brief 32位int数据获取
 * @param buffer 数据缓存数组
 * @param index 数据索引
 * @return 32位int数据
 */
inline int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
    int32_t res = ((uint32_t)buffer[*index]) << 24 |
                  ((uint32_t)buffer[*index + 1]) << 16 |
                  ((uint32_t)buffer[*index + 2]) << 8 |
                  ((uint32_t)buffer[*index + 3]);
    *index += 4;
    return res;
}

/**
 * @brief 32位uint数据获取
 * @param buffer 数据缓存数组
 * @param index 数据索引
 * @return 32位uint数据
 */
inline uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index) {
    uint32_t res = ((uint32_t)buffer[*index]) << 24 |
                   ((uint32_t)buffer[*index + 1]) << 16 |
                   ((uint32_t)buffer[*index + 2]) << 8 |
                   ((uint32_t)buffer[*index + 3]);
    *index += 4;
    return res;
}

/**
 * @brief 浮点转换函数, 16位float数据获取
 * @param buffer 数据缓存数组
 * @param scale 数量级
 * @param index 数据索引
 * @param unsign_flag 是否使用unsigned, 使用置1，否则置0
 * @return 16位float数据
 */
inline float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index, bool unsign_flag) {
    if (unsign_flag)
        return (float)buffer_get_uint16(buffer, index) / scale;
    else
        return (float)buffer_get_int16(buffer, index) / scale;
}

/**
 * @brief 浮点转换函数, 32位float数据获取
 * @param buffer 数据缓存数组
 * @param scale 数量级
 * @param index 数据索引
 * @param unsign_flag 是否使用unsigned, 使用置1，否则置0
 * @return 32位float数据
 */
inline float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index, bool unsign_flag) {
    if (unsign_flag)
        return (float)buffer_get_uint32(buffer, index) / scale;
    else
        return (float)buffer_get_int32(buffer, index) / scale;
}

inline float DM_fmaxf(float a, float b) {
    return a >= b ? a : b;
}

inline float DM_fminf(float a, float b) {
    return a <= b ? a : b;
}

/***浮点型转整形***
 * 入口参数：浮点数据、该数据最小值、该数据最大值、位数
 *****************/
inline int DM_float_to_uint(float x1, float x1_min, float x1_max, int bits) {
    float span = x1_max - x1_min;
    float offset = x1_min;
    return (int)((x1 - offset) * ((float)((1 << bits) - 1)) / span);
}

// 整型转浮点型
// 根据给定的范围和位数，将无符号整数转换为浮点
inline float DM_uint_to_float(int x1_int, float x1_min, float x1_max, int bits) {
    float span = x1_max - x1_min;
    float offset = x1_min;
    return ((float)x1_int) * span / ((float)((1 << bits) - 1)) + offset;
}

#ifdef __cplusplus
}
#endif
