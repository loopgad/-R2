//
// Created by loopgad on 25-2-26.
//
/*
 * LaserRanger.h
 * 激光测距模块控制类（符合MISRA C++2014规范）
 * 版本：1.3
 * 最后更新：2023-12-20
 */

#ifndef LASER_RANGER_H
#define LASER_RANGER_H

#include <cstdint>

// 设备默认地址（协议规定）
#define DEFAULT_ADDRESS      0x80
// 广播地址（用于地址设置等特殊操作）
#define BROADCAST_ADDRESS    0xFA
// 最大重试次数（协议建议值）
#define MAX_RETRY_ATTEMPTS   3
// 滤波窗口大小（环形缓冲区）
#define FILTER_WINDOW_SIZE   4

class LaserRanger {
public:
    /**
     * @brief 测量结果数据结构（共用体实现协议规定的ASCII格式转换）
     */
    typedef union {
        uint8_t raw[12];    // 原始字节存储（最大支持10位有效数字）
        char str[12];       // ASCII字符串表示
        float value;        // 浮点数值转换
    } MeasurementData;

    /**
     * @brief 错误代码定义（协议文档第4章）
     */
    typedef enum {
        ERR_NONE,            // 无错误
        ERR_CHECKSUM,        // 校验和错误
        ERR_FORMAT,          // 数据格式错误
        ERR_OUT_OF_RANGE,    // 超出量程（对应协议ERR-15）
        ERR_LOW_SIGNAL,      // 信号弱（对应协议ERR-16）
        ERR_HIGH_AMBIENT     // 环境光过强（对应协议ERR-18）
    } ErrorCode;

    /**
     * @brief 构造函数
     * @param address 设备地址（默认0x80）
     */
    explicit LaserRanger(uint8_t address = DEFAULT_ADDRESS);

    /* 核心功能命令接口 */

    /**
     * @brief 生成激光开启命令（自动进入连续测量模式）
     * @param[out] len 生成的命令长度
     * @return 命令数据指针
     */
    const uint8_t* enableLaser(uint8_t& len);

    /**
     * @brief 生成激光关闭命令
     * @param[out] len 生成的命令长度
     * @return 命令数据指针
     */
    const uint8_t* disableLaser(uint8_t& len);

    /**
     * @brief 设置测量分辨率
     * @param resolution 1-1mm，2-0.1mm（协议规定值）
     * @param[out] len 生成的命令长度
     * @return 命令数据指针
     */
    const uint8_t* setResolution(uint8_t resolution, uint8_t& len);

    /**
     * @brief 设置测量周期
     * @param interval_ms 测量间隔（单位：毫秒，范围0-255）
     * @param[out] len 生成的命令长度
     * @return 命令数据指针
     */
    const uint8_t* setMeasurementInterval(uint8_t interval_ms, uint8_t& len);

    /**
     * @brief 处理接收数据
     * @param data 接收数据指针
     * @param length 数据长度
     * @param[out] result 测量结果输出
     * @return 错误代码
     */
    ErrorCode processReceivedData(const uint8_t* data, uint16_t length, MeasurementData& result);

    /**
     * @brief 获取滤波后的距离值
     * @return 滤波后的距离值（单位：米）
     * @note 使用4点滑动窗口滤波，数据不足时返回最新有效值
     */
    float getFilteredDistance() const;

    /**
     * @brief 检查是否需要重试
     * @return true表示需要重发最后一条命令
     */
    bool needRetry() const;

private:
    // 临界区保护宏（需根据目标平台实现）
    #define ENTER_CRITICAL()  /* 实现平台相关的中断禁用 */
    #define EXIT_CRITICAL()   /* 实现平台相关的中断启用 */

    /**
     * @brief 校验和计算（协议附录A）
     * @param data 数据指针
     * @param len 数据长度
     * @return 校验和字节
     */
    uint8_t calculateChecksum(const uint8_t* data, uint8_t len) const;

    /**
     * @brief 更新滑动窗口滤波器
     * @param value 新测量值
     */
    void updateFilter(float value);

    // 成员变量
    uint8_t m_address;               // 当前设备地址
    uint8_t m_cmdBuffer[8];          // 命令缓冲区（足够存放最长命令）
    volatile float m_filterWindow[FILTER_WINDOW_SIZE]; // 滤波窗口
    volatile uint8_t m_filterIndex;  // 当前写入位置
    uint8_t m_retryCount;            // 当前重试次数
    bool m_laserState;               // 激光器状态
};

#endif // LASER_RANGER_H
