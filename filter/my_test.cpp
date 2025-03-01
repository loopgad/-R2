/*
 * LaserRanger.cpp
 * 实现文件（适用于STM32等嵌入式平台）
 */

#include "my_test.h"
#include <string.h>
#include <math.h>

LaserRanger::LaserRanger(uint8_t address)
    : m_address(address), m_filterIndex(0), m_retryCount(0), m_laserState(false) {
    for (size_t i = 0; i < FILTER_WINDOW_SIZE; ++i) {
        m_filterWindow[i] = 0.0f;
    }
}

const uint8_t* LaserRanger::enableLaser(uint8_t& len) {
    /* 协议格式：ADDR 06 05 01 CS */
    m_cmdBuffer[0] = m_address;
    m_cmdBuffer[1] = 0x06;
    m_cmdBuffer[2] = 0x05;
    m_cmdBuffer[3] = 0x01;
    m_cmdBuffer[4] = calculateChecksum(m_cmdBuffer, 4);
    len = 5;
    m_laserState = true;
    return m_cmdBuffer;
}

const uint8_t* LaserRanger::disableLaser(uint8_t& len) {
    /* 协议格式：ADDR 06 05 00 CS */
    m_cmdBuffer[0] = m_address;
    m_cmdBuffer[1] = 0x06;
    m_cmdBuffer[2] = 0x05;
    m_cmdBuffer[3] = 0x00;
    m_cmdBuffer[4] = calculateChecksum(m_cmdBuffer, 4);
    len = 5;
    m_laserState = false;
    return m_cmdBuffer;
}

const uint8_t* LaserRanger::setResolution(uint8_t resolution, uint8_t& len) {
    /* 协议格式：FA 04 0C [01/02] CS */
    m_cmdBuffer[0] = BROADCAST_ADDRESS;
    m_cmdBuffer[1] = 0x04;
    m_cmdBuffer[2] = 0x0C;
    m_cmdBuffer[3] = (resolution == 2) ? 0x02 : 0x01;
    m_cmdBuffer[4] = calculateChecksum(m_cmdBuffer, 4);
    len = 5;
    return m_cmdBuffer;
}

const uint8_t* LaserRanger::setMeasurementInterval(uint8_t interval_ms, uint8_t& len) {
    /* 协议格式：FA 04 05 XX CS */
    m_cmdBuffer[0] = BROADCAST_ADDRESS;
    m_cmdBuffer[1] = 0x04;
    m_cmdBuffer[2] = 0x05;
    m_cmdBuffer[3] = interval_ms;
    m_cmdBuffer[4] = calculateChecksum(m_cmdBuffer, 4);
    len = 5;
    return m_cmdBuffer;
}

LaserRanger::ErrorCode LaserRanger::processReceivedData(const uint8_t* data,
                                                      uint16_t length,
                                                      MeasurementData& result) {
    /* 基本格式校验 */
    if(length < 5) return ERR_FORMAT;

    /* 校验和验证（协议3.2节） */
    uint8_t calculated_cs = calculateChecksum(data, length-1);
    if(calculated_cs != data[length-1]) {
        ENTER_CRITICAL();
        m_retryCount = (m_retryCount < MAX_RETRY_ATTEMPTS) ? m_retryCount + 1 : 0;
        EXIT_CRITICAL();
        return ERR_CHECKSUM;
    }

    /* 处理测量数据（协议3.3.7节） */
    if(data[0] == m_address && data[1] == 0x06) {
        // 解析ASCII距离值
        uint8_t dataLength = length - 4; // 去除协议头尾
        memcpy(result.raw, data+3, dataLength);
        result.str[dataLength] = '\0';

        // 转换为浮点数
        char* endPtr;
        result.value = strtof(result.str, &endPtr);
        if(endPtr == result.str) return ERR_FORMAT;

        // 更新滤波器
        updateFilter(result.value);
        m_retryCount = 0; // 成功接收后重置重试计数器
        return ERR_NONE;
    }

    /* 错误代码处理（协议4.1节） */
    if(memcmp(data+3, "ERR-15", 6) == 0) return ERR_OUT_OF_RANGE;
    if(memcmp(data+3, "ERR-16", 6) == 0) return ERR_LOW_SIGNAL;
    if(memcmp(data+3, "ERR-18", 6) == 0) return ERR_HIGH_AMBIENT;

    return ERR_NONE;
}

float LaserRanger::getFilteredDistance() const {
    ENTER_CRITICAL();
    uint8_t validCount = 0;
    float sum = 0.0f;

    // 计算有效数据平均值
    for(uint8_t i = 0; i < FILTER_WINDOW_SIZE; ++i) {
        if(!isnan(m_filterWindow[i])) {
            sum += m_filterWindow[i];
            validCount++;
        }
    }
    EXIT_CRITICAL();

    return (validCount > 0) ? (sum / validCount) : NAN;
}

bool LaserRanger::needRetry() const {
    return (m_retryCount > 0 && m_retryCount <= MAX_RETRY_ATTEMPTS);
}

uint8_t LaserRanger::calculateChecksum(const uint8_t* data, uint8_t len) const {
    /* 校验和算法（协议3.2节） */
    uint16_t sum = 0;
    for(uint8_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return static_cast<uint8_t>(~sum + 1);
}

void LaserRanger::updateFilter(float value) {
    ENTER_CRITICAL();
    m_filterWindow[m_filterIndex] = value;
    m_filterIndex = (m_filterIndex + 1) % FILTER_WINDOW_SIZE;
    EXIT_CRITICAL();
}