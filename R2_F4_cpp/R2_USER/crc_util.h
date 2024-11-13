
#ifndef __CRC_UTIL_H__
#define __CRC_UTIL_H__

#include <cstdint>

uint16_t CRC16_Table(uint8_t *p, uint8_t counter);
uint8_t CRC8_Table(uint8_t *p, uint8_t counter);


#endif // __CRC_UTIL_H__
