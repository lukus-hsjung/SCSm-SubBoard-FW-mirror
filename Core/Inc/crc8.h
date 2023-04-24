/*
 * crc8.h
 *
 *  Created on: Apr 19, 2023
 *      Author: hiseob
 */

#ifndef INC_CRC8_H_
#define INC_CRC8_H_
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

uint8_t __crc8(uint8_t *pcBlock, uint8_t len);
uint8_t __crc8_with_init(uint8_t init_value, uint8_t *pcBlock, uint8_t len);
uint8_t __crc8_byte(uint8_t old_crc, uint8_t byte);

#ifdef __cplusplus
}
#endif
#endif /* INC_CRC8_H_ */
