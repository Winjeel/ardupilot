
#ifndef CORVO_CRC_H
#define CORVO_CRC_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <stddef.h>


uint8_t crc8x_simple(uint8_t const * data, size_t sz);
uint8_t crc8x_simple_accum(uint8_t crc, uint8_t const * data, size_t sz);
uint8_t crc8x_fast(uint8_t const * data, size_t sz);
uint8_t crc8x_fast_accum(uint8_t crc, uint8_t const * data, size_t sz);


#ifdef __cplusplus
}
#endif // __cplusplus
#endif // CORVO_CRC_H
