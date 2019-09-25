
#ifndef CORVO_PACKET_H
#define CORVO_PACKET_H


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include "corvo_crc.h"

enum {
    kCorvoPacketSentinel = 0xc5,
};

typedef struct {
    uint8_t sentinel;
    uint8_t id;
    uint8_t dataSz;
    uint8_t * const data;
    uint8_t crc;
} __attribute__((packed)) CorvoPacket;


#ifdef __cplusplus
}
#endif // __cplusplus
#endif // CORVO_PACKET_H
