
// NOTE: This file is stored in the common-interfaces repo, but is copied
//       into the projects that use this interface. Make your changes in
//       the common-interfaces repo, otherwise they will be lost when the
//       interface is next generated.

#include "PpdsMotorPodProtocol.h"


//! \return the packet data pointer from the packet
uint8_t* getPpdsMotorPodPacketData(CorvoPacket* pkt) {
    return pkt->data;
}

//! \return the packet data pointer from the packet, const
const uint8_t* getPpdsMotorPodPacketDataConst(const CorvoPacket* pkt) {
    return pkt->data;
}

//! Complete a packet after the data have been encoded
void finishPpdsMotorPodPacket(CorvoPacket* pkt, int size, uint32_t packetID) {
    const size_t kHeaderCrcBytes = offsetof(CorvoPacket, data);

    pkt->sentinel = kCorvoPacketSentinel;
    pkt->id       = packetID;
    pkt->dataSz   = size;
    pkt->crc      = crc8x_fast((uint8_t const *)pkt, kHeaderCrcBytes);
    pkt->crc      = crc8x_fast_accum(pkt->crc, pkt->data, size);
}

//! \return the size of a packet from the packet header
int getPpdsMotorPodPacketSize(const CorvoPacket* pkt) {
    return pkt->dataSz;
}

//! \return the ID of a packet from the packet header
uint32_t getPpdsMotorPodPacketID(const CorvoPacket* pkt) {
    return pkt->id;
}
