#include "SL_MsgBuffer.h"
#include <cassert>


#define CRC_OFFSET (get(kOffsetLength) + (unsigned)kOffsetLength)


static const uint8_t kCrc8_Table[256] = {
      0,  94, 188, 226,  97,  63, 221, 131, 194, 156, 126,  32, 163, 253,  31,  65,
    157, 195,  33, 127, 252, 162,  64,  30,  95,   1, 227, 189,  62,  96, 130, 220,
     35, 125, 159, 193,  66,  28, 254, 160, 225, 191,  93,   3, 128, 222,  60,  98,
    190, 224,   2,  92, 223, 129,  99,  61, 124,  34, 192, 158,  29,  67, 161, 255,
     70,  24, 250, 164,  39, 121, 155, 197, 132, 218,  56, 102, 229, 187,  89,   7,
    219, 133, 103,  57, 186, 228,   6,  88,  25,  71, 165, 251, 120,  38, 196, 154,
    101,  59, 217, 135,   4,  90, 184, 230, 167, 249,  27,  69, 198, 152, 122,  36,
    248, 166,  68,  26, 153, 199,  37, 123,  58, 100, 134, 216,  91,   5, 231, 185,
    140, 210,  48, 110, 237, 179,  81,  15,  78,  16, 242, 172,  47, 113, 147, 205,
     17,  79, 173, 243, 112,  46, 204, 146, 211, 141, 111,  49, 178, 236,  14,  80,
    175, 241,  19,  77, 206, 144, 114,  44, 109,  51, 209, 143,  12,  82, 176, 238,
     50, 108, 142, 208,  83,  13, 239, 177, 240, 174,  76,  18, 145, 207,  45, 115,
    202, 148, 118,  40, 171, 245,  23,  73,   8,  86, 180, 234, 105,  55, 213, 139,
     87,   9, 235, 181,  54, 104, 138, 212, 149, 203,  41, 119, 244, 170,  72,  22,
    233, 183,  85,  11, 136, 214,  52, 106,  43, 117, 151, 201,  74,  20, 246, 168,
    116,  42, 200, 150,  21,  75, 169, 247, 182, 232,  10,  84, 215, 137, 107,  53,
};


SL_MsgBuffer::SL_MsgBuffer(void) :
    state(AwaitMagic1)
{
    // empty
}


SL_MsgId SL_MsgBuffer::assess(void) {
    SL_MsgId msgType = SL_MsgId_None;
    size_t bytesNeeded = 1;

    while (bytesNeeded && (bytesNeeded <= getBytesUsed())) {
        switch (state) {
        case AwaitMagic1:
            if (_hasMagic1()) {
                bytesNeeded = 2;
                state = AwaitMagic2;
            } else {
                removeFromHead(1);
                bytesNeeded = 1;
                state = AwaitMagic1;
            }
            break;

        case AwaitMagic2:
            if (_hasMagic2()) {
                bytesNeeded = 3;
                state = AwaitMsg;
            } else {
                removeFromHead(2);
                bytesNeeded = 1;
                state = AwaitMagic1;
            }
            break;

        case AwaitMsg:
            if (_hasMsg()) {
                if (isValidMsg()) {
                    msgType = (SL_MsgId)get(kOffsetType);
                    bytesNeeded = 0;
                } else {
                    removeFromHead(1);
                    bytesNeeded = 1;
                    state = AwaitMagic1;
                }
            } else {
                bytesNeeded = get(kOffsetLength) + kHeaderSz;
            }
            break;

        default:
            break;
        }
    }

    return msgType;
}

size_t SL_MsgBuffer::copyMsg(void * const out, const size_t len) {
    return extract(out, len, 0);
}

size_t SL_MsgBuffer::copyData(void * const out, const size_t len) {
    size_t dataLen = getMsgDataLength();
    if (dataLen > len) {
        dataLen = len;
    }
    return extract(out, dataLen, kHeaderSz);
}

size_t SL_MsgBuffer::consumeMsg(void) {
    size_t num = isValidMsg() ? getMsgLength() : 1;

    removeFromHead(num);
    state = AwaitMagic1;

    return num;
}


SL_MsgId SL_MsgBuffer::getMsgType(void) {
    if (getBytesUsed() > kOffsetType) {
        return (SL_MsgId)get(kOffsetType);
    } else {
        return SL_MsgId_None;
    }
}

size_t SL_MsgBuffer::getMsgLength(void) {
    if (getBytesUsed() > kOffsetLength) {
        // magic1, magic2 and length fields are not included in length byte, so add them in
        return get(kOffsetLength) + 3;
    } else {
        return 0;
    }
}

size_t SL_MsgBuffer::getMsgDataLength(void) {
    if (getBytesUsed() > kOffsetType) {
        uint8_t len = get(kOffsetLength);
        // Length = number of bytes following: type, data[], crc
        return (len >= 2) ? (len - 2) : 0;
    } else {
        return 0;
    }
}

bool SL_MsgBuffer::isValidMsg(void) {
    uint8_t crc = _getCrc();
    uint8_t calc = _calculateCrc();

    return (_hasMsg() && (crc == calc));
}


uint8_t SL_MsgBuffer::calculateCrc(SL_MsgId id, uint8_t const * const data, size_t dataLen) {
    uint8_t crc = kCrc8_Table[0x01 ^ (uint8_t)id];

    // for each byte between Length and Checksum fields
    if (dataLen) {
        for (size_t idx = 0; idx < dataLen; idx++) {
            crc = kCrc8_Table[crc ^ data[idx]];
        }
    }

    return crc;
}


uint8_t SL_MsgBuffer::_getCrc(void) {
    assert(getBytesUsed() >= CRC_OFFSET);
    return get(CRC_OFFSET);
}

uint8_t SL_MsgBuffer::_calculateCrc(void) {
    assert(get(kOffsetMagic1) == SL_MAGIC_1);
    assert(get(kOffsetMagic2) == SL_MAGIC_2);

    uint8_t crc = 0x01;
    // for each byte between Length and Checksum fields
    size_t msgLen = getMsgDataLength();
    // length includes CRC bytes, CRC calc doesn't
    for (size_t idx = kOffsetType; idx < (msgLen + kOffsetType + 1); idx++) {
        crc = kCrc8_Table[crc ^ get(idx)];
    }

    return crc;
}

bool SL_MsgBuffer::_hasMagic1(void) {
    return ((getBytesUsed() > kOffsetMagic1) &&
            (get(kOffsetMagic1) == SL_MAGIC_1));
}

bool SL_MsgBuffer::_hasMagic2(void) {
    return ((getBytesUsed() > kOffsetMagic2) &&
            (get(kOffsetMagic2) == SL_MAGIC_2));
}

bool SL_MsgBuffer::_hasMsg(void) {
    // TODO: Edge case: if the length is wrong, we can potentially wait for a
    //       long time until we get enough bytes to reset the state machine.
    //       Potential work-arooud: check against the expected length values,
    //       and fail early if an invalid length is received.
    size_t used = getBytesUsed();
    return ((used > kOffsetLength) &&
            (used > CRC_OFFSET));
}
