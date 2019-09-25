#include "CorvoMsgBuffer.hpp"
#include <cassert>


CorvoMsgBuffer::CorvoMsgBuffer(void) :
    state(AwaitSentinel)
{
    // empty
}

size_t CorvoMsgBuffer::_getCrcOffset(void) {
    return kHeaderSz + get(kSzOffset);
}

size_t CorvoMsgBuffer::_getMsgSz(void) {
    return (kHeaderSz + get(kSzOffset) + kFooterSz);
}


bool CorvoMsgBuffer::_hasSentinel(void) {
    return ((getBytesUsed() > kSentinelOffset) &&
            (get(kSentinelOffset) == kCorvoPacketSentinel));
}

bool CorvoMsgBuffer::_isCompleteMsg(void) {
    // TODO: Edge case: if the length is wrong, we can potentially wait for a
    //       long time until we get enough bytes to reset the state machine.
    //       Potential work-around: check against the expected length values,
    //       and fail early if an invalid length is received.
    size_t used = getBytesUsed();
    return ((used > kSzOffset) && (used >= _getMsgSz()));
}

bool CorvoMsgBuffer::_isValidMsg(void) {
    bool result = false;

    if (_isCompleteMsg()) {
        uint8_t crc = get(_getCrcOffset());

        uint8_t calc = ~crc;

        size_t msgCrcSz = _getMsgSz() - kCrcSz;
        if (msgCrcSz) {
            uint8_t byte = get(0);
            // decoding uses crc8x_fast(), which initialises the CRC seed, so we
            // need to use it here and then accumulate.
            calc = crc8x_fast(&byte, 1);
            for (size_t i = 1; i < msgCrcSz; i++) {
                byte = get(i);
                // the underlying buffer isn't contiguous, so need to accumulate
                // CRC one byte at a time.
                calc = crc8x_fast_accum(calc, &byte, 1);
            }
        }

        result = (crc == calc);
    }

    return result;
}


bool CorvoMsgBuffer::hasMsg(void) {
    uint8_t result = 0;

    size_t bytesNeeded = 1;
    while (bytesNeeded && (bytesNeeded <= getBytesUsed())) {
        switch (state) {
        case AwaitSentinel:
            if (_hasSentinel()) {
                bytesNeeded = (kHeaderSz + kFooterSz - kSentinelSz);
                state = AwaitMessage;
            } else {
                removeFromHead(1);
                bytesNeeded = kSentinelSz;
            }
            break;

        case AwaitMessage:
            if (_isCompleteMsg()) {
                if (_isValidMsg()) {
                    result = 1;
                    bytesNeeded = 0;
                } else {
                    // invalid msg, so remove the sentinel and reset the state machine
                    removeFromHead(kSentinelSz);
                    bytesNeeded = kSentinelSz;
                    state = AwaitSentinel;
                }
            } else {
                bytesNeeded = _getMsgSz();
                if (bytesNeeded > getBufferSz()) {
                    // buffer is not big enough to hold entire message, so discard it
                    // and reset the state machine
                    removeFromHead(1);
                    bytesNeeded = kSentinelSz;
                    state = AwaitSentinel;
                    // TODO: need a notification here
                }
            }
            break;

        default:
            state = AwaitSentinel;
            break;
        }
    }

    return result;
}

size_t CorvoMsgBuffer::consume(void) {
    size_t num = 1;
    if (_isValidMsg()) {
        num = _getMsgSz();
    }

    removeFromHead(num);
    state = AwaitSentinel;

    return num;
}

CorvoMsgID CorvoMsgBuffer::getID(void) {
    if (getBytesUsed() <= kIdOffset) {
        return kInvalidMessageID;
    }

    return get(kIdOffset);
}

uint8_t CorvoMsgBuffer::getDataSz(void) {
    if (getBytesUsed() <= kSzOffset) {
        return 0;
    }

    return get(kSzOffset);
}

size_t CorvoMsgBuffer::copyMsg(uint8_t * const out, size_t outSz) {
    if (_isValidMsg() && (outSz >= _getMsgSz())) {
        return copy(out, _getMsgSz());
    }

    return 0;
}

size_t CorvoMsgBuffer::copyData(void * const out, const size_t outSz) {
    if (_isValidMsg()) {
        const size_t kDataSz = get(kSzOffset);
        if (outSz >= kDataSz) {
            return copy(out, kDataSz, kDataOffset);
        }
    }

    return 0;
}
