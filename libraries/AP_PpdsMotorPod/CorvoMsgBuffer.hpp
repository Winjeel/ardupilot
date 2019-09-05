
#include "CircularBuffer.hpp"
#include "corvo_packet.h"

#include <cstddef> // for offsetof


typedef uint8_t CorvoMsgID;



class CorvoMsgBuffer : private CircularBuffer<256, 0> {
    public:

        enum {
            kInvalidMessageID = -1,
        };


        CorvoMsgBuffer(void);

        // from base class
        using CircularBuffer::push;
        size_t push(uint8_t byte) {
            return CircularBuffer::push(byte);
        }
        size_t push(uint8_t bytes[], size_t num) {
            return CircularBuffer::push(bytes, num);
        }

        using CircularBuffer::getBytesFree;
        size_t getBytesFree(void) {
            return CircularBuffer::getBytesFree();
        }


        bool hasMsg(void);

        size_t consume(void);

        CorvoMsgID getID(void);
        uint8_t getDataSz(void);

        size_t copyData(void * const out, const size_t len);


    private:
        typedef enum {
            AwaitSentinel,
            AwaitMessage,
        } MessageState;

        MessageState state;

        size_t _getCrcOffset(void);
        size_t _getMsgSz(void);

        bool _hasSentinel(void);
        bool _isCompleteMsg(void);
        bool _isValidMsg(void);

        enum {
            kSentinelSz     = sizeof(((CorvoPacket *)0)->sentinel),

            kSentinelOffset = offsetof(CorvoPacket, sentinel),
            kIdOffset       = offsetof(CorvoPacket, id),
            kSzOffset       = offsetof(CorvoPacket, dataSz),
            kDataOffset     = offsetof(CorvoPacket, data),

            kHeaderSz       = offsetof(CorvoPacket, data),
            kCrcSz          = sizeof(((CorvoPacket *)0)->crc),
            kFooterSz       = kCrcSz,
        };
};
