
#include "CircularBuffer.h"
#include "sightline_protocol.h"

#include <cstddef> // for offsetof


// TODO: check the max message size (256 = 251 data + 4 header + 1 crc)
//       currently needs to be power of two
class SL_MsgBuffer : private CircularBuffer<256, 0> {
    public:

        SL_MsgBuffer(void);

        // from base class
        using CircularBuffer::push;
        size_t push(uint8_t byte) {
            return CircularBuffer::push(byte);
        }

        using CircularBuffer::getBytesFree;
        size_t getBytesFree(void) {
            return CircularBuffer::getBytesFree();
        }

        using CircularBuffer::printChars;
        void print(char const * prefix) {
            CircularBuffer::printChars(prefix);
        }

        SL_MsgId assess(void);

        size_t copyMsg(void * const out, const size_t len);
        size_t copyData(void * const out, const size_t len);
        size_t consumeMsg(void);

        bool isValidMsg(void);
        SL_MsgId getMsgType(void);
        size_t getMsgDataLength(void);
        size_t getMsgLength(void);


        uint8_t calculateCrc(const SL_MsgId id, uint8_t const * const data, size_t dataLen);

    private:
        typedef enum {
            AwaitMagic1 = 0,
            AwaitMagic2 = 1,
            AwaitMsg    = 2,
        } SL_MsgState;

        SL_MsgState state;

        uint8_t _getCrc(void);
        uint8_t _calculateCrc(void);

        bool _hasMagic1(void);
        bool _hasMagic2(void);
        bool _hasMsg(void);

        enum {
            kOffsetMagic1 = 0, // offsetof(SL_MsgHeader, magic1),
            kOffsetMagic2 = 1, // offsetof(SL_MsgHeader, magic2),
            kOffsetLength = 2, // offsetof(SL_MsgHeader, length),
            kOffsetType   = 3, // offsetof(SL_MsgHeader, type),

            kHeaderSz = sizeof(SL_MsgHeader),
            kCrcSz    = 1,
        };
};
