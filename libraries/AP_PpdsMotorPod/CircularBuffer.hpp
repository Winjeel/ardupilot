
#pragma once

#include <cstddef>
#include <cstdint>

#include <cstring> // for memset


// TODO: Currently kBufferSz needs to be a power of two...
template <size_t kBufferSz, int kClearChar = '\0'>
class CircularBuffer {
    public:
        CircularBuffer(void) :
            bytesUsed(0),
            head(0),
            rxErrCnt(0)
        {
            memset(buffer, kClearChar, kBufferSz);
        }

        size_t clear(void) {
            const size_t bytesRemoved = bytesUsed;

            bytesUsed = 0;
            head = 0;
            rxErrCnt = 0;
            memset(buffer, kClearChar, kBufferSz);

            return bytesRemoved;
        }

        size_t removeFromHead(const size_t num) {
            if (num == 0 || bytesUsed == 0) {
                return 0;
            }

            size_t toClear = (num < bytesUsed) ? num : bytesUsed;

            // // overwrite removed chars
            // for (size_t i = 0; i < toClear; i++) {
            //     size_t clearIdx = (head + i) % kBufferSz;
            //     buffer[clearIdx] = kClearChar;
            // }

            head = (head + toClear) % kBufferSz;
            bytesUsed -= toClear;

            return toClear;
        }

        size_t removeFromTail(const size_t num) {
            if (num == 0 || bytesUsed == 0) {
                return 0;
            }

            size_t toClear = (num < bytesUsed) ? num : bytesUsed;

            // // overwrite removed chars
            // for (size_t i = 0; i < toClear; i++) {
            //     size_t clearIdx = (head + bytesUsed - i - 1) % kBufferSz;
            //     buffer[clearIdx] = kClearChar;
            // }

            bytesUsed -= toClear;

            return toClear;
        }

        size_t push(uint8_t byte) {
            size_t bytesPushed = 0;

            if (getBytesFree() > 0) {
                buffer[(head + bytesUsed) % kBufferSz] = byte;
                bytesUsed++;
                bytesPushed = 1;
            } else {
                rxErrCnt++;
            }

            return bytesPushed;
        }

        size_t push(uint8_t bytes[], size_t num) {
            const size_t kFree = getBytesFree();

            if (kFree == 0) {
                rxErrCnt += num;
                return 0;
            }

            if (num > kFree) {
                rxErrCnt += (num - kFree);
                num = kFree;
            }

            size_t top = (kBufferSz - (head + bytesUsed));
            size_t numAtHead = (num < top) ? num : top;
            size_t numAtZero = (numAtHead < num) ? (num -  numAtHead) : 0;

            memcpy(&buffer[head + bytesUsed], bytes, numAtHead);
            memcpy(buffer, &bytes[numAtHead], numAtZero);
            bytesUsed += num;

            return num;
        }

        size_t copy(void * const out, const size_t num) {
            return copy(out, num, 0);
        }

        size_t copy(void * const out, const size_t num, size_t offset) const {
            if (out == nullptr) {
                return 0;
            }

            size_t bytesAvailable = (offset < bytesUsed) ? (bytesUsed - offset) : 0;
            size_t toCopy = (num < bytesAvailable) ? num : bytesAvailable;

            // copy data
            size_t numAtHead = (toCopy < (kBufferSz - head)) ? toCopy : (kBufferSz - head);
            size_t numAtZero = (numAtHead < toCopy) ? (toCopy -  numAtHead): 0;
            memcpy(out, &buffer[head + offset], numAtHead);
            memcpy(((uint8_t * const)out + numAtHead), buffer, numAtZero);

            // clear excess (don't copy junk into output buffer)
            size_t toClear = num - toCopy;
            void * const clearPtr = (uint8_t * const)out + toCopy;
            memset(clearPtr, kClearChar, toClear);

            return toCopy;
        }

        size_t popFromHead(void * const out, const size_t num) {
            size_t numBytes = copy(out, num, 0);
            return removeFromHead(numBytes);
        }

        size_t getBytesFree(void) const {
            return (kBufferSz - bytesUsed);
        }


    protected:

        size_t getBytesUsed(void) const {
            return bytesUsed;
        }

        uint8_t operator[](size_t idx) const {
            return get(idx);
        }

        uint8_t get(size_t idx) const {
            if (idx < bytesUsed) {
                return buffer[(head + idx) % kBufferSz];
            }
            return kClearChar;
        }

        size_t getBufferSz(void) const { return kBufferSz; }

    private:
        uint8_t buffer[kBufferSz];
        size_t bytesUsed;
        size_t head;
        uint32_t rxErrCnt;
};
