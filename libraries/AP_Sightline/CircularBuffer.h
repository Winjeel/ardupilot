
#pragma once

#include <cstddef>
#include <cstdint>

#include <cstring> // for memset

#if (INCLUDE_PRINT)
#include <iostream>
#endif // INCLUDE_PRINT


template <size_t kBufferSz, int kClearChar = '\0'>
class CircularBuffer {
    public:
        CircularBuffer(void) :
            bytesUsed(0),
            head(0)
        {
            memset(buffer, kClearChar, kBufferSz);
        }

        void removeFromHead(const size_t num) {
            if (num == 0 || bytesUsed == 0) {
                return;
            }

            size_t toClear = (num < bytesUsed) ? num : bytesUsed;

            // overwrite removed chars
            for (size_t i = 0; i < toClear; i++) {
                size_t clearIdx = (head + i) % kBufferSz;
                buffer[clearIdx] = kClearChar;
            }

            head = (head + toClear) % kBufferSz;
            bytesUsed -= toClear;
        }


        void removeFromTail(const size_t num) {
            if (num == 0 || bytesUsed == 0) {
                return;
            }

            size_t toClear = (num < bytesUsed) ? num : bytesUsed;

            // overwrite removed chars
            for (size_t i = 0; i < toClear; i++) {
                size_t clearIdx = (head + bytesUsed - i - 1) % kBufferSz;
                buffer[clearIdx] = kClearChar;
            }

            bytesUsed -= toClear;
        }

        size_t push(uint8_t byte) {
            if (getBytesFree() > 0) {
                buffer[(head + bytesUsed) % kBufferSz] = byte;
                bytesUsed++;
                return 1;
            }
            return 0;
        }

        size_t push(uint8_t bytes[], size_t num) {
            if (num > getBytesFree()) {
                num = getBytesFree();
            }

            size_t top = (kBufferSz - (head + bytesUsed));
            size_t numAtHead = (num < top) ? num : top;
            size_t numAtZero = (numAtHead < num) ? (num -  numAtHead) : 0;

            memcpy(&buffer[head + bytesUsed], bytes, numAtHead);
            memcpy(buffer, &bytes[numAtHead], numAtZero);
            bytesUsed += num;

            return num;
        }

        size_t extract(void * const out, const size_t num) {
            return extract(out, num, 0);
        }

        size_t extract(void * const out, const size_t num, size_t offset) {
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

            // clear excess
            size_t toClear = num - toCopy;
            void * const clearPtr = (uint8_t * const)out + toCopy;
            memset(clearPtr, kClearChar, toClear);

            return toCopy;
        }

        size_t getBytesFree(void) {
            return (kBufferSz - bytesUsed);
        }

        size_t getBytesUsed(void) {
            return bytesUsed;
        }

#if (INCLUDE_PRINT)
        void printChars(void) {
            printChars("");
        }

        void printChars(char const * prefix) {
            const size_t kCharsPerChunk = 8;
            const size_t kChunksPerLine = 8;

            char buff[16] = {0};

            snprintf(buff, sizeof(buff), "b[%03lu/%03lu]: ", bytesUsed, kBufferSz);
            std::cout << prefix << buff;
            memset(buff, ' ', sizeof(buff));

            for (size_t i = 0; i < kBufferSz; i++) {
                char c = (char)get(i);
                if (!isprint(c)) {
                    c = '.';
                }

                if ((i % (kCharsPerChunk * kChunksPerLine)) == 0) {
                    std::cout << std::endl << prefix << "  ";
                } else if ((i % kCharsPerChunk) == 0) {
                    std::cout << " ";
                }

                snprintf(buff, sizeof(buff), "%c", c);
                std::cout << buff;
            }
            std::cout << std::endl << std::endl;
        }

        void printHex(void) {
            const size_t kBytesPerChunk = 8;
            const size_t kChunksPerLine = 4;

            char buff[16] = {0};

            snprintf(buff, sizeof(buff), "b[%03lu/%03lu]: ", bytesUsed, kBufferSz);
            std::cout << buff;
            memset(buff, ' ', sizeof(buff));

            for (size_t i = 0; i < kBufferSz; i++) {
                if ((i % (kBytesPerChunk * kChunksPerLine)) == 0) {
                    std::cout << std::endl << "  ";
                } else if ((i % kBytesPerChunk) == 0) {
                    std::cout << " ";
                }

                snprintf(buff, sizeof(buff), "%02d", get(i));
                std::cout << buff;
            }
            std::cout << std::endl << std::endl;
        }
#else
        void printChars(void) { }
        void printChars(char const * prefix) { }
        void printHex(void) { }
#endif // INCLUDE_PRINT


    protected:

        // TODO: use operator[]
        uint8_t get(size_t i) {
            if (i < bytesUsed) {
                return buffer[(head + i) % kBufferSz];
            }
            return kClearChar;
        }


    private:
        uint8_t buffer[kBufferSz];
        size_t bytesUsed;
        size_t head;
};
