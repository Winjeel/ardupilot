
#include "../CircularBuffer.hpp"


#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"


static const uint8_t sArr[16] { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
static const uint8_t sZeroes[16] { 0, };

static const size_t kSize = 16;
static const uint8_t kClearChar = 0xFF;


SCENARIO( "test copying", "[CircularBuffer]" ) {

    GIVEN( "a half full buffer with size 16 (a power of two) that wraps" ) {
        CircularBuffer<16, kClearChar> cb;

        REQUIRE( cb.getBytesFree() == 16 );
        REQUIRE( cb.getHead() == 0 );

        const uint8_t kOther[12] = { 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,
                                     0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, };
        size_t pushed = cb.push(kOther, 12);
        REQUIRE( pushed == 12 );
        REQUIRE( cb.getBytesFree() == 4 );
        REQUIRE( cb.getHead() == 0 );

        size_t removed = cb.removeFromHead(12);
        REQUIRE( removed == 12 );
        REQUIRE( cb.getBytesFree() == 16 );
        REQUIRE( cb.getHead() == 12 );

        pushed = cb.push(sArr, 8);
        REQUIRE( pushed == 8 );
        REQUIRE( cb.getBytesFree() == 8 );
        REQUIRE( cb.getHead() == 12 );

        uint8_t out[16];

        // At this point, the buffer looks like:
        //    0x0 0x1 0x2 0x3 0x4 0x5 0x6 0x7 0x8 0x9 0xA 0xB 0xC 0xD 0xE 0xF
        //   +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
        //   | 5 | 6 | 7 | 8 | f | f | f | f | f | f | f | f | 1 | 2 | 3 | 4 |
        //   +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
        //                                                     ^
        //                                                   head

        WHEN( "6 bytes copied from an offset of 0" ) {
            memset(out, 0, sizeof(out));
            size_t copied = cb.copy(out, 6, 0);

            THEN( "correct bytes are copied" ) {
                REQUIRE( copied == 6 );
                REQUIRE( out[0] == 1 );
                REQUIRE( out[1] == 2 );
                REQUIRE( out[2] == 3 );
                REQUIRE( out[3] == 4 );
                REQUIRE( out[4] == 5 );
                REQUIRE( out[5] == 6 );
            }

            THEN( "rest of out buffer is untouched" ) {
                REQUIRE( memcmp(&out[copied], sZeroes, (16 - copied)) == 0 );
            }

            THEN( "bytes free and head don't change" ) {
                REQUIRE( cb.getBytesFree() == 8 );
                REQUIRE( cb.getHead() == 12 );
            }
        }

        WHEN( "6 bytes copied from an offset of 2" ) {
            memset(out, 0, sizeof(out));
            size_t copied = cb.copy(out, 6, 2);

            THEN( "correct bytes are copied" ) {
                REQUIRE( copied == 6 );
                REQUIRE( out[0] == 3 );
                REQUIRE( out[1] == 4 );
                REQUIRE( out[2] == 5 );
                REQUIRE( out[3] == 6 );
                REQUIRE( out[4] == 7 );
                REQUIRE( out[5] == 8 );
            }

            THEN( "rest of out buffer is untouched" ) {
                REQUIRE( memcmp(&out[copied], sZeroes, (16 - copied)) == 0 );
            }

            THEN( "bytes free and head don't change" ) {
                REQUIRE( cb.getBytesFree() == 8 );
                REQUIRE( cb.getHead() == 12 );
            }
        }

        WHEN( "2 bytes copied from an offset of 6" ) {
            memset(out, 0, sizeof(out));
            size_t copied = cb.copy(out, 2, 6);

            THEN( "correct bytes are copied" ) {
                REQUIRE( copied == 2 );
                REQUIRE( out[0] == 7 );
                REQUIRE( out[1] == 8 );
            }

            THEN( "bytes free and head don't change" ) {
                REQUIRE( cb.getBytesFree() == 8 );
                REQUIRE( cb.getHead() == 12 );
            }
        }

        WHEN( "4 bytes copied from an offset of 6" ) {
            memset(out, 0, sizeof(out));
            size_t copied = cb.copy(out, 4, 6);

            THEN( "correct bytes are copied" ) {
                REQUIRE( copied == 2 );
                REQUIRE( out[0] == 7 );
                REQUIRE( out[1] == 8 );
            }

            THEN( "rest of out buffer is untouched" ) {
                REQUIRE( memcmp(&out[copied], sZeroes, (16 - copied)) == 0 );
            }

            THEN( "bytes free and head don't change" ) {
                REQUIRE( cb.getBytesFree() == 8 );
                REQUIRE( cb.getHead() == 12 );
            }
        }
    }

    GIVEN( "a half full buffer with size 15 (not power of two) that wraps" ) {
        CircularBuffer<15, kClearChar> cb;

        REQUIRE( cb.getBytesFree() == 15 );
        REQUIRE( cb.getHead() == 0 );

        const uint8_t kOther[11] = { 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,
                                     0x0f, 0x0f, 0x0f, 0x0f, 0x0f,       };
        size_t pushed = cb.push(kOther, 11);
        REQUIRE( pushed == 11 );
        REQUIRE( cb.getBytesFree() == 4 );
        REQUIRE( cb.getHead() == 0 );

        size_t removed = cb.removeFromHead(11);
        REQUIRE( removed == 11 );
        REQUIRE( cb.getBytesFree() == 15 );
        REQUIRE( cb.getHead() == 11 );

        pushed = cb.push(sArr, 8);
        REQUIRE( pushed == 8 );
        REQUIRE( cb.getBytesFree() == 7 );
        REQUIRE( cb.getHead() == 11 );

        uint8_t out[16];

        // At this point, the buffer looks like:
        //    0x0 0x1 0x2 0x3 0x4 0x5 0x6 0x7 0x8 0x9 0xA 0xB 0xC 0xD 0xE
        //   +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
        //   | 5 | 6 | 7 | 8 | f | f | f | f | f | f | f | 1 | 2 | 3 | 4 |
        //   +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
        //                                                 ^
        //                                               head

        WHEN( "6 bytes copied from an offset of 0" ) {
            memset(out, 0, sizeof(out));
            size_t copied = cb.copy(out, 6, 0);

            THEN( "correct bytes are copied" ) {
                REQUIRE( copied == 6 );
                REQUIRE( out[0] == 1 );
                REQUIRE( out[1] == 2 );
                REQUIRE( out[2] == 3 );
                REQUIRE( out[3] == 4 );
                REQUIRE( out[4] == 5 );
                REQUIRE( out[5] == 6 );
            }

            THEN( "rest of out buffer is untouched" ) {
                REQUIRE( memcmp(&out[copied], sZeroes, (15 - copied)) == 0 );
            }

            THEN( "bytes free and head don't change" ) {
                REQUIRE( cb.getBytesFree() == 7 );
                REQUIRE( cb.getHead() == 11 );
            }
        }

        WHEN( "6 bytes copied from an offset of 2" ) {
            memset(out, 0, sizeof(out));
            size_t copied = cb.copy(out, 6, 2);

            THEN( "correct bytes are copied" ) {
                REQUIRE( copied == 6 );
                REQUIRE( out[0] == 3 );
                REQUIRE( out[1] == 4 );
                REQUIRE( out[2] == 5 );
                REQUIRE( out[3] == 6 );
                REQUIRE( out[4] == 7 );
                REQUIRE( out[5] == 8 );
            }

            THEN( "rest of out buffer is untouched" ) {
                REQUIRE( memcmp(&out[copied], sZeroes, (15 - copied)) == 0 );
            }

            THEN( "bytes free and head don't change" ) {
                REQUIRE( cb.getBytesFree() == 7 );
                REQUIRE( cb.getHead() == 11 );
            }
        }

        WHEN( "2 bytes copied from an offset of 6" ) {
            memset(out, 0, sizeof(out));
            size_t copied = cb.copy(out, 2, 6);

            THEN( "correct bytes are copied" ) {
                REQUIRE( copied == 2 );
                REQUIRE( out[0] == 7 );
                REQUIRE( out[1] == 8 );
            }

            THEN( "rest of out buffer is untouched" ) {
                REQUIRE( memcmp(&out[copied], sZeroes, (15 - copied)) == 0 );
            }

            THEN( "bytes free and head don't change" ) {
                REQUIRE( cb.getBytesFree() == 7 );
                REQUIRE( cb.getHead() == 11 );
            }
        }

        WHEN( "4 bytes copied from an offset of 6" ) {
            memset(out, 0, sizeof(out));
            size_t copied = cb.copy(out, 4, 6);

            THEN( "correct bytes are copied" ) {
                REQUIRE( copied == 2 );
                REQUIRE( out[0] == 7 );
                REQUIRE( out[1] == 8 );
            }

            THEN( "rest of out buffer is untouched" ) {
                REQUIRE( memcmp(&out[copied], sZeroes, (15 - copied)) == 0 );
            }

            THEN( "bytes free and head don't change" ) {
                REQUIRE( cb.getBytesFree() == 7 );
                REQUIRE( cb.getHead() == 11 );
            }
        }
    }
}

SCENARIO( "test basics", "[CircularBuffer]" ) {

    GIVEN( "An empty circular buffer" ) {
        CircularBuffer<kSize, kClearChar> cb;

        REQUIRE( cb.getBytesFree() == 16 );
        REQUIRE( cb.getHead() == 0 );

        WHEN( "one byte is added" ) {
            size_t pushed = cb.push(0xA5);

            THEN( "bytes free decreases and head doesn't change" ) {
                REQUIRE( pushed == 1 );
                REQUIRE( cb.getBytesFree() == 15 );
                REQUIRE( cb.getHead() == 0 );
            }
        }

        WHEN( "16 bytes are added" ) {
            size_t pushed = cb.push(sArr, 16);

            THEN( "bytes free is zero and head doesn't change" ) {
                REQUIRE( pushed == 16 );
                REQUIRE( cb.getBytesFree() == 0 );
                REQUIRE( cb.getHead() == 0 );
            }
        }
    }

    GIVEN( "A half-full circular buffer with a capacity of 16" ) {
        CircularBuffer<kSize, kClearChar> cb;

        cb.push(sArr, 8);

        REQUIRE( cb.getBytesFree() == 8 );
        REQUIRE( cb.getHead() == 0 );

        WHEN( "20 bytes copied from head" ) {
            uint8_t out[20] = { 0x5A, };
            size_t copied = cb.copy(out, 20);

            THEN( "correct bytes are copied" ) {
                REQUIRE( copied == 8 );
                REQUIRE( memcmp(out, sArr, 8) == 0 );
            }

            THEN( "bytes free and head don't change" ) {
                REQUIRE( cb.getBytesFree() == 8 );
                REQUIRE( cb.getHead() == 0 );
            }
        }

        WHEN( "20 bytes copied from offset 4" ) {
            uint8_t out[20] = { 0x5A, };
            size_t copied = cb.copy(out, 20, 4);

            THEN( "correct bytes are copied" ) {
                // copying doesn't wrap over head, so copies at most (bytesUsed - offset) bytes
                REQUIRE( copied == (8 - 4) );
                REQUIRE( memcmp(&out[0], &sArr[4], 4) == 0 );
                REQUIRE( memcmp(&out[4], &sZeroes[0], 4) == 0 );
            }

            THEN( "bytes free and head don't change" ) {
                REQUIRE( cb.getBytesFree() == 8 );
                REQUIRE( cb.getHead() == 0 );
            }
        }
    }

    GIVEN( "A full circular buffer with a capacity of 16" ) {
        CircularBuffer<16> cb;

        cb.push(sArr, 16);

        REQUIRE( cb.getBytesFree() == 0 );
        REQUIRE( cb.getHead() == 0 );

        WHEN( "one byte is added" ) {
            size_t pushed = cb.push(0xA5);

            THEN( "nothing added, bytes free and head don't change" ) {
                REQUIRE( pushed == 0 );
                REQUIRE( cb.getBytesFree() == 0 );
                REQUIRE( cb.getHead() == 0 );
            }
        }

        WHEN( "16 bytes are added" ) {
            size_t pushed = cb.push(sArr, 16);

            THEN( "nothing added, bytes free and head don't change" ) {
                REQUIRE( pushed == 0 );
                REQUIRE( cb.getBytesFree() == 0 );
                REQUIRE( cb.getHead() == 0 );
            }
        }

        WHEN( "buffer is cleared" ) {
            size_t removed = cb.clear();

            THEN( "all bytes are removed, and head is reset" ) {
                REQUIRE( removed == 16 );
                REQUIRE( cb.getBytesFree() == 16 );
                REQUIRE( cb.getHead() == 0 );
            }
        }

        WHEN( "0 bytes removed from head" ) {
            size_t removed = cb.removeFromHead(0);

            THEN( "head and bytes free change" ) {
                REQUIRE( removed == 0 );
                REQUIRE( cb.getBytesFree() == 0 );
                REQUIRE( cb.getHead() == 0 );
            }
        }

        WHEN( "8 bytes removed from head" ) {
            size_t removed = cb.removeFromHead(8);

            THEN( "head and bytes free change" ) {
                REQUIRE( removed == 8 );
                REQUIRE( cb.getBytesFree() == 8 );
                REQUIRE( cb.getHead() == 8 );
            }
        }

        WHEN( "20 bytes removed from head" ) {
            size_t removed = cb.removeFromHead(20);

            THEN( "head and bytes free change" ) {
                REQUIRE( removed == 16 );
                REQUIRE( cb.getBytesFree() == 16 );
                REQUIRE( cb.getHead() == 0 );
            }
        }

        WHEN( "0 bytes removed from tail" ) {
            size_t removed = cb.removeFromTail(0);

            THEN( "bytes free changes, head doesn't" ) {
                REQUIRE( removed == 0 );
                REQUIRE( cb.getBytesFree() == 0 );
                REQUIRE( cb.getHead() == 0 );
            }
        }

        WHEN( "8 bytes removed from tail" ) {
            size_t removed = cb.removeFromTail(8);

            THEN( "bytes free changes, head doesn't" ) {
                REQUIRE( removed == 8 );
                REQUIRE( cb.getBytesFree() == 8 );
                REQUIRE( cb.getHead() == 0 );
            }
        }

        WHEN( "20 bytes removed from tail" ) {
            size_t removed = cb.removeFromTail(20);

            THEN( "bytes free changes, head doesn't" ) {
                REQUIRE( removed == 16 );
                REQUIRE( cb.getBytesFree() == 16 );
                REQUIRE( cb.getHead() == 0 );
            }
        }

        WHEN( "0 bytes copied from head" ) {
            uint8_t out[16] = { 0, };
            size_t copied = cb.copy(out, 0);

            THEN( "correct bytes are copied" ) {
                REQUIRE( copied == 0 );
                REQUIRE( memcmp(out, sZeroes, 16) == 0 );
            }

            THEN( "bytes free and head don't change" ) {
                REQUIRE( cb.getBytesFree() == 0 );
                REQUIRE( cb.getHead() == 0 );
            }
        }

        WHEN( "8 bytes copied from head" ) {
            uint8_t out[16] = { 0, };
            size_t copied = cb.copy(out, 8);

            THEN( "correct bytes are copied" ) {
                REQUIRE( copied == 8 );
                REQUIRE( memcmp(out, sArr, 8) == 0 );
                REQUIRE( memcmp(&out[8], sZeroes, 8) == 0 );
            }

            THEN( "bytes free and head don't change" ) {
                REQUIRE( cb.getBytesFree() == 0 );
                REQUIRE( cb.getHead() == 0 );
            }
        }

        WHEN( "20 bytes copied from head" ) {
            uint8_t out[20] = { 0, };
            size_t copied = cb.copy(out, 20);

            THEN( "correct bytes are copied" ) {
                REQUIRE( copied == 16 );
                REQUIRE( memcmp(out, sArr, 16) == 0 );
                REQUIRE( memcmp(&out[16], sZeroes, 4) == 0 );
            }

            THEN( "bytes free and head don't change" ) {
                REQUIRE( cb.getBytesFree() == 0 );
                REQUIRE( cb.getHead() == 0 );
            }
        }

        // copying from offset in separate Scenario above

        WHEN( "0 bytes popped" ) {
            uint8_t out[16] = { 0, };
            size_t copied = cb.popFromHead(out, 0);

            THEN( "correct bytes are copied" ) {
                REQUIRE( copied == 0 );
                REQUIRE( memcmp(out, sZeroes, 16) == 0 );
            }

            THEN( "bytes free and head don't change" ) {
                REQUIRE( cb.getBytesFree() == 0 );
                REQUIRE( cb.getHead() == 0 );
            }
        }

        WHEN( "8 bytes popped" ) {
            uint8_t out[16] = { 0, };
            size_t copied = cb.popFromHead(out, 8);

            THEN( "correct bytes are copied" ) {
                REQUIRE( copied == 8 );
                REQUIRE( memcmp(out, sArr, 8) == 0 );
                REQUIRE( memcmp(&out[8], sZeroes, 8) == 0 );
            }

            THEN( "bytes free and head do change" ) {
                REQUIRE( cb.getBytesFree() == 8 );
                REQUIRE( cb.getHead() == 8 );
            }
        }

        WHEN( "20 bytes popped" ) {
            uint8_t out[20] = { 0, };
            size_t copied = cb.popFromHead(out, 20);

            THEN( "correct bytes are copied" ) {
                REQUIRE( copied == 16 );
                REQUIRE( memcmp(out, sArr, 16) == 0 );
                REQUIRE( memcmp(&out[16], sZeroes, 4) == 0 );
            }

            THEN( "bytes free and head do change" ) {
                REQUIRE( cb.getBytesFree() == 16 );
                REQUIRE( cb.getHead() == 0 );
            }
        }

        WHEN( "bytes copied from offset that is too big" ) {
            uint8_t out[16] = { 0, };
            size_t copied = cb.copy(out, 8, 16);

            THEN( "correct bytes are copied" ) {
                REQUIRE( copied == 0 );
                REQUIRE( memcmp(out, sZeroes, 8) == 0 );
            }

            THEN( "bytes free and head don't change" ) {
                REQUIRE( cb.getBytesFree() == 0 );
                REQUIRE( cb.getHead() == 0 );
            }
        }
    }
}
