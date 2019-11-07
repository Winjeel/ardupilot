#pragma once


#include "AP_HAL/AP_HAL.h"


class ATAES132A {
public:
    ATAES132A(void) {}

    /* Do not allow copies */
    ATAES132A(const ATAES132A &other) = delete;
    ATAES132A &operator=(const ATAES132A&) = delete;


    bool init(void);

    enum class Opcode : uint8_t {
        Info = 0x0C,
    };
    typedef struct {
        Opcode opcode;
        uint8_t mode;
        uint16_t param1;
        uint16_t param2;
        uint8_t data[];
        uint8_t sz;
    } Command;
    bool send_command(Command const &cmd);

    enum class ReturnCode : uint8_t {
        Success       = 0x00,
        BoundaryError = 0x02,
        RWConfig      = 0x04,
        BadAddr       = 0x08,
        CountErr      = 0x10,
        NonceError    = 0x20,
        MacError      = 0x40,
        ParseError    = 0x50,
        DataMatch     = 0x60,
        LockError     = 0x70,
        KeyErr        = 0x80,
    };
    enum class ResponseStatus {
        Ok,
        ResponseTimeout,
        TransferError,
        CrcError,
        ReturnCodeError,
        BufferSzError,
    };
    ResponseStatus read_response(ReturnCode &rc, uint8_t data[], uint8_t const sz, uint32_t wait_ms = 100);

private:
    uint8_t SR_MASK_WIP  = (1 << 0);
    uint8_t SR_MASK_WEN  = (1 << 1);
    uint8_t SR_MASK_WAKE = (1 << 2);
    uint8_t SR_MASK_CRCE = (1 << 4);
    uint8_t SR_MASK_RRDY = (1 << 6);
    uint8_t SR_MASK_EERR = (1 << 7);

    static uint16_t _crc16(uint8_t const data[], size_t const sz, uint16_t crc = 0);

    // pre-condition: semaphore has been taken
    inline bool _read_status_register(void) {
        uint8_t com[] = { 0x03, 0xFF, 0xF0, };
        return dev->transfer(com, sizeof(com), &status_register, sizeof(status_register));
    }

    // pre-condition: semaphore has been taken
    bool _wait_until_ready(uint32_t timeout);
    // pre-condition: semaphore has been taken
    bool _wait_for_response(uint32_t timeout);

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    AP_HAL::Semaphore* sem;
    uint8_t status_register;
};
