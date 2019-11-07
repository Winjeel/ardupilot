
#include "ATAES132A.h"


uint16_t ATAES132A::_crc16(uint8_t const data[], size_t const sz, uint16_t crc) {
    if (data == NULL || sz == 0) {
        return crc;
    }

    uint8_t crcLSB = (crc & 0x00FF);
    uint8_t crcMSB = (crc >> 8);
    uint8_t polyLSB = 0x05;
    uint8_t polyMSB = 0x80;

    for (size_t counter = 0; counter < sz; counter++) {
        for (uint8_t shiftRegister = 0x80; shiftRegister > 0x00; shiftRegister >>= 1) {
            uint8_t dataBit = (data[counter] & shiftRegister) ? 1 : 0;
            uint8_t crcBit = crcMSB >> 7;

            // Shift CRC to the left by 1.
            uint8_t crcCarry = crcLSB >> 7;
            crcLSB <<= 1;
            crcMSB <<= 1;
            crcMSB |= crcCarry;

            if ((dataBit ^ crcBit) != 0) {
                crcLSB ^= polyLSB;
                crcMSB ^= polyMSB;
            }
        }
    }

    uint16_t result = (crcMSB & 0x00FF) | (crcLSB << 8);
    return result;
}

bool ATAES132A::_wait_until_ready(uint32_t timeout) {
    AP_HAL::HAL const &hal = AP_HAL::get_HAL();
    uint32_t const kNow = AP_HAL::millis();
    EXPECT_DELAY_MS(timeout);
    while (AP_HAL::millis() < (kNow + timeout)) {
        if (_read_status_register() &&
            ((status_register & SR_MASK_WIP) == 0)) {
            return true;
        }
        hal.scheduler->delay(1);
    }

    return false;
}

bool ATAES132A::_wait_for_response(uint32_t timeout) {
    AP_HAL::HAL const &hal = AP_HAL::get_HAL();
    uint32_t const kNow = AP_HAL::millis();
    EXPECT_DELAY_MS(timeout);
    while (AP_HAL::millis() < (kNow + timeout)) {
        if (_read_status_register() &&
            ((status_register & SR_MASK_RRDY) == SR_MASK_RRDY)) {
            return true;
        }

        hal.scheduler->delay(1);
    }

    return false;
}

bool ATAES132A::init(void) {
    const AP_HAL::HAL &hal = AP_HAL::get_HAL();
    dev = hal.spi->get_device("ATAES132A_ext");
    if (!dev) {
        return false;
    }

    sem = dev->get_semaphore();
    if (!sem) {
        return false;
    }

    WITH_SEMAPHORE(sem);
    return _wait_until_ready(1000);
}


#define LSB(x) static_cast<uint8_t>(x & 0xFF)
#define MSB(x) static_cast<uint8_t>(x >> 8)


bool ATAES132A::send_command(Command const &cmd) {
    uint8_t const kWriteReg = 0x02;
    uint16_t const kCommandAddr = 0xFE00;
    uint8_t const kHeaderSz = 3;
    uint8_t const kCount = 9 + cmd.sz;
    uint8_t const kCrcSz = 2;
    uint8_t buffer[kHeaderSz + kCount] = {
        kWriteReg,
        MSB(kCommandAddr),
        LSB(kCommandAddr),
        kCount,
        (uint8_t)cmd.opcode,
        cmd.mode,
        MSB(cmd.param1),
        LSB(cmd.param1),
        MSB(cmd.param2),
        LSB(cmd.param2),
    };

    size_t idx = kHeaderSz + kCount - kCrcSz;
    if ((cmd.sz > 0) && (cmd.data != nullptr)) {
        memcpy(&buffer[idx], cmd.data, cmd.sz);
        idx += cmd.sz;
    }

    size_t const kCountIdx = 3;
    uint16_t const crc = _crc16(&buffer[kCountIdx], buffer[kCountIdx] - kCrcSz);
    memcpy(&buffer[idx], &crc, sizeof(crc));

    WITH_SEMAPHORE(sem);
    return dev->transfer(buffer, sizeof(buffer), nullptr, 0);
}

ATAES132A::ResponseStatus ATAES132A::read_response(ReturnCode &rc, uint8_t data[], uint8_t const sz, uint32_t wait_ms) {
    uint8_t const kReadReg = 0x03;
    uint16_t const kCommandAddr = 0xFE00;
    uint8_t read_cmd[] = {
        kReadReg,
        MSB(kCommandAddr),
        LSB(kCommandAddr),
    };
    uint8_t const kHeaderSz = 2;
    uint8_t const kCrcSz = 2;
    uint8_t buffer[kHeaderSz + sz + kCrcSz];

    WITH_SEMAPHORE(sem);
    if (!_wait_for_response(wait_ms)) {
        return ResponseStatus::ResponseTimeout;
    }

    if (!dev->transfer(read_cmd, sizeof(read_cmd), buffer, sizeof(buffer))) {
        return ResponseStatus::TransferError;
    }

    size_t const kIdxCount = 0;
    size_t const kIdxReturnCode = 1;
    size_t const kIdxData = 2;
    size_t const kIdxCrc = buffer[kIdxCount] - kCrcSz;

    uint16_t const msg_crc = buffer[kIdxCrc] +
                             (static_cast<uint16_t>(buffer[kIdxCrc + 1]) << 8);
    uint16_t const calc_crc = _crc16(buffer, sizeof(buffer) - kCrcSz);
    if (msg_crc != calc_crc) {
        return ResponseStatus::CrcError;
    }

    ResponseStatus result = ResponseStatus::Ok;
    rc = static_cast<ReturnCode>(buffer[kIdxReturnCode]);
    if (rc != ReturnCode::Success) {
        result = ResponseStatus::ReturnCodeError;
    }

    uint8_t data_sz = buffer[kIdxCount] - kHeaderSz - kCrcSz;
    if (data_sz > sz) {
        data_sz = sz;
        result = ResponseStatus::BufferSzError;
    }
    memcpy(data, &buffer[kIdxData], data_sz);
    return result;
}
