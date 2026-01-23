#ifndef TEST_ARDUINO_H_
#define TEST_ARDUINO_H_

#define PROGMEM
#define pgm_read_byte(x) *(x)
#define SERIAL_8N1 0

#include <cstdio>
#include <cassert>
#include <cstdint>
#include <ctime>
#include <chrono>

// Arduino mocks
inline unsigned long _test_time = 1;

static unsigned long millis() {
    return _test_time;
}

static void yield() {
    // No-op for test
}

struct RingBuffer {
    uint8_t data[128];
    size_t readPos = 0;
    size_t writePos = 0;
    
    size_t available() const {
        if (writePos >= readPos) {
            return writePos - readPos;
        } else {
            return sizeof(data) - (readPos - writePos);
        }
    }

    size_t availableForWrite() const {
        return sizeof(data) - available() - 1;
    }

    bool write(uint8_t byte) {
        if (availableForWrite() == 0) {
            return false;
        }
        data[writePos] = byte;
        writePos = (writePos + 1) % sizeof(data);
        return true;
    }

    size_t write(const uint8_t* buffer, size_t length) {
        size_t bytesWritten = 0;
        while (bytesWritten < length && availableForWrite() > 0) {
            data[writePos] = buffer[bytesWritten];
            writePos = (writePos + 1) % sizeof(data);
            bytesWritten++;
        }
        return bytesWritten;
    }

    int read() {
        if (available() == 0) {
            return -1;
        }
        uint8_t byte = data[readPos];
        readPos = (readPos + 1) % sizeof(data);
        return byte;
    }

    size_t readBytes(uint8_t* buffer, size_t length) {
        size_t bytesRead = 0;
        while (bytesRead < length && available() > 0) {
            int byte = read();
            if (byte == -1) {
                break;
            }
            buffer[bytesRead] = static_cast<uint8_t>(byte);
            bytesRead++;
        }
        return bytesRead;
    }

    void flush() {
        readPos = writePos;
    }
};

// Mock of Serial class to allow testing of two Endpoint instances communicating via serial.
class SerialMock {
public:
    RingBuffer& rxBuffer;
    RingBuffer& txBuffer;
    SerialMock(RingBuffer& rx, RingBuffer& tx) : rxBuffer(rx), txBuffer(tx) {}
    int available() { return rxBuffer.available(); }
    size_t availableForWrite() { return txBuffer.availableForWrite(); }
    size_t readBytes(uint8_t* buffer, size_t length) { return rxBuffer.readBytes(buffer, length); }
    size_t write(const uint8_t* buffer, size_t length) { return txBuffer.write(buffer, length); }
    void begin(unsigned long baud, int config) {}
    void setTimeout(unsigned long timeout) {}
    void flush() { rxBuffer.flush(); }
    int read() { return rxBuffer.read(); }
};

// Provide default Serial instance expected by Arduino sketches.
static RingBuffer SerialRxBuffer {};
static RingBuffer SerialTxBuffer {};
static SerialMock Serial { SerialRxBuffer, SerialTxBuffer };

#endif  // TEST_ARDUINO_H_