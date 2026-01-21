#include "Endpoint.h"

#if defined(ARDUINO_AVR_NANO)
#include <stddef.h>
#else
#include <cstddef>
#include <algorithm>
using std::min;
#endif

namespace serial_transport {

// CRC-8-CCITT lookup table (polynomial 0x07, initial value 0x00)
const uint8_t Endpoint::CRC8_TABLE[256] PROGMEM = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x1D, 0x1A, 0x13, 0x14, 0x01, 0x06, 0x0F, 0x08,
    0x25, 0x22, 0x2B, 0x2C, 0x39, 0x3E, 0x37, 0x30,
    0xAC, 0xAB, 0xA2, 0xA5, 0xB0, 0xB7, 0xBE, 0xB9,
    0x94, 0x93, 0x9A, 0x9D, 0x88, 0x8F, 0x86, 0x81,
    0xDC, 0xDB, 0xD2, 0xD5, 0xC0, 0xC7, 0xCE, 0xC9,
    0xE4, 0xE3, 0xEA, 0xED, 0xF8, 0xFF, 0xF6, 0xF1,
    0x5C, 0x5B, 0x52, 0x55, 0x40, 0x47, 0x4E, 0x49,
    0x64, 0x63, 0x6A, 0x6D, 0x78, 0x7F, 0x76, 0x71,
    0x2C, 0x2B, 0x22, 0x25, 0x30, 0x37, 0x3E, 0x39,
    0x14, 0x13, 0x1A, 0x1D, 0x08, 0x0F, 0x06, 0x01
};

const char* describe(ErrorCode errorCode) {
    switch (errorCode) {
        case ErrorCode::ResendLimitExceeded: return "Resend limit exceeded";
        case ErrorCode::BufferFull: return "Receive buffer full";
        default: return "Unknown error code";
    }
}

Endpoint::Endpoint(ReceiveCallback processReceived, ErrorCallback handleError) :
    _processReceived(processReceived),
    _handleError(handleError)
{}

void Endpoint::setup(unsigned long baud, SerialConfig serialMode) {
    Serial.begin(baud, serialMode);
    Serial.setTimeout(2);
}

void Endpoint::reset() {
    _rxBufferSize = 0u;
    _lastTxQueueIndex = 0u;
    _currentTxSequenceNumber = 0u;
    _currentRxSequenceNumber = 0u;
    _currentUnacknowledgedTxQueueIndex = 0u;
    for (auto& message : _txQueue) {
        message = QueuedMessage{};
    }
    
    Serial.flush();
    size_t available = Serial.available();
    while (available-- > 0) {
        Serial.read();
        yield();
    }
}

void Endpoint::loop() {
    receive();

    if (_timeoutEnabled && txMessageQueued()) {
        QueuedMessage& currentMessage = currentUnacknowledgedMessage();
        if ((currentMessage.sendTime + _timeout) < millis()) {
            if (currentMessage.retries > 0u) {
                send(currentMessage);
                currentMessage.retries -= 1u;
            } else {
                _handleError(ErrorCode::ResendLimitExceeded, *this);
                acknowledgeCurrentUnacknowledgedMessage();
            }
        }
    }
}

bool Endpoint::canQueue() const {
    return _txQueue[nextTxQueueIndex()].acknowledged;
}

bool Endpoint::canSend(size_t queueIndex) const {
    return _currentUnacknowledgedTxQueueIndex == queueIndex;
}

bool Endpoint::queue(const char* fmt, ...) {
    if (!canQueue()) {
        return false;
    }

    char payload[PAYLOAD_MAX_SIZE] = {};

    va_list args;
    va_start(args, fmt);
    int payloadSize = vsnprintf(payload, PAYLOAD_MAX_SIZE, fmt, args);
    va_end(args);

    if (payloadSize < 0 || payloadSize >= static_cast<int>(PAYLOAD_MAX_SIZE)) {
        return false;
    }

    QueuedMessage& message = _txQueue[incrementLastTxQueueIndex()];
    message.sequenceNumber = incrementTxSequenceNumber();
    message.acknowledged = false;
    message.sendTime = 0u;
    message.retries = _resendLimit;
    message.payloadSize = static_cast<size_t>(payloadSize);
    memcpy(message.payload, payload, message.payloadSize);

    if (canSend(_lastTxQueueIndex)) {
        send(message);
    }

    return true;
}

uint8_t Endpoint::calculateCrc8(const uint8_t* data, size_t length) const {
    uint8_t crc = 0x00u;
    for (size_t i = 0; i < length; ++i) {
        uint8_t tableIndex = crc ^ data[i];
        crc = pgm_read_byte(&CRC8_TABLE[tableIndex]);
    }
    return crc;
}

void Endpoint::sendFrame(uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t payloadLen) {
    uint8_t frameData[BUFFER_MAX_SIZE];
    frameData[0] = SYNC1;
    frameData[1] = SYNC2;
    frameData[2] = type;
    frameData[3] = payloadLen;
    frameData[4] = seq;
    
    if (payloadLen > 0 && payload != nullptr) {
        memcpy(&frameData[5], payload, payloadLen);
    }
    
    uint8_t crc = calculateCrc8(&frameData[2], 3 + payloadLen);
    frameData[5 + payloadLen] = crc;
    
    Serial.write(frameData, 6 + payloadLen);
}

void Endpoint::send(QueuedMessage& message) {
    message.acknowledged = false;
    message.sendTime = millis();
    
    sendFrame(FRAME_TYPE_DATA, message.sequenceNumber, 
              reinterpret_cast<const uint8_t*>(message.payload), 
              static_cast<uint8_t>(message.payloadSize));
}

void Endpoint::sendAcknowledge(uint8_t sequenceNumber) {
    sendFrame(FRAME_TYPE_ACK, sequenceNumber, nullptr, 0);
}

const Endpoint::QueuedMessage& Endpoint::currentUnacknowledgedMessage() const {
    return _txQueue[_currentUnacknowledgedTxQueueIndex];
}

Endpoint::QueuedMessage& Endpoint::currentUnacknowledgedMessage() {
    return _txQueue[_currentUnacknowledgedTxQueueIndex];
}

void Endpoint::acknowledgeCurrentUnacknowledgedMessage() {
    currentUnacknowledgedMessage().acknowledged = true;
    incrementCurrentUnacknowledgedTxQueueIndex();
}

bool Endpoint::txMessageQueued() const {
    return !currentUnacknowledgedMessage().acknowledged;
}

size_t Endpoint::nextTxQueueIndex() const {
    return (_lastTxQueueIndex + 1u) % TX_QUEUE_SIZE;
}

size_t Endpoint::incrementLastTxQueueIndex() {
    _lastTxQueueIndex = nextTxQueueIndex();

    while (!txMessageQueued() && _currentUnacknowledgedTxQueueIndex != _lastTxQueueIndex) {
        acknowledgeCurrentUnacknowledgedMessage();
    }
    return _lastTxQueueIndex;
}

size_t Endpoint::incrementCurrentUnacknowledgedTxQueueIndex() {
    if (_currentUnacknowledgedTxQueueIndex == _lastTxQueueIndex) {
        return _currentUnacknowledgedTxQueueIndex;
    } else {
        return _currentUnacknowledgedTxQueueIndex = (_currentUnacknowledgedTxQueueIndex + 1u) % TX_QUEUE_SIZE;
    }
}

uint8_t Endpoint::incrementTxSequenceNumber() {
    return ++_currentTxSequenceNumber;
}

uint8_t Endpoint::incrementRxSequenceNumber() {
    return ++_currentRxSequenceNumber;
}

void Endpoint::receive() {
    if (Serial.available() == 0) {
        return;
    }

    // Try to read available bytes into buffer
    size_t availableSpace = BUFFER_MAX_SIZE - _rxBufferSize;
    if (availableSpace == 0) {
        // Buffer is full, signal error and discard oldest byte to recover
        _handleError(ErrorCode::BufferFull, *this);
        _rxBufferSize = BUFFER_MAX_SIZE - 1;
        memmove(_rxBuffer, _rxBuffer + 1, _rxBufferSize);
        availableSpace = 1;
    }

    size_t bytesRead = Serial.readBytes(&_rxBuffer[_rxBufferSize], availableSpace);
    _rxBufferSize += bytesRead;

    // State machine to parse binary frames
    while (_rxBufferSize >= 6) {  // Minimum frame size: SYNC1 + SYNC2 + TYPE + LENGTH + SEQ + CRC8
        // Look for sync bytes
        size_t syncIndex = 0;
        bool foundSync = false;
        
        for (syncIndex = 0; syncIndex < _rxBufferSize - 1; ++syncIndex) {
            if (_rxBuffer[syncIndex] == SYNC1 && _rxBuffer[syncIndex + 1] == SYNC2) {
                foundSync = true;
                break;
            }
        }

        if (!foundSync) {
            // No sync found, discard everything
            _rxBufferSize = 0;
            return;
        }

        if (syncIndex > 0) {
            // Discard bytes before sync
            memmove(_rxBuffer, &_rxBuffer[syncIndex], _rxBufferSize - syncIndex);
            _rxBufferSize -= syncIndex;
        }

        if (_rxBufferSize < 6) {
            // Not enough data for frame header
            return;
        }

        uint8_t type = _rxBuffer[2];
        uint8_t length = _rxBuffer[3];
        uint8_t seq = _rxBuffer[4];

        // Validate length
        if (length > PAYLOAD_MAX_SIZE) {
            // Invalid length, discard sync bytes and look for next frame
            memmove(_rxBuffer, &_rxBuffer[2], _rxBufferSize - 2);
            _rxBufferSize -= 2;
            continue;
        }

        size_t frameSize = 6 + length;  // SYNC1 + SYNC2 + TYPE + LENGTH + SEQ + PAYLOAD + CRC8
        if (_rxBufferSize < frameSize) {
            // Not enough data for complete frame
            return;
        }

        // Verify CRC (covers TYPE + LENGTH + SEQ + PAYLOAD, not the sync bytes or CRC itself)
        uint8_t expectedCrc = calculateCrc8(&_rxBuffer[2], 3 + length);
        uint8_t receivedCrc = _rxBuffer[5 + length];

        if (expectedCrc != receivedCrc) {
            // CRC mismatch, discard sync bytes and look for next frame
            memmove(_rxBuffer, &_rxBuffer[2], _rxBufferSize - 2);
            _rxBufferSize -= 2;
            continue;
        }

        // Frame is valid, extract and handle it
        uint8_t* payload = (length > 0) ? &_rxBuffer[5] : nullptr;
        handleFrame(type, seq, payload, length);

        // Remove processed frame from buffer
        memmove(_rxBuffer, &_rxBuffer[frameSize], _rxBufferSize - frameSize);
        _rxBufferSize -= frameSize;
    }
}

void Endpoint::handleFrame(uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t payloadLen) {
    switch (type) {
        case FRAME_TYPE_DATA: {
            const uint8_t expectedSeq = _currentRxSequenceNumber + 1u; // Expect next in-order frame
            if (seq == expectedSeq) {
                _currentRxSequenceNumber = seq;
                sendAcknowledge(seq);

                // Null-terminate payload and process it
                if (payloadLen > 0 && payloadLen < PAYLOAD_MAX_SIZE) {
                    char message[PAYLOAD_MAX_SIZE + 1];
                    memcpy(message, payload, payloadLen);
                    message[payloadLen] = '\0';
                    _processReceived(message, *this);
                } else if (payloadLen == 0) {
                    // Empty message (shouldn't happen normally, but handle gracefully)
                    char emptyMsg[1] = { '\0' };
                    _processReceived(emptyMsg, *this);
                }
            } else {
                // Out of sequence, send last ACK again to trigger a re-send
                sendAcknowledge(_currentRxSequenceNumber);
            }
            break;
        }
        case FRAME_TYPE_ACK: {
            handleAcknowledge(seq);
            break;
        }
        case FRAME_TYPE_TIMEOUT_CONTROL: {
            if (payloadLen == 1) {
                handleTimeoutControl(payload[0]);
            }
            break;
        }
    }
}

void Endpoint::handleAcknowledge(uint8_t sequenceNumber) {
    QueuedMessage& currentMessage = currentUnacknowledgedMessage();
    if (!currentMessage.acknowledged && currentMessage.sequenceNumber == sequenceNumber) {
        acknowledgeCurrentUnacknowledgedMessage();
        QueuedMessage& nextMessage = currentUnacknowledgedMessage();
        if (!nextMessage.acknowledged && canSend(_currentUnacknowledgedTxQueueIndex)) {
            send(nextMessage);
        }
    }
}

void Endpoint::handleTimeoutControl(uint8_t operation) {
    if (operation == '+') {
        _timeoutEnabled = true;
    } else if (operation == '-') {
        _timeoutEnabled = false;
    }
}

bool queueCanMessage(Endpoint& serial, const char* direction, uint32_t id, bool ext, bool rtr, uint8_t length, const uint8_t (&data)[8]) {
    return serial.queue("CAN%s %08lX %u %02X %02X %02X %02X %02X %02X %02X %02X",
        direction,
        id | (uint32_t(ext) << 31) | (uint32_t(rtr) << 30),
        length,
        data[0],
        data[1],
        data[2],
        data[3],
        data[4],
        data[5],
        data[6],
        data[7]
      );
}

bool queueCanTxMessage(Endpoint& serial, uint32_t id, bool ext, bool rtr, uint8_t length, const uint8_t (&data)[8]) {
    return queueCanMessage(serial, "TX", id, ext, rtr, length, data);
}

bool queueCanRxMessage(Endpoint& serial, uint32_t id, bool ext, bool rtr, uint8_t length, const uint8_t (&data)[8]) {
    return queueCanMessage(serial, "RX", id, ext, rtr, length, data);
}

}
