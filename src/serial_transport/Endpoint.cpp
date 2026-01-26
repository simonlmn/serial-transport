#include "Endpoint.h"

#if defined(ARDUINO_AVR_NANO)
#include <stddef.h>
#else
#include <cstddef>
#include <algorithm>
#endif

#include <toolbox/Formatter.h>

namespace serial_transport {

// CRC-8-CCITT lookup table (polynomial 0x07, initial value 0x00)
static const uint8_t CRC8_TABLE[256] PROGMEM = {
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

toolbox::strref describe(ConnectionState state)  {
    switch (state) {
        case ConnectionState::CLOSED:     return F("Endpoint is closed.");
        case ConnectionState::LISTENING:  return F("Listening for incoming connection.");
        case ConnectionState::WAITING:    return F("Waiting for connection to establish.");
        case ConnectionState::CONNECTED:  return F("Connected to remote endpoint.");
        default:                          return F("UNKNOWN");
    }
}

unsigned long calculateTimeout(unsigned long baudRate) {
    // Calculate timeout based on baud rate for optimal responsiveness
    // Max frame: 64 bytes × 10 bits/byte = 640 bits
    // Round-trip time (send DATA + receive ACK) with 5 times safety margin for processing/jitter
    unsigned long timeout = (640UL * 1000UL * 5UL) / baudRate;
    
    // Clamp to reasonable range: 20-500ms
    if (timeout < 20UL) timeout = 20UL;
    if (timeout > 500UL) timeout = 500UL;
    
    return timeout;
}

constexpr uint8_t wrapSequenceNumber(int sequenceNumber) {
    return static_cast<uint8_t>(sequenceNumber & 0xFFu);
}

Endpoint::Endpoint(EndpointRole role, SerialType& serial, ReceiveCallback receive, StateCallback notifyState, FrameCallback frame) :
    _role(role),
    _serial(serial),
    _receive(receive),
    _notifyState(notifyState),
    _frame(frame)
{}

void Endpoint::setup(unsigned long baud, SerialConfig serialMode) {
    _timeout = calculateTimeout(baud);
    _serial.begin(baud, serialMode);
    _serial.setTimeout(2);
    reset();
}

void Endpoint::reset() {
    _lastSynSendTime = 0u;

    _rxBufferSize = 0u;
    _lastRxSequenceNumber = 0u;

    for (auto& message : _txQueue) {
        message = QueuedMessage{};
    }
    _lastTxQueueIndex = 0u;
    _firstTxQueueIndex = 0u;
    
    _serial.flush();
    size_t available = _serial.available();
    while (available-- > 0) {
        _serial.read();
        yield();
    }
    setConnectionState(ConnectionState::CLOSED);
}

void Endpoint::setConnectionState(ConnectionState state) {
    if (_connectionState != state) {
        _connectionState = state;
        if (_notifyState) {
            _notifyState(state, *this);    
        }

        if (diagnosticEnabled(DIAG_CONNECTION_STATE)) {
            sendDebug(toolbox::format(F("CS=%u"), static_cast<uint8_t>(state)));
        }
    }
}

void Endpoint::loop() {
    if (_connectionState != ConnectionState::CLOSED) {
        receive();
    }

    switch (_connectionState) {
        case ConnectionState::CLOSED:
            if (_role == EndpointRole::SERVER) {
                setConnectionState(ConnectionState::LISTENING);
            } else {
                sendSyn();
                setConnectionState(ConnectionState::WAITING);
            }
            break;
        case ConnectionState::LISTENING:
            // Wait for incoming SYN
            break;
        case ConnectionState::WAITING:
            // Resend SYN/SYNACK periodically
            if (millis() - _lastSynSendTime >= _timeout) {
                if (_role == EndpointRole::SERVER) {
                    sendSynAcknowledge();
                } else {
                    sendSyn();
                }
            }
            break;
        case ConnectionState::CONNECTED:
            trySend();
            if (diagnosticEnabled(DIAG_PERIODIC_STATS) && (_lastDiagnosticsTime == 0u || (millis() - _lastDiagnosticsTime) >= 1000u)) {
                sendDiagnostics();
            }
            break;
    }
}

void Endpoint::receive() {
    if (_serial.available() == 0) {
        return;
    }

    // Try to read available bytes into buffer
    const uint8_t availableSpace = BUFFER_MAX_SIZE - _rxBufferSize;
    if (availableSpace == 0) {
        // Buffer is full - we've lost sync, discard everything and start fresh
        _rxBufferSize = 0;
        return;
    }

    const size_t bytesRead = _serial.readBytes(&_rxBuffer[_rxBufferSize], availableSpace);
    _rxBufferSize = static_cast<uint8_t>(_rxBufferSize + bytesRead);

    // State machine to parse binary frames
    while (_rxBufferSize >= FRAME_MIN_SIZE) {
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

        if (_rxBufferSize < FRAME_MIN_SIZE) {
            // Not enough data for frame header
            return;
        }

        uint8_t type = _rxBuffer[2];
        uint8_t length = _rxBuffer[3];
        uint8_t sequenceNumber = _rxBuffer[4];

        // Validate length
        if (length > PAYLOAD_MAX_SIZE) {
            // Invalid length, discard sync bytes and look for next frame
            memmove(_rxBuffer, &_rxBuffer[FRAME_SYNC_SIZE], _rxBufferSize - FRAME_SYNC_SIZE);
            _rxBufferSize -= FRAME_SYNC_SIZE;
            continue;
        }

        const size_t frameSize = FRAME_OVERHEAD + static_cast<size_t>(length);
        if (_rxBufferSize < frameSize) {
            // Not enough data for complete frame
            return;
        }

        // Verify CRC (covers TYPE + LENGTH + SEQ + PAYLOAD, not the sync bytes or CRC itself)
        uint8_t expectedCrc = calculateCrc8(&_rxBuffer[FRAME_SYNC_SIZE], FRAME_META_SIZE + length);
        uint8_t receivedCrc = _rxBuffer[FRAME_HEADER_SIZE + length];

        if (expectedCrc != receivedCrc) {
            // CRC mismatch, discard sync bytes and look for next frame
            memmove(_rxBuffer, &_rxBuffer[FRAME_SYNC_SIZE], _rxBufferSize - FRAME_SYNC_SIZE);
            _rxBufferSize -= FRAME_SYNC_SIZE;
            continue;
        }

        // Frame is valid, extract and handle it
        // We can directly write a zero-termination at the end of the payload (overwriting CRC byte),
        // as we have already checked it and will discard the frame afterwards anyway. This allows
        // direct handling of payload as string in callbacks.
        uint8_t* payload = &_rxBuffer[FRAME_HEADER_SIZE];
        payload[length] = '\0';
        handleFrame(type, sequenceNumber, payload, length);

        // Note: this condition also guards against invalid length calculation due to a reset() inside handleFrame()
        if (_rxBufferSize > frameSize) {
            // Remove processed frame from buffer
            memmove(_rxBuffer, &_rxBuffer[frameSize], _rxBufferSize - frameSize);
            _rxBufferSize -= frameSize;
        } else {
            // No more data in buffer
            _rxBufferSize = 0;
        }
    }
}

uint8_t Endpoint::calculateCrc8(const uint8_t* data, size_t length) const {
    uint8_t crc = 0x00u;
    for (size_t i = 0; i < length; ++i) {
        uint8_t tableIndex = crc ^ data[i];
        crc = pgm_read_byte(&CRC8_TABLE[tableIndex]);
    }
    return crc;
}

void Endpoint::handleFrame(uint8_t type, uint8_t sequenceNumber, const uint8_t* payload, uint8_t payloadLen) {
    if ((type == FRAME_TYPE_DATA && diagnosticEnabled(DIAG_RX_DATA_FRAMES))
        || (type == FRAME_TYPE_ACK && diagnosticEnabled(DIAG_RX_ACK_FRAMES))
        || (type == FRAME_TYPE_SYN && diagnosticEnabled(DIAG_RX_SYN_FRAMES))
        || (type == FRAME_TYPE_SYNACK && diagnosticEnabled(DIAG_RX_SYNACK_FRAMES))
        || (type == FRAME_TYPE_RST && diagnosticEnabled(DIAG_RX_RST_FRAMES))
    ) {
        sendDebug(toolbox::format(F("RX:T=%02X|S=%u|L=%u"), type, sequenceNumber, payloadLen));
    }
    if (_frame) {
        _frame('R', type, sequenceNumber, payload, payloadLen);
    }
    switch (type) {
        case FRAME_TYPE_DATA: {
            handleData(sequenceNumber, payload, payloadLen);
            break;
        }

        case FRAME_TYPE_ACK: {
            handleAcknowledge(sequenceNumber);
            break;
        }

        case FRAME_TYPE_SYN: {
            handleSyn(sequenceNumber);
            break;
        }

        case FRAME_TYPE_SYNACK: {
            if (payloadLen == 1u) {
                handleSynAcknowledge(sequenceNumber, payload[0]);
            }
            break;
        }

        case FRAME_TYPE_RST: {
            handleReset();
            break;
        }

        case FRAME_TYPE_DBG: {
            // Ignore, debug frames are not acknowledged
            // Can be handled via FrameCallback if needed
            break;
        }
    }
}

void Endpoint::handleData(uint8_t sequenceNumber, const uint8_t* payload, uint8_t payloadLen) {
    switch (_connectionState) {
        case ConnectionState::CLOSED:
        case ConnectionState::LISTENING:
            // Remote sending DATA when not connected, send RST
            sendReset();
            if (diagnosticEnabled(DIAG_RESETS)) {
                sendDebug(F("RST:RXDATA"));
            }
            break;
        case ConnectionState::WAITING:
            // Ignore DATA when in SYN/ACK phase
            return;
        case ConnectionState::CONNECTED:
            if (sequenceNumber == wrapSequenceNumber(_lastRxSequenceNumber + 1u)) {
                _lastRxSequenceNumber = sequenceNumber;
                sendAcknowledge(sequenceNumber);
                if (_receive) _receive(payload, payloadLen, *this);
            } else {
                // Duplicate or out of order (likely missed frame): ACK last known again
                sendAcknowledge(_lastRxSequenceNumber);
            }
            break;
    }
}

void Endpoint::handleSyn(uint8_t sequenceNumber) {
    if (_role != EndpointRole::SERVER) {
        return;
    }
    
    switch (_connectionState) {
        case ConnectionState::CLOSED:
            // Ignore SYN when closed
            break;
        case ConnectionState::LISTENING:
            // sequenceNumber is the sequence number the other side will use for its next DATA frame
            _lastRxSequenceNumber = wrapSequenceNumber(sequenceNumber - 1u);
            sendSynAcknowledge();
            setConnectionState(ConnectionState::WAITING);
            break;
        case ConnectionState::WAITING:
            // Retransmitted SYN, resend SYNACK
            sendSynAcknowledge();
            break;
        case ConnectionState::CONNECTED:
            // Remote was disconnected, reset
            reset();
            break;
    }
}

void Endpoint::handleSynAcknowledge(uint8_t sequenceNumber, uint8_t acknowledgedSeqNumber) {
    if (_role != EndpointRole::CLIENT) {
        return;
    }

    switch (_connectionState) {
        case ConnectionState::CLOSED:
            // Ignore SYNACK when closed
            break;
        case ConnectionState::LISTENING:
            // Client should not be in LISTENING state!
            reset();
            sendReset();
            if (diagnosticEnabled(DIAG_RESETS)) {
                sendDebug(F("RST:SYNACK"));
            }
            break;
        case ConnectionState::WAITING:
            // Verify that our SYN was acknowledged
            if (acknowledgedSeqNumber == wrapSequenceNumber(nextTxSequenceNumberToSend() - 1u)) {
                // sequenceNumber is the sequence number the other side will use for its next DATA frame
                _lastRxSequenceNumber = wrapSequenceNumber(sequenceNumber - 1u);
                sendAcknowledge(_lastRxSequenceNumber);
                setConnectionState(ConnectionState::CONNECTED);
            }
            break;
        case ConnectionState::CONNECTED:
            // Repeat ACK if already connected, remote might have missed it
            sendAcknowledge(wrapSequenceNumber(sequenceNumber - 1u));
            break;
    }
}

void Endpoint::handleAcknowledge(uint8_t sequenceNumber) {
    switch (_connectionState) {
        case ConnectionState::CLOSED:
            // Ignore ACKs when closed
            break;
        case ConnectionState::LISTENING:
            // Remote sending ACK when not connected, send RST
            sendReset();
            if (diagnosticEnabled(DIAG_RESETS)) {
                sendDebug(F("RST:ACK"));
            }
            break;
        case ConnectionState::WAITING:
            if (_role == EndpointRole::SERVER) {
                // Verify that our SYNACK was acknowledged
                if (sequenceNumber == wrapSequenceNumber(nextTxSequenceNumberToSend() - 1u)) {
                    setConnectionState(ConnectionState::CONNECTED);
                }
            }
            break;
        case ConnectionState::CONNECTED:
            // Normal operation: mark message as acknowledged
            if (hasQueuedTxMessage()) {
                QueuedMessage& message = firstTxQueueMessage();
                if (message.acknowledged == 0u && message.sequenceNumber == sequenceNumber) {
                    message.acknowledged = 1u;
                    incrementFirstTxQueueIndex();
                    trySend();
                }
            }
            break;
    }
}

void Endpoint::handleReset() {
    reset();
}

void Endpoint::incrementFirstTxQueueIndex() {
    if (_firstTxQueueIndex == _lastTxQueueIndex) {
        return;
    }
    _firstTxQueueIndex = (_firstTxQueueIndex + 1u) % TX_QUEUE_SIZE;
}

void Endpoint::trySend() {
    if (_connectionState != ConnectionState::CONNECTED) {
        return;
    }
    if (!hasQueuedTxMessage()) {
        return;
    }

    QueuedMessage& message = firstTxQueueMessage();
    // Only apply timeout delay for retries (sendTime != 0)
    // New messages (sendTime == 0) should be sent immediately
    if (message.sendTime != 0u && millis() < (message.sendTime + _timeout)) {
        return;
    }
    if (!sendData(message)) {
        return;
    }
    if (message.retries == 0u) {
        reset();
        sendReset();
        if (diagnosticEnabled(DIAG_RESETS)) {
            sendDebug(F("RST:TXRL"));
        }
    }
}

void Endpoint::sendSyn() {
    sendFrame(FRAME_TYPE_SYN, nextTxSequenceNumberToSend(), nullptr, 0);
    _lastSynSendTime = millis();
}

void Endpoint::sendSynAcknowledge() {
    _txFrameBuffer[0] = _lastRxSequenceNumber;
    sendFrame(FRAME_TYPE_SYNACK, nextTxSequenceNumberToSend(), _txFrameBuffer, 1);
    _lastSynSendTime = millis();
}

void Endpoint::sendAcknowledge(uint8_t sequenceNumber) {
    sendFrame(FRAME_TYPE_ACK, sequenceNumber, nullptr, 0);
}

void Endpoint::sendReset() {
    sendFrame(FRAME_TYPE_RST, 0, nullptr, 0);
}

bool Endpoint::sendData(QueuedMessage& message) {
    if (canWrite(message))
    {
        return false;
    }

    sendFrame(FRAME_TYPE_DATA, message.sequenceNumber, 
              reinterpret_cast<const uint8_t*>(message.payload), 
              static_cast<uint8_t>(message.payloadSize));
    
    message.sendTime = millis();
    message.retries -= 1u;

    return true;
}

bool Endpoint::canWrite(const serial_transport::Endpoint::QueuedMessage& message)
{
    return _serial.availableForWrite() < ((message.payloadSize / 2) + FRAME_OVERHEAD);
}

void Endpoint::sendFrame(uint8_t type, uint8_t sequenceNumber, const uint8_t* payload, uint8_t payloadLen) {
    if (payloadLen > PAYLOAD_MAX_SIZE) {
        return;
    }
    
    if (_frame) {
        _frame('T', type, sequenceNumber, payload, payloadLen);
    }

    // We copy the payload first, as payload might use the same buffer as _txFrameBuffer
    if (payloadLen > 0 && payload != nullptr) {
        memmove(&_txFrameBuffer[5], payload, payloadLen);
    }
    
    _txFrameBuffer[0] = SYNC1;
    _txFrameBuffer[1] = SYNC2;
    _txFrameBuffer[2] = type;
    _txFrameBuffer[3] = payloadLen;
    _txFrameBuffer[4] = sequenceNumber;

    uint8_t crc = calculateCrc8(&_txFrameBuffer[2], 3 + payloadLen);
    _txFrameBuffer[5 + payloadLen] = crc;
    
    _serial.write(_txFrameBuffer, FRAME_OVERHEAD + payloadLen);
}

bool Endpoint::queue(const toolbox::strref& data) {
    if (!canQueue()) {
        return false;
    }

    if (data.length() > static_cast<size_t>(PAYLOAD_MAX_SIZE)) {
        return false;
    }

    // Since we checked canQueue(), this will always be safe to already use the next message slot for payload formatting
    QueuedMessage& message = _txQueue[nextTxQueueIndex()];
    message.sequenceNumber = wrapSequenceNumber(lastTxSequenceNumber() + 1u);
    message.acknowledged = 0u;
    message.sendTime = 0u;
    message.retries = _resendLimit;
    message.payloadSize = static_cast<uint8_t>(data.length());
    data.copy(reinterpret_cast<char*>(message.payload), PAYLOAD_MAX_SIZE, false);

    const uint8_t queueIndex = incrementLastTxQueueIndex(); // Advance the last index to point to the newly added message
    if (isFirstInQueue(queueIndex)) {
       trySend();
    }

    return true;
}

bool Endpoint::canQueue() const {
    return _txQueue[nextTxQueueIndex()].acknowledged != 0u;
}

bool Endpoint::hasQueuedTxMessage() const {
    return firstTxQueueMessage().acknowledged == 0u;
}

uint8_t Endpoint::numberOfQueuedMessages() const {
    if (!hasQueuedTxMessage()) {
        return 0u;
    }

    if (_lastTxQueueIndex >= _firstTxQueueIndex) {
        return static_cast<uint8_t>(_lastTxQueueIndex - _firstTxQueueIndex + 1u);
    } else {
        return static_cast<uint8_t>((TX_QUEUE_SIZE - _firstTxQueueIndex) + (_lastTxQueueIndex + 1u));
    }
}

bool Endpoint::isFirstInQueue(uint8_t queueIndex) const {
    return _firstTxQueueIndex == queueIndex;
}

const Endpoint::QueuedMessage& Endpoint::firstTxQueueMessage() const {
    return _txQueue[_firstTxQueueIndex];
}

Endpoint::QueuedMessage& Endpoint::firstTxQueueMessage() {
    return _txQueue[_firstTxQueueIndex];
}

uint8_t Endpoint::firstTxSequenceNumber() const {
    return _txQueue[_firstTxQueueIndex].sequenceNumber;
}

uint8_t Endpoint::nextTxQueueIndex() const {
    return static_cast<uint8_t>((_lastTxQueueIndex + 1u) % TX_QUEUE_SIZE);
}

uint8_t Endpoint::incrementLastTxQueueIndex() {
    if (hasQueuedTxMessage()) {
        _lastTxQueueIndex = nextTxQueueIndex();
    } else {
        _lastTxQueueIndex = _firstTxQueueIndex = nextTxQueueIndex();
    }
    return _lastTxQueueIndex;
}

uint8_t Endpoint::lastTxSequenceNumber() const {
    return _txQueue[_lastTxQueueIndex].sequenceNumber;
}

uint8_t Endpoint::nextTxSequenceNumberToSend() const {
    if (hasQueuedTxMessage()) {
        return firstTxSequenceNumber();
    } else {
        return lastTxSequenceNumber() + 1u;
    }
}

void Endpoint::sendDebug(const toolbox::strref& message) {
    if (message.length() > static_cast<size_t>(PAYLOAD_MAX_SIZE)) {
        return;
    }

    message.copy(reinterpret_cast<char*>(_txFrameBuffer), PAYLOAD_MAX_SIZE, false);
    
    sendFrame(FRAME_TYPE_DBG, 0u, reinterpret_cast<const uint8_t*>(_txFrameBuffer), static_cast<uint8_t>(message.length()));
}

void Endpoint::sendDiagnostics() {
    _lastDiagnosticsTime = millis();
    sendDebug(toolbox::format(F("RxS=%u|Tx^=%u|Tx$=%u|#Tx=%u|TxS=%u|Q?=%c|W?=%c"),
        _lastRxSequenceNumber,
        _firstTxQueueIndex,
        _lastTxQueueIndex,
        numberOfQueuedMessages(),
        firstTxSequenceNumber(),
        hasQueuedTxMessage() ? 'Y' : 'N',
        canWrite(firstTxQueueMessage()) ? 'Y' : 'N'
    ));
}

bool queueCanMessage(Endpoint& serial, char direction, uint32_t id, bool ext, bool rtr, uint8_t length, const uint8_t (&data)[8]) {
    return serial.queue(toolbox::format(F("CAN%cX %08lX %u %02X %02X %02X %02X %02X %02X %02X %02X"),
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
      ));
}

bool queueCanTxMessage(Endpoint& serial, uint32_t id, bool ext, bool rtr, uint8_t length, const uint8_t (&data)[8]) {
    return queueCanMessage(serial, 'T', id, ext, rtr, length, data);
}

bool queueCanRxMessage(Endpoint& serial, uint32_t id, bool ext, bool rtr, uint8_t length, const uint8_t (&data)[8]) {
    return queueCanMessage(serial, 'R', id, ext, rtr, length, data);
}

}
