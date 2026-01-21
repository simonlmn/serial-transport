#include "Endpoint.h"

#if defined(ARDUINO_AVR_NANO)
#include <stddef.h>
#else
#include <cstddef>
#include <algorithm>
using std::min;
#endif

namespace serial_transport {

static const char BEGIN_CONTROL_MESSAGE = '#';
static const char BEGIN_CONTROL_RESPONSE_MESSAGE = '>';
static const char BEGIN_NORMAL_MESSAGE = '+';
static const char TERMINATOR = '\n';

static const char CONTROL_ACKNOWLEDGE = '=';
static const char CONTROL_ERROR = '!';
static const char CONTROL_TIMEOUT = 'T';
static const char CONTROL_DEBUG = 'D';

const char* describe(ErrorCode errorCode) {
    switch (errorCode) {
        case ErrorCode::RxBufferOverflow: return "Receive buffer overflow";
        case ErrorCode::InvalidMessageStart: return "Invalid message start indicator";
        case ErrorCode::InvalidMessageSize: return "Invalid message size";
        case ErrorCode::InvalidControlMessage: return "Unknown control message";
        case ErrorCode::InvalidTimeoutControl: return "Unknown timeout control operation";
        case ErrorCode::InvalidSequenceCounter: return "Invalid sequence counter";
        case ErrorCode::ResendLimitExceeded: return "Resend limit exceeded";
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
    _bufferOverflow = false;
    _lastTxQueueIndex = 0u;
    _currentTxSequenceNumber = MAX_SEQUENCE_NUMBER;
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
                sendError(ErrorCode::ResendLimitExceeded);
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

void Endpoint::send(QueuedMessage& message) {
    message.acknowledged = false;
    message.sendTime = millis();

    Serial.write(BEGIN_NORMAL_MESSAGE);
    Serial.write('A' + message.sequenceNumber);
    Serial.write(' ');
    Serial.write(message.payload, message.payloadSize);
    Serial.write(TERMINATOR);
}

void Endpoint::sendAcknowledge(uint8_t sequenceNumber) {
    Serial.write(BEGIN_CONTROL_MESSAGE);
    Serial.write(CONTROL_ACKNOWLEDGE);
    Serial.write('A' + sequenceNumber);
    Serial.write(' ');
    Serial.write(TERMINATOR);
}

void Endpoint::sendError(ErrorCode errorCode, char detail) {
    Serial.write(BEGIN_CONTROL_MESSAGE);
    Serial.write(CONTROL_ERROR);
    Serial.write(static_cast<char>(errorCode));
    Serial.write(detail);
    Serial.write(TERMINATOR);
}

void Endpoint::sendControlResponse(const char forControl, const char* fmt, ...) {
    char payload[PAYLOAD_MAX_SIZE];

    va_list args;
    va_start(args, fmt);
    size_t payloadSize = vsnprintf(payload, PAYLOAD_MAX_SIZE, fmt, args);
    va_end(args);

    if (payloadSize < 0 || payloadSize >= static_cast<int>(PAYLOAD_MAX_SIZE)) {
        return;
    }

    Serial.write(BEGIN_CONTROL_RESPONSE_MESSAGE);
    Serial.write(forControl);
    Serial.write(payload, payloadSize);
    Serial.write(TERMINATOR);
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

    // Move oldest index along if already acknowledged (e.g. as it is the case in the beginning).
    while (!txMessageQueued() && _currentUnacknowledgedTxQueueIndex != _lastTxQueueIndex) {
        acknowledgeCurrentUnacknowledgedMessage();
    }
    return _lastTxQueueIndex;
}

size_t Endpoint::incrementCurrentUnacknowledgedTxQueueIndex() {
    if (_currentUnacknowledgedTxQueueIndex == _lastTxQueueIndex) { // do not increment beyond last message
        return _currentUnacknowledgedTxQueueIndex;
    } else {
        return _currentUnacknowledgedTxQueueIndex = (_currentUnacknowledgedTxQueueIndex + 1u) % TX_QUEUE_SIZE;
    }
}

uint8_t Endpoint::incrementTxSequenceNumber() {
    return _currentTxSequenceNumber = wrapSequenceNumber(_currentTxSequenceNumber + 1u);
}

uint8_t Endpoint::incrementRxSequenceNumber() {
    return _currentRxSequenceNumber = wrapSequenceNumber(_currentRxSequenceNumber + 1u);
}

uint8_t Endpoint::wrapSequenceNumber(uint8_t sequenceNumber) {
    return (sequenceNumber > MAX_SEQUENCE_NUMBER) ? 0u : sequenceNumber;
}

void Endpoint::receive() {
    if (Serial.available() == 0) {
        return;
    }

    auto bytesRead = Serial.readBytes(_rxBuffer + _rxBufferSize, BUFFER_MAX_SIZE - _rxBufferSize);
    auto newRxBufferSize = _rxBufferSize + bytesRead;

    char* terminator = reinterpret_cast<char*>(memchr(_rxBuffer, TERMINATOR, newRxBufferSize));
    if (terminator != nullptr) {
        if (!_bufferOverflow) {
            char* messageStart = _rxBuffer;
            while (messageStart != terminator && *messageStart == ' ') { // skip leading spaces
                ++messageStart;
            }

            if (messageStart != terminator) { // non-empty message
                *terminator = '\0';
                handleRxMessage(messageStart);
            }
        }
        // Shift remaining data to the front of the buffer
        _rxBufferSize = (_rxBuffer + newRxBufferSize) - (terminator + 1);
        memmove(_rxBuffer, terminator + 1, _rxBufferSize);
        _bufferOverflow = false;
    } else if (newRxBufferSize >= BUFFER_MAX_SIZE) {
        sendError(ErrorCode::RxBufferOverflow);
        _rxBufferSize = 0u;
        _bufferOverflow = true;
    } else {
        _rxBufferSize = newRxBufferSize;
    }
}

void Endpoint::handleRxMessage(const char* rxMessage) {
    size_t rxMessageLength = strlen(rxMessage);
    switch (rxMessage[0]) {
        case BEGIN_NORMAL_MESSAGE:
            handleNormalMessage(rxMessage, rxMessageLength);
            break;
        case BEGIN_CONTROL_MESSAGE:
            handleControlMessage(rxMessage, rxMessageLength);
            break;
        case BEGIN_CONTROL_RESPONSE_MESSAGE:
            // we don't handle it, as it is used for interactive testing/debugging only for now.
            break;
        default:
            sendError(ErrorCode::InvalidMessageStart, rxMessage[0]);
            break;
    }
}

void Endpoint::handleNormalMessage(const char* rxMessage, size_t rxMessageLength) {
    if (rxMessageLength <= BEGIN_SIZE + SEQUENCE_COUNTER_SIZE) {
        return; // Ignore overly short/truncated normal messages to avoid spurious warnings
    }

    if (rxMessage[2] != ' ') {
        sendError(ErrorCode::InvalidSequenceCounter, rxMessage[2]);
        return;
    }

    uint8_t sequenceNumber = uint8_t(rxMessage[1] - 'A');
    if (sequenceNumber == _currentRxSequenceNumber) {
        sendAcknowledge(incrementRxSequenceNumber());
        _processReceived(rxMessage + BEGIN_SIZE + SEQUENCE_COUNTER_SIZE, *this);
    } else {
        sendAcknowledge(_currentRxSequenceNumber); // Send last ack again to trigger a re-send
    }
}

void Endpoint::handleControlMessage(const char* rxMessage, size_t rxMessageLength) {
    switch (rxMessage[1]) {
        case CONTROL_ACKNOWLEDGE:
            if (rxMessageLength == CONTROL_MESSAGE_HEAD_SIZE + SEQUENCE_COUNTER_SIZE) {
                handleAcknowledge(uint8_t(rxMessage[2] - 'A'));
            } else {
                sendError(ErrorCode::InvalidMessageSize, CONTROL_ACKNOWLEDGE);
            }
            break;
        case CONTROL_ERROR:
            if (rxMessageLength == CONTROL_MESSAGE_HEAD_SIZE + ERROR_SIZE) {
                _handleError(static_cast<ErrorCode>(rxMessage[2]), rxMessage[3], *this);
            } else {
                sendError(ErrorCode::InvalidMessageSize, CONTROL_ERROR);
            }
            break;
        case CONTROL_TIMEOUT:
            if (rxMessageLength == CONTROL_MESSAGE_HEAD_SIZE + 1u) {
                handleTimeoutControl(rxMessage[2]);
            } else {
                sendError(ErrorCode::InvalidMessageSize, CONTROL_TIMEOUT);
            }
            break;
        case CONTROL_DEBUG:
            if (rxMessageLength == CONTROL_MESSAGE_HEAD_SIZE) {
                sendControlResponse(CONTROL_DEBUG, "%u %u %u %u %u %u", _currentTxSequenceNumber, _currentRxSequenceNumber, _lastTxQueueIndex, _currentUnacknowledgedTxQueueIndex, _txQueue[_currentUnacknowledgedTxQueueIndex].acknowledged, _txQueue[_currentUnacknowledgedTxQueueIndex].sendTime);
            } else {
                sendError(ErrorCode::InvalidMessageSize, CONTROL_DEBUG);
            }
            break;
        default:
            sendError(ErrorCode::InvalidControlMessage, rxMessage[1]);
            break;
    }
}

void Endpoint::handleAcknowledge(uint8_t sequenceNumber) {
    QueuedMessage& currentMessage = currentUnacknowledgedMessage();
    if (!currentMessage.acknowledged && wrapSequenceNumber(currentMessage.sequenceNumber + 1u) == sequenceNumber) {
        acknowledgeCurrentUnacknowledgedMessage();
        QueuedMessage& nextMessage = currentUnacknowledgedMessage();
        if (!nextMessage.acknowledged && canSend(_currentUnacknowledgedTxQueueIndex)) {
            send(nextMessage);
        }
    }
}

void Endpoint::handleTimeoutControl(const char operation) {
    switch (operation) {
        case '+':
            _timeoutEnabled = true;
            sendControlResponse(CONTROL_TIMEOUT, "%c", operation);
            break;
        case '-':
            _timeoutEnabled = false;
            sendControlResponse(CONTROL_TIMEOUT, "%c", operation);
            break;
        default:
            sendError(ErrorCode::InvalidTimeoutControl);
            break;
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
