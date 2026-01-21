#ifndef SERIAL_TRANSPORT_ENDPOINT_H_
#define SERIAL_TRANSPORT_ENDPOINT_H_

#include <Arduino.h>

namespace serial_transport {

class Endpoint;

enum struct WarningCode : char {
    ResendLimitReached = 'R',
    RxBufferFull = 'O'
};

const char* describe(WarningCode warning);

#if defined(ARDUINO_AVR_NANO)
using ReceiveCallback = void (*)(const char* message, Endpoint& serial);
using WarningCallback = void (*)(WarningCode warning, Endpoint& serial);
#else
#include <functional>
using ReceiveCallback = std::function<void(const char* message, Endpoint& serial)>;
using WarningCallback = std::function<void(WarningCode warning, Endpoint& serial)>;
#endif

#if defined(__AVR__)
using SerialConfig = uint8_t;
#endif

/**
 * Binary framing protocol for reliable serial communication without hardware flow control.
 * 
 * Frame format:
 * [SYNC1:0x5A] [SYNC2:0xA5] [TYPE] [LENGTH] [SEQ] [PAYLOAD...] [CRC8]
 * 
 * SYNC1/SYNC2: 2 bytes, frame start marker (0x5A 0xA5)
 * TYPE: Frame type (0x01=DATA, 0x02=ACK)
 * LENGTH: Payload length (0-57 bytes max)
 * SEQ: Sequence number (0-255, wraps around)
 * PAYLOAD: Up to 57 bytes of data
 * CRC8: CCITT CRC (polynomial 0x07, initial 0x00)
 * 
 * Total frame size: 2 + 3 header + payload + 1 CRC = max 64 bytes
 * 
 * Timeout handling:
 * - Calculated automatically based on baud rate (typically 20-100ms at 115200 baud)
 * - High retry limit (200) allows automatic recovery from transient issues
 * - Only reports errors for truly unrecoverable communication failures
 * 
 * Payloads (unchanged from ASCII protocol):
 * READY\n
 * SETUP FFFFFFFF NOR|LOP|SLP|LIS\n
 * SETUP OK 9 64 8 8 8 4 1 1234567890 1 1234567890\n
 * SETUP EFFFF: FFFFFFFF NOR|LOP|SLP|LIS\n
 * CANRX FFFFFFFF 8 01 02 03 04 05 06 07 08 \n
 * CANTX FFFFFFFF 8 01 02 03 04 05 06 07 08 \n
 * CANTX OK FFFFFFFF 8 01 02 03 04 05 06 07 08 \n
 * etc.
 */
class Endpoint final {
private:
    static const unsigned long DEFAULT_BAUD_RATE = 115200u;
    static const SerialConfig DEFAULT_SERIAL_MODE = SERIAL_8E1;
    static const uint8_t DEFAULT_RESEND_LIMIT = 200u;

    static const uint8_t SYNC1 = 0x5Au;
    static const uint8_t SYNC2 = 0xA5u;
    
    static const uint8_t FRAME_TYPE_DATA = 0x01u;
    static const uint8_t FRAME_TYPE_ACK = 0x02u;

    static const size_t BUFFER_MAX_SIZE = 64u;
    static const size_t FRAME_OVERHEAD = 6u; // SYNC1 + SYNC2 + TYPE + LENGTH + SEQ + CRC8
    static const size_t FRAME_MIN_SIZE = FRAME_OVERHEAD; // Minimum valid frame size (no payload)
    static const size_t PAYLOAD_MAX_SIZE = BUFFER_MAX_SIZE - FRAME_OVERHEAD;

    struct QueuedMessage {
        char payload[PAYLOAD_MAX_SIZE] = {};
        size_t payloadSize = 0u;
        uint8_t sequenceNumber = 0u;
        unsigned long sendTime = 0u;
        uint8_t retries = 0u;
        bool acknowledged = true;
    };

    static const size_t TX_QUEUE_SIZE = 6u;
    QueuedMessage _txQueue[TX_QUEUE_SIZE] = {};
    size_t _lastTxQueueIndex = 0u;
    size_t _currentUnacknowledgedTxQueueIndex = 0u;

    uint8_t _rxBuffer[BUFFER_MAX_SIZE] = {};
    size_t _rxBufferSize = 0u;

    uint8_t _currentTxSequenceNumber = 0u;
    uint8_t _currentRxSequenceNumber = 0u;

    unsigned long _timeout = 100UL;
    uint8_t _resendLimit = DEFAULT_RESEND_LIMIT;

    ReceiveCallback _receive;
    WarningCallback _warn;

    void receive();
    uint8_t calculateCrc8(const uint8_t* data, size_t length) const;
    void handleFrame(uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t payloadLen);
    void handleAcknowledge(uint8_t sequenceNumber);

    void send();
    bool send(QueuedMessage& message);
    void sendAcknowledge(uint8_t sequenceNumber);
    void sendFrame(uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t payloadLen);
    bool canSend(size_t queueIndex) const;
    
    const QueuedMessage& currentUnacknowledgedMessage() const;
    QueuedMessage& currentUnacknowledgedMessage();
    void acknowledgeCurrentUnacknowledgedMessage();
    bool txMessageQueued() const;
    size_t nextTxQueueIndex() const;
    size_t incrementLastTxQueueIndex();
    size_t incrementCurrentUnacknowledgedTxQueueIndex();
    uint8_t incrementTxSequenceNumber();
    
public:
    Endpoint(ReceiveCallback receive, WarningCallback warn);

    void setup(unsigned long baud = DEFAULT_BAUD_RATE, SerialConfig serialMode = DEFAULT_SERIAL_MODE);
    void reset();
    void loop();
    bool queue(const char* fmt, ...);
    bool canQueue() const;
};

// Helper functions to send CAN messages from both sides

bool queueCanTxMessage(Endpoint& serial, uint32_t id, bool ext, bool rtr, uint8_t length, const uint8_t (&data)[8]);

bool queueCanRxMessage(Endpoint& serial, uint32_t id, bool ext, bool rtr, uint8_t length, const uint8_t (&data)[8]);

}

#endif
