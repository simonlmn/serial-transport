#ifndef SERIAL_TRANSPORT_ENDPOINT_H_
#define SERIAL_TRANSPORT_ENDPOINT_H_

#include <Arduino.h>

#if !defined(ARDUINO_AVR_NANO)
#include <functional>
#endif

using SerialType = decltype(Serial);

#if !defined(ARDUINO_ARCH_ESP8266) && !defined(ARDUINO_ARCH_ESP32)
using SerialConfig = uint8_t;
#endif

namespace serial_transport {

class Endpoint;

enum struct ConnectionState {
    UNSYNCED = 0,   // Need to send SYN
    SYNCING = 1,    // SYN sent, waiting for peer's ACK  
    SYNCED = 2      // SYN + ACK complete, normal DATA/ACK operation
};

const char* describe(ConnectionState state);

#if defined(ARDUINO_AVR_NANO)
using ReceiveCallback = void (*)(const char* message, Endpoint& serial);
using StateCallback = void (*)(ConnectionState state, Endpoint& serial);
using FrameCallback = void (*)(char direction, uint8_t type, uint8_t sequenceNumber, const uint8_t* payload, uint8_t payloadLen);
#else
using ReceiveCallback = std::function<void(const char* message, Endpoint& serial)>;
using StateCallback = std::function<void(ConnectionState state, Endpoint& serial)>;
using FrameCallback = std::function<void(char direction, uint8_t type, uint8_t sequenceNumber, const uint8_t* payload, uint8_t payloadLen)>;
#endif

/**
 * Binary framing protocol for reliable serial communication.
 * 
 * Frame format:
 * [SYNC1:0x5A] [SYNC2:0xA5] [TYPE] [LENGTH] [SEQ] [PAYLOAD...] [CRC8]
 * 
 * SYNC1/SYNC2: 2 bytes, frame start marker (0x5A 0xA5)
 * TYPE: Frame type (0x01=DATA, 0x02=ACK, 0x03=SYN)
 * LENGTH: Payload length (0-57 bytes max)
 * SEQ: Sequence number (0-255, wraps around)
 * PAYLOAD: Up to 57 bytes of data
 * CRC8: CCITT CRC (polynomial 0x07, initial 0x00)
 * 
 * Total frame size: 2 + 3 header + payload + 1 CRC = max 64 bytes
 * 
 * Synchronization protocol (TCP-like):
 * - On startup or after error, endpoints send SYN frames to synchronize their sequence number
 * - SYN frame SEQ field contains the sequence number for the NEXT DATA frame
 * - On receiving SYN, endpoint updates expected RX sequence and responds with ACK
 * - Only DATA frames are sent after successful synchronization
  * 
 * Connection states:
 * - UNSYNCED: No sync established, send SYN frames periodically
 * - SYNCING: SYN sent, waiting for peer's ACK response
 * - SYNCED: Both sides synchronized, normal DATA/ACK operation
 * 
 * Timeout handling:
 * - Calculated automatically based on baud rate (typically 20-100ms at 115200 baud)
 * - High retry limit (200) allows automatic recovery from transient issues
 * - ResendLimitReached warning triggers resynchronization attempt
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
    static const unsigned long DEFAULT_BAUD_RATE = 115200ul;
    static const SerialConfig DEFAULT_SERIAL_MODE = SERIAL_8E1;
    static const uint8_t DEFAULT_RESEND_LIMIT = 200u;
    static const unsigned long SYN_RETRY_INTERVAL = 150ul;

    static constexpr uint8_t SYNC1 = 0x5Au;
    static constexpr uint8_t SYNC2 = 0xA5u;
    
    static constexpr uint8_t FRAME_TYPE_DATA = 0x01u;
    static constexpr uint8_t FRAME_TYPE_ACK = 0x02u;
    static constexpr uint8_t FRAME_TYPE_SYN = 0x03u;
    static constexpr uint8_t FRAME_TYPE_DBG = 0xFFu;

    static constexpr uint8_t BUFFER_MAX_SIZE = 64u;
    
    static constexpr uint8_t FRAME_SYNC_SIZE = 2u; // SYNC1 + SYNC2
    static constexpr uint8_t FRAME_META_SIZE = 3u; // TYPE + LENGTH + SEQ
    static constexpr uint8_t FRAME_HEADER_SIZE = FRAME_SYNC_SIZE + FRAME_META_SIZE;
    static constexpr uint8_t FRAME_CRC_SIZE = 1u; // CRC8
    static constexpr uint8_t FRAME_OVERHEAD = FRAME_HEADER_SIZE + FRAME_CRC_SIZE;
    static constexpr uint8_t FRAME_MIN_SIZE = FRAME_OVERHEAD; // Minimum valid frame size (no payload)
    static constexpr uint8_t PAYLOAD_MAX_SIZE = BUFFER_MAX_SIZE - FRAME_OVERHEAD;

#if defined(ARDUINO_AVR_NANO)
    static const uint8_t TX_QUEUE_SIZE = 4u;
#else
    static const uint8_t TX_QUEUE_SIZE = 8u;
#endif

    struct QueuedMessage {
        char payload[PAYLOAD_MAX_SIZE] = {};
        uint8_t payloadSize = 0u;
        uint8_t sequenceNumber = 0u;
        uint8_t retries = 0u;
        uint8_t acknowledged = 1u;
        unsigned long sendTime = 0u;
    };

    ConnectionState _connectionState = ConnectionState::UNSYNCED;
    unsigned long _lastSynSendTime = 0u;
    unsigned long _lastDiagnosticsTime = 0u;
    
    unsigned long _timeout = 100UL;
    uint8_t _resendLimit = DEFAULT_RESEND_LIMIT;

    uint8_t _rxBuffer[BUFFER_MAX_SIZE] = {};
    uint8_t _rxBufferSize = 0u;
    uint8_t _lastRxSequenceNumber = 0u;

    uint8_t _txFrameBuffer[BUFFER_MAX_SIZE] = {};

    QueuedMessage _txQueue[TX_QUEUE_SIZE] = {};
    uint8_t _lastTxQueueIndex = 0u;
    uint8_t _firstTxQueueIndex = 0u;

    ReceiveCallback _receive;
    StateCallback _notifyState;
    FrameCallback _frame;

    SerialType& _serial;

    bool _debugEnabled = false;

    void setConnectionState(ConnectionState state);

    void receive();
    uint8_t calculateCrc8(const uint8_t* data, size_t length) const;
    void handleFrame(uint8_t type, uint8_t sequenceNumber, const uint8_t* payload, uint8_t payloadLen);
    void handleSyn(uint8_t sequenceNumber);
    void handleAcknowledge(uint8_t sequenceNumber);
    void incrementFirstTxQueueIndex();

    void sync();

    void send();
    void sendSyn();
    void sendAcknowledge(uint8_t sequenceNumber);
    bool send(QueuedMessage &message);
    bool canWrite(const serial_transport::Endpoint::QueuedMessage& message);
    void sendFrame(uint8_t type, uint8_t sequenceNumber, const uint8_t* payload, uint8_t payloadLen);
    
    bool isFirstInQueue(uint8_t queueIndex) const;
    const QueuedMessage& firstTxQueueMessage() const;
    QueuedMessage& firstTxQueueMessage();
    uint8_t firstTxSequenceNumber() const;
    uint8_t nextTxQueueIndex() const;
    uint8_t incrementLastTxQueueIndex();
    uint8_t lastTxSequenceNumber() const;

    uint8_t nextTxSequenceNumberToSend() const;
    
public:
    Endpoint(SerialType& serial, ReceiveCallback receive = nullptr, StateCallback notifyState = nullptr, FrameCallback frame = nullptr);

    void setup(unsigned long baud = DEFAULT_BAUD_RATE, SerialConfig serialMode = DEFAULT_SERIAL_MODE);
    void reset();
    void loop();
    bool queue(const char* fmt, ...);
    bool canQueue() const;

    bool hasQueuedTxMessage() const;
    uint8_t numberOfQueuedMessages() const;
    
    ConnectionState connectionState() const { return _connectionState; }

    void enableDebug(bool enabled);
    void sendDebug(const char* fmt, ...);

    void setReceiveCallback(ReceiveCallback callback) { _receive = callback; }
    void setStateCallback(StateCallback callback) { _notifyState = callback; }
    void setFrameCallback(FrameCallback callback) { _frame = callback; }
};

// Helper functions to send CAN messages from both sides

bool queueCanTxMessage(Endpoint& serial, uint32_t id, bool ext, bool rtr, uint8_t length, const uint8_t (&data)[8]);

bool queueCanRxMessage(Endpoint& serial, uint32_t id, bool ext, bool rtr, uint8_t length, const uint8_t (&data)[8]);

}

#endif
