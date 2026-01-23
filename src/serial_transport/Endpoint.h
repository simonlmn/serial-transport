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

enum struct EndpointRole : uint8_t {
    CLIENT = 0,
    SERVER = 1,
};

enum struct ConnectionState : uint8_t {
    CLOSED     = 0, // Endpoint is closed
    LISTENING  = 1, // Listening for incoming connection (SYN)
    WAITING    = 2, // SYN / SYNACK sent, waiting for SYNACK / ACK
    CONNECTED  = 3,  // SYN + SYNACK + ACK complete, normal DATA/ACK operation
};

const char* describe(ConnectionState state);

#if defined(ARDUINO_AVR_NANO)
using ReceiveCallback = void (*)(const uint8_t* payload, uint8_t payloadLen, Endpoint& serial);
using StateCallback = void (*)(ConnectionState state, Endpoint& serial);
using FrameCallback = void (*)(char direction, uint8_t type, uint8_t sequenceNumber, const uint8_t* payload, uint8_t payloadLen);
#else
using ReceiveCallback = std::function<void(const uint8_t* payload, uint8_t payloadLen, Endpoint& serial)>;
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
 * TYPE: Frame type (0x01=DATA, 0x02=ACK, 0x03=SYN, 0x04=SYNACK, 0xFF=DEBUG)
 * LENGTH: Payload length (0-57 bytes max)
 * SEQ: Sequence number (0-255, wraps around)
 * PAYLOAD: Up to 57 bytes of data
 * CRC8: CCITT CRC (polynomial 0x07, initial 0x00)
 * 
 * Total frame size: 2 + 3 header + payload + 1 CRC = max 64 bytes
 * 
 * Synchronization protocol (TCP-like):
 * - Endpoints can operate in CLIENT or SERVER role
 * - On startup/after reset, CLIENT sends SYN to SERVER
 * - SERVER responds with SYNACK
 * - CLIENT responds with ACK to complete handshake
 * - SYN(ACK) frame SEQ field contains the sequence number for the NEXT DATA frame
 * - On receiving SYN(ACK), endpoint updates expected RX sequence and responds with SYNACK/ACK
 * - Only DATA frames are sent after successful synchronization
 * - If at any time a frame is received in an invalid state, a RESET (RST) frame is sent to restart the connection.
 * 
 * Connection states:
 * - CLOSED: Endpoint is closed
 * - LISTENING: Listening for incoming connection (SYN)
 * - CONNECTING: SYN / SYNACK sent, waiting for SYNACK / ACK
 * - CONNECTED: SYN + SYNACK + ACK complete, normal DATA/ACK operation
 * 
 * Timeout handling:
 * - Calculated automatically based on baud rate (typically 20-100ms at 115200 baud)
 * - High retry limit (250) allows automatic recovery from transient issues
 * - ResendLimitReached warning triggers resynchronization attempt
 */
class Endpoint final {
private:
    static const unsigned long DEFAULT_BAUD_RATE = 115200ul;
    static const SerialConfig DEFAULT_SERIAL_MODE = SERIAL_8N1;
    static const uint8_t DEFAULT_RESEND_LIMIT = 250u;
    static const unsigned long SYN_RETRY_INTERVAL = 150ul;

    static constexpr uint8_t SYNC1 = 0x5Au;
    static constexpr uint8_t SYNC2 = 0xA5u;
    
    static constexpr uint8_t FRAME_TYPE_DATA = 0x01u;
    static constexpr uint8_t FRAME_TYPE_ACK = 0x02u;
    static constexpr uint8_t FRAME_TYPE_SYN = 0x03u;
    static constexpr uint8_t FRAME_TYPE_SYNACK = 0x04u;
    static constexpr uint8_t FRAME_TYPE_RST = 0x05u;
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
        uint8_t payload[PAYLOAD_MAX_SIZE] = {};
        uint8_t payloadSize = 0u;
        uint8_t sequenceNumber = 0u;
        uint8_t retries = 0u;
        uint8_t acknowledged = 1u;
        unsigned long sendTime = 0u;
    };

    ConnectionState _connectionState = ConnectionState::CLOSED;
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

    bool _diagnosticsEnabled = false;

    EndpointRole _role = EndpointRole::CLIENT;

    void setConnectionState(ConnectionState state);

    void receive();
    uint8_t calculateCrc8(const uint8_t* data, size_t length) const;
    void handleFrame(uint8_t type, uint8_t sequenceNumber, const uint8_t* payload, uint8_t payloadLen);
    void handleData(uint8_t sequenceNumber, const uint8_t* payload, uint8_t payloadLen);
    void handleSyn(uint8_t sequenceNumber);
    void handleSynAcknowledge(uint8_t sequenceNumber, uint8_t acknowledgedSeqNumber);
    void handleAcknowledge(uint8_t sequenceNumber);
    void handleReset();
    void incrementFirstTxQueueIndex();

    void trySend();
    void sendSyn();
    void sendSynAcknowledge();
    void sendAcknowledge(uint8_t sequenceNumber);
    void sendReset();
    bool sendData(QueuedMessage &message);
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

    void sendDiagnostics();
    
public:
    Endpoint(EndpointRole role, SerialType& serial, ReceiveCallback receive = nullptr, StateCallback notifyState = nullptr, FrameCallback frame = nullptr);

    void setup(unsigned long baud = DEFAULT_BAUD_RATE, SerialConfig serialMode = DEFAULT_SERIAL_MODE);
    void reset();
    ConnectionState connectionState() const { return _connectionState; }
    void loop();
    bool canQueue() const;
    bool queue(const char* fmt, ...);
    bool queue(const uint8_t* data, uint8_t length);
    
    bool hasQueuedTxMessage() const;
    uint8_t numberOfQueuedMessages() const;
    
    void sendDebug(const char* fmt, ...);
    void enableDiagnostics(bool enabled);
    
    void setReceiveCallback(ReceiveCallback callback) { _receive = callback; }
    void setStateCallback(StateCallback callback) { _notifyState = callback; }
    void setFrameCallback(FrameCallback callback) { _frame = callback; }
};

// Helper functions to send CAN messages from both sides

bool queueCanTxMessage(Endpoint& serial, uint32_t id, bool ext, bool rtr, uint8_t length, const uint8_t (&data)[8]);

bool queueCanRxMessage(Endpoint& serial, uint32_t id, bool ext, bool rtr, uint8_t length, const uint8_t (&data)[8]);

}

#endif
