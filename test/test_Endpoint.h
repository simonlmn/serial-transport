#include "Arduino.h"

#include "../src/serial_transport/Endpoint.h"

void testEndpoint() {
    RingBuffer serial1to2Buffer;
    RingBuffer serial2to1Buffer;

    SerialMock serial1 { serial1to2Buffer, serial2to1Buffer };

    serial_transport::Endpoint endpoint1 {
        serial_transport::EndpointRole::SERVER,
        serial1,
        [] (const uint8_t* payload, uint8_t payloadLen, serial_transport::Endpoint& serial) {
            printf("Endpoint 1 received: %s\n", payload);
        },
        [] (serial_transport::ConnectionState state, serial_transport::Endpoint& serial) {
            printf("Endpoint 1 state changed: %d\n", static_cast<int>(state));
        },
        [] (bool tx, uint8_t type, uint8_t sequenceNumber, const uint8_t* payload, uint8_t payloadLen) {
            printf("Endpoint 1 frame %s: type=%02X seq=%02X len=%u\n", tx ? "TX" : "RX", type, sequenceNumber, payloadLen);
        }        
    };

    SerialMock serial2 { serial2to1Buffer, serial1to2Buffer };
    serial_transport::Endpoint endpoint2 {
        serial_transport::EndpointRole::CLIENT,
        serial2,
        [] (const uint8_t* payload, uint8_t payloadLen, serial_transport::Endpoint& serial) {
            printf("Endpoint 2 received: %s\n", payload);
        },
        [] (serial_transport::ConnectionState state, serial_transport::Endpoint& serial) {
            printf("Endpoint 2 state changed: %d\n", static_cast<int>(state));
        },
        [] (bool tx, uint8_t type, uint8_t sequenceNumber, const uint8_t* payload, uint8_t payloadLen) {
            printf("Endpoint 2 frame %s: type=%02X seq=%02X len=%u\n", tx ? "TX" : "RX", type, sequenceNumber, payloadLen);
        }        
    };

    auto loop =[&] () { endpoint1.loop(); endpoint2.loop(); _test_time += 100; };
    
    endpoint1.setup(); endpoint2.setup();
    assert(endpoint1.connectionState() == serial_transport::ConnectionState::CLOSED);
    assert(endpoint2.connectionState() == serial_transport::ConnectionState::CLOSED);

    endpoint1.queue("TEST M1");
    assert(endpoint1.hasQueuedTxMessage());
    assert(endpoint1.numberOfQueuedMessages() == 1);

    loop();
    assert(endpoint1.connectionState() == serial_transport::ConnectionState::LISTENING);
    assert(endpoint2.connectionState() == serial_transport::ConnectionState::WAITING);

    loop();
    assert(endpoint1.connectionState() == serial_transport::ConnectionState::WAITING);
    assert(endpoint2.connectionState() == serial_transport::ConnectionState::CONNECTED);

    loop();
    assert(endpoint1.connectionState() == serial_transport::ConnectionState::CONNECTED);
    assert(endpoint2.connectionState() == serial_transport::ConnectionState::CONNECTED);

    loop();
    assert(!endpoint1.hasQueuedTxMessage());

    endpoint2.queue("TEST M2");
    assert(endpoint2.hasQueuedTxMessage());
    assert(endpoint2.numberOfQueuedMessages() == 1);
    loop();
    loop();

    endpoint1.queue("TEST M1-2");
    endpoint1.queue("TEST M1-3");
    endpoint1.queue("TEST M1-4");
    assert(endpoint1.hasQueuedTxMessage());
    assert(endpoint1.numberOfQueuedMessages() == 3);
    loop();
    loop();
    loop();
    loop();
    loop();
    loop();
    assert(!endpoint1.hasQueuedTxMessage());

    for (int i = 0; i < 255; ++i) {
        endpoint1.queue("TEST E1 %d", i);
        endpoint2.queue("TEST E2 %d", i);
        loop();
        loop();
    }

    assert(endpoint1.connectionState() == serial_transport::ConnectionState::CONNECTED);
    assert(endpoint2.connectionState() == serial_transport::ConnectionState::CONNECTED);
    assert(!endpoint1.hasQueuedTxMessage());
    assert(!endpoint2.hasQueuedTxMessage());

    endpoint1.reset();
    endpoint1.queue("TEST MESSAGE AFTER RESET");
    endpoint1.hasQueuedTxMessage();
    assert(endpoint1.numberOfQueuedMessages() == 1);
    assert(endpoint1.connectionState() == serial_transport::ConnectionState::CLOSED);
    assert(endpoint2.connectionState() == serial_transport::ConnectionState::CONNECTED);
    loop();
    assert(endpoint1.connectionState() == serial_transport::ConnectionState::LISTENING);
    assert(endpoint2.connectionState() == serial_transport::ConnectionState::CONNECTED);
    loop();
    assert(endpoint1.connectionState() == serial_transport::ConnectionState::LISTENING);
    assert(endpoint2.connectionState() == serial_transport::ConnectionState::CONNECTED);
    endpoint2.queue("TEST MESSAGE TO TRIGGER RST");
    loop();
    assert(endpoint1.connectionState() == serial_transport::ConnectionState::LISTENING);
    assert(endpoint2.connectionState() == serial_transport::ConnectionState::WAITING);
    assert(!endpoint2.hasQueuedTxMessage());
    loop();
    assert(endpoint1.connectionState() == serial_transport::ConnectionState::WAITING);
    assert(endpoint2.connectionState() == serial_transport::ConnectionState::CONNECTED);
    loop();
    assert(endpoint1.connectionState() == serial_transport::ConnectionState::CONNECTED);
    assert(endpoint2.connectionState() == serial_transport::ConnectionState::CONNECTED);
    loop();
    assert(!endpoint1.hasQueuedTxMessage());
}