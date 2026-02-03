#include <yatest.h>
#include <serial_transport/Endpoint.h>

using namespace yatest;
using namespace yatest::expect;

namespace {

// Helper to create a pair of endpoints for testing
struct EndpointPair {
    RingBuffer serial1to2Buffer;
    RingBuffer serial2to1Buffer;
    SerialMock serial1;
    SerialMock serial2;
    serial_transport::Endpoint endpoint1;
    serial_transport::Endpoint endpoint2;

    EndpointPair() 
        : serial1(serial1to2Buffer, serial2to1Buffer)
        , serial2(serial2to1Buffer, serial1to2Buffer)
        , endpoint1(
            serial_transport::EndpointRole::SERVER,
            serial1,
            [](const uint8_t*, uint8_t, serial_transport::Endpoint&) {},
            [](serial_transport::ConnectionState, serial_transport::Endpoint&) {},
            [](bool, uint8_t, uint8_t, const uint8_t*, uint8_t) {}
        )
        , endpoint2(
            serial_transport::EndpointRole::CLIENT,
            serial2,
            [](const uint8_t*, uint8_t, serial_transport::Endpoint&) {},
            [](serial_transport::ConnectionState, serial_transport::Endpoint&) {},
            [](bool, uint8_t, uint8_t, const uint8_t*, uint8_t) {}
        )
    {
        endpoint1.setup();
        endpoint2.setup();
    }

    void loop() {
        endpoint1.loop();
        endpoint2.loop();
        _test_time += 100;
    }

    void loopUntilConnected() {
        // Usually takes 3 loop iterations to establish connection
        for (int i = 0; i < 10 && 
            (endpoint1.connectionState() != serial_transport::ConnectionState::CONNECTED ||
             endpoint2.connectionState() != serial_transport::ConnectionState::CONNECTED); ++i) {
            loop();
        }
    }
};

static const TestSuite& EndpointTests =
    suite("Endpoint")
        .tests("initial state after setup", []() {
            EndpointPair pair;
            
            equals(pair.endpoint1.connectionState(), serial_transport::ConnectionState::CLOSED);
            equals(pair.endpoint2.connectionState(), serial_transport::ConnectionState::CLOSED);
            isFalse(pair.endpoint1.hasQueuedTxMessage());
            isFalse(pair.endpoint2.hasQueuedTxMessage());
            equals(pair.endpoint1.numberOfQueuedMessages(), 0);
            equals(pair.endpoint2.numberOfQueuedMessages(), 0);
        })
        
        .tests("connection establishment sequence", []() {
            EndpointPair pair;
            
            // Initial state
            equals(pair.endpoint1.connectionState(), serial_transport::ConnectionState::CLOSED);
            equals(pair.endpoint2.connectionState(), serial_transport::ConnectionState::CLOSED);
            
            // After first loop - server listens, client waits
            pair.loop();
            equals(pair.endpoint1.connectionState(), serial_transport::ConnectionState::LISTENING);
            equals(pair.endpoint2.connectionState(), serial_transport::ConnectionState::WAITING);
            
            // After second loop - server waits, client connects
            pair.loop();
            equals(pair.endpoint1.connectionState(), serial_transport::ConnectionState::WAITING);
            equals(pair.endpoint2.connectionState(), serial_transport::ConnectionState::CONNECTED);
            
            // After third loop - both connected
            pair.loop();
            equals(pair.endpoint1.connectionState(), serial_transport::ConnectionState::CONNECTED);
            equals(pair.endpoint2.connectionState(), serial_transport::ConnectionState::CONNECTED);
        })
        
        .tests("queue single message before connection", []() {
            EndpointPair pair;
            
            pair.endpoint1.queue("TEST M1");
            isTrue(pair.endpoint1.hasQueuedTxMessage());
            equals(pair.endpoint1.numberOfQueuedMessages(), 1);
        })
        
        .tests("send message after connection established", []() {
            EndpointPair pair;
            pair.endpoint1.queue("TEST M1");
            
            // Establish connection (takes 3 loops)
            pair.loopUntilConnected();
            
            // After connection, message should be sent
            pair.loop();
            isFalse(pair.endpoint1.hasQueuedTxMessage());
            equals(pair.endpoint1.numberOfQueuedMessages(), 0);
        })
        
        .tests("queue and send message from other endpoint", []() {
            EndpointPair pair;
            pair.loopUntilConnected();
            
            pair.endpoint2.queue("TEST M2");
            isTrue(pair.endpoint2.hasQueuedTxMessage());
            equals(pair.endpoint2.numberOfQueuedMessages(), 1);
            
            // Send and receive
            pair.loop();
            pair.loop();
            
            isFalse(pair.endpoint2.hasQueuedTxMessage());
            equals(pair.endpoint2.numberOfQueuedMessages(), 0);
        })
        
        .tests("queue multiple messages", []() {
            EndpointPair pair;
            pair.loopUntilConnected();
            
            pair.endpoint1.queue("TEST M1-1");
            pair.endpoint1.queue("TEST M1-2");
            pair.endpoint1.queue("TEST M1-3");
            
            isTrue(pair.endpoint1.hasQueuedTxMessage());
            equals(pair.endpoint1.numberOfQueuedMessages(), 3);
        })
        
        .tests("send multiple queued messages", []() {
            EndpointPair pair;
            pair.loopUntilConnected();
            
            pair.endpoint1.queue("TEST M1-1");
            pair.endpoint1.queue("TEST M1-2");
            pair.endpoint1.queue("TEST M1-3");
            
            // Send all messages (need multiple loops for acknowledgments)
            for (int i = 0; i < 10; ++i) {
                pair.loop();
            }
            
            isFalse(pair.endpoint1.hasQueuedTxMessage());
            equals(pair.endpoint1.numberOfQueuedMessages(), 0);
        })
        
        .tests("bidirectional message exchange", []() {
            EndpointPair pair;
            pair.loopUntilConnected();
            
            // Exchange messages back and forth
            for (int i = 0; i < 10; ++i) {
                pair.endpoint1.queue("TEST E1");
                pair.endpoint2.queue("TEST E2");
                pair.loop();
                pair.loop();
            }
            
            equals(pair.endpoint1.connectionState(), serial_transport::ConnectionState::CONNECTED);
            equals(pair.endpoint2.connectionState(), serial_transport::ConnectionState::CONNECTED);
            isFalse(pair.endpoint1.hasQueuedTxMessage());
            isFalse(pair.endpoint2.hasQueuedTxMessage());
        })
        
        .tests("sequence number rollover", []() {
            EndpointPair pair;
            pair.loopUntilConnected();
            
            // Send 255 messages to test sequence number rollover
            for (int i = 0; i < 255; ++i) {
                pair.endpoint1.queue("TEST E1");
                pair.endpoint2.queue("TEST E2");
                pair.loop();
                pair.loop();
            }
            
            // Connection should remain stable after rollover
            equals(pair.endpoint1.connectionState(), serial_transport::ConnectionState::CONNECTED);
            equals(pair.endpoint2.connectionState(), serial_transport::ConnectionState::CONNECTED);
            isFalse(pair.endpoint1.hasQueuedTxMessage());
            isFalse(pair.endpoint2.hasQueuedTxMessage());
        })
        
        .tests("endpoint reset closes connection", []() {
            EndpointPair pair;
            pair.loopUntilConnected();
            
            equals(pair.endpoint1.connectionState(), serial_transport::ConnectionState::CONNECTED);
            equals(pair.endpoint2.connectionState(), serial_transport::ConnectionState::CONNECTED);
            
            pair.endpoint1.reset();
            
            equals(pair.endpoint1.connectionState(), serial_transport::ConnectionState::CLOSED);
            equals(pair.endpoint2.connectionState(), serial_transport::ConnectionState::CONNECTED);
        })
        
        .tests("message queued before reset is preserved", []() {
            EndpointPair pair;
            pair.loopUntilConnected();
            
            pair.endpoint1.reset();
            pair.endpoint1.queue("TEST MESSAGE AFTER RESET");
            
            isTrue(pair.endpoint1.hasQueuedTxMessage());
            equals(pair.endpoint1.numberOfQueuedMessages(), 1);
        })
        
        .tests("reconnection after reset", []() {
            EndpointPair pair;
            pair.loopUntilConnected();
            
            // Reset endpoint1
            pair.endpoint1.reset();
            pair.endpoint1.queue("TEST MESSAGE AFTER RESET");
            
            // After first loop, endpoint1 starts listening
            pair.loop();
            equals(pair.endpoint1.connectionState(), serial_transport::ConnectionState::LISTENING);
            equals(pair.endpoint2.connectionState(), serial_transport::ConnectionState::CONNECTED);
            
            // Endpoint2 still thinks it's connected
            pair.loop();
            equals(pair.endpoint1.connectionState(), serial_transport::ConnectionState::LISTENING);
            equals(pair.endpoint2.connectionState(), serial_transport::ConnectionState::CONNECTED);
            
            // Trigger reset on endpoint2 by sending a message
            pair.endpoint2.queue("TEST MESSAGE TO TRIGGER RST");
            pair.loop();
            
            // Endpoint2 realizes connection is broken and resets
            equals(pair.endpoint1.connectionState(), serial_transport::ConnectionState::LISTENING);
            equals(pair.endpoint2.connectionState(), serial_transport::ConnectionState::WAITING);
            isFalse(pair.endpoint2.hasQueuedTxMessage());
        })
        
        .tests("full reconnection sequence after reset", []() {
            EndpointPair pair;
            pair.loopUntilConnected();
            
            // Reset and queue message
            pair.endpoint1.reset();
            pair.endpoint1.queue("TEST MESSAGE AFTER RESET");
            
            // Trigger reconnection
            pair.loop();
            pair.endpoint2.queue("TEST MESSAGE TO TRIGGER RST");
            pair.loop();
            
            // Now reconnect
            pair.loop();
            equals(pair.endpoint1.connectionState(), serial_transport::ConnectionState::WAITING);
            equals(pair.endpoint2.connectionState(), serial_transport::ConnectionState::CONNECTED);
            
            pair.loop();
            equals(pair.endpoint1.connectionState(), serial_transport::ConnectionState::CONNECTED);
            equals(pair.endpoint2.connectionState(), serial_transport::ConnectionState::CONNECTED);
            
            // Message should eventually be sent
            pair.loop();
            isFalse(pair.endpoint1.hasQueuedTxMessage());
        });

} // anonymous namespace
