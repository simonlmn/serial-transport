#include <Arduino.h>

#include <serial_transport.h>

void receive(const char* message, serial_transport::Endpoint& serial) {
    serial.queue("ECHO %s", message);
}

void warn(serial_transport::WarningCode warning, serial_transport::Endpoint& serial) {
    // Ignore warnings for this simple echo example
}

serial_transport::Endpoint serial {&receive, &warn};

void setup() {
  serial.setup();
  serial.queue("HELLO!");
}

void loop() {
  serial.loop();
}
