/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

#include <Arduino.h>
#include "webserver.h"

void setup() {
    // Start serial communication for debugging
    Serial.begin(115200);
    delay(1000); // Give time for the serial monitor to start

    // Initialize WiFi
    HBridgeWebServer::initWiFi();

    // Initialize the server
    HBridgeWebServer::initServer();
}

void loop() {
    // Nothing to do here, everything is handled by the server
}