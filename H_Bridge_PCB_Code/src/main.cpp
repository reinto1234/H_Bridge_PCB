#include <Arduino.h>
#include "webserver.h"
#include "PWM.h"

void setup() {
    Serial.begin(115200);
    Serial.println("Starting H-Bridge Inverter System...");

    // Initialize WiFi and WebServer
    HBridgeWebServer::initWiFi();
    HBridgeWebServer::initServer();

    Serial.println("System Initialized!");
}

void loop() {
    if (inverter != nullptr) {
        // Simulated control loop (replace with actual measurement)
        float vMeasured = analogRead(A0);  // Example: Read input voltage (adjust as needed)
        inverter->loop(512, vMeasured);    // 512 as a placeholder setpoint
    }

    delayMicroseconds(200);  // Small delay to avoid excessive CPU usage
}

