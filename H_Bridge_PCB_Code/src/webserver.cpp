/************************************************************************
 * @file webserver.cpp
 * @brief Web server and WebSocket implementation (BIPOLAR only)
 ************************************************************************/
#include "webserver.h"
#include "PWM.h"
#include "LittleFS.h"
#include "mutexdefinitions.h"

// Define static variables
float HBridgeWebServer::_VRMS = 14.0f; // Default RMS voltage
bool  HBridgeWebServer::_isRunning = false;
AsyncWebServer   HBridgeWebServer::server(80);
WebSocketsServer HBridgeWebServer::webSocket(81);

// Broadcasts the inverter's running status to all connected clients
void HBridgeWebServer::broadcastStatus() {
    StaticJsonDocument<128> statusDoc;
    statusDoc["type"] = "status";
    statusDoc["isRunning"] = _isRunning;

    char buffer[128];
    serializeJson(statusDoc, buffer);
    webSocket.broadcastTXT(buffer);
}

void HBridgeWebServer::resetDefaults() {
    _VRMS = 14.0f;
    _isRunning = false;
}

void HBridgeWebServer::initWiFi() {
    resetDefaults();
    WiFi.softAP("HBridge_Control", "12345678");
    Serial.println("WiFi Access Point Started.");
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
}

// Initializes Web Server and WebSocket event handlers
void HBridgeWebServer::initServer() {
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed!");
        return;
    }

    // Handle favicon.ico requests to prevent 500 errors
    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(204);
    });

    // Serve static files
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/index.html", "text/html");
    });
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/style.css", "text/css");
    });
    server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/script.js", "application/javascript");
    });

    // Start the inverter 
    server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (_isRunning) {
            request->send(400, "text/plain", "Already running");
            return;
        }

        // Read VRMS parameter if provided
        if (request->hasParam("vrms")) {
            _VRMS = request->getParam("vrms")->value().toFloat();
        }

        // start inverter
        startInverter(_VRMS);
        _isRunning = true;

        broadcastStatus(); // Notify clients of the new state
        request->send(200, "text/plain", "Inverter Started (Bipolar)");
    });

    // Stop the inverter
    server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!_isRunning) {
            request->send(400, "text/plain", "Already stopped");
            return;
        }
        stopInverter();
        _isRunning = false;
        broadcastStatus(); // Notify clients of the new state
        request->send(200, "text/plain", "Inverter Stopped");
    });

    server.begin();
    Serial.println("HTTP Server Started");
    webSocket.onEvent(onWebSocketEvent);
    webSocket.begin();
    Serial.println("WebSocket Server Started");

    webSocket.enableHeartbeat(15000, 3000, 2);   // ping every 15s, timeout 3s, 2 fails -> drop
}

// Updates measurements and broadcasts via WebSocket
void HBridgeWebServer::updateMeasurements(float* input, float* output) {
    StaticJsonDocument<512> jsonDoc;

    jsonDoc["voltage"]       = input[0];
    jsonDoc["current"]       = input[1];
    jsonDoc["power"]         = input[2];
    jsonDoc["voltage_out"]   = output[0];
    jsonDoc["current_out"]   = output[1];
    jsonDoc["power_out"]     = output[2];
    jsonDoc["powerfactor"]   = output[3];
    jsonDoc["phase"]         = output[4];
    jsonDoc["imaginaryPower"]= output[5];
    jsonDoc["frequency"]     = output[6];

    char buffer[512];
    size_t len = serializeJson(jsonDoc, buffer);
    webSocket.broadcastTXT(buffer, len);
}

// Handles WebSocket events like connect, disconnect, and messages
void HBridgeWebServer::onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Connected from %s\n", num, ip.toString().c_str());

            // Send initial status to the newly connected client
            StaticJsonDocument<128> statusDoc;
            statusDoc["type"] = "status";
            statusDoc["isRunning"] = _isRunning;
            char buffer[128];
            size_t len = serializeJson(statusDoc, buffer);
            webSocket.sendTXT(num, buffer, len);
            break;
        }
        case WStype_TEXT:
            // Not used in this application
            break;
    }
}
