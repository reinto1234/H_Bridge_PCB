/************************************************************************
 * @file webserver.cpp
 * @brief Corrected web server and WebSocket implementation
 ************************************************************************/
#include "webserver.h"
#include "PWM.h"
#include "LittleFS.h"
#include "mutexdefinitions.h"

// Define static variables
uint32_t HBridgeWebServer::_switchingFrequency = 20000;
String HBridgeWebServer::_modulationType = "BIPOLAR";
bool HBridgeWebServer::_isRunning = false;
AsyncWebServer HBridgeWebServer::server(80);
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

// Converts modulation string to enum, now case-insensitive
ModulationType HBridgeWebServer::getModulationType() {
    if (_modulationType.equalsIgnoreCase("BIPOLAR")) {
        return BIPOLAR;
    } else {
        return UNIPOLAR;
    }
}

void HBridgeWebServer::resetDefaults() {
    _switchingFrequency = 20000;
    _modulationType = "BIPOLAR";
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

    // --- FIX: Handle favicon.ico requests to prevent 500 errors ---
    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(204); // Send "204 No Content" is a standard way to handle this
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
     server.on("/bipolar.png", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/bipolar.png", "image/png");
    });
    server.on("/unipolar.png", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/unipolar.png", "image/png");
    });

    // Start the inverter
    server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (_isRunning) {
            request->send(400, "text/plain", "Already running");
            return;
        }

        // Read and validate parameters
        if (request->hasParam("freq")) {
            _switchingFrequency = request->getParam("freq")->value().toInt();
        }
        if (request->hasParam("modulation")) {
            _modulationType = request->getParam("modulation")->value();
        }

        ModulationType modType = getModulationType();
        
        // Assumes you have fixed PWM.cpp to be stable
        startInverter(0.1, 0.01, 0, 1023, modType, _switchingFrequency);
        _isRunning = true;
        
        broadcastStatus(); // Notify clients of the new state
        request->send(200, "text/plain", "Inverter Started");
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
}

// Updates measurements and broadcasts via WebSocket
void HBridgeWebServer::updateMeasurements(float* input, float* output) {
    // --- FIX: Increased JSON document size to prevent data corruption ---
    StaticJsonDocument<512> jsonDoc;

    jsonDoc["voltage"] = input[0];
    jsonDoc["current"] = input[1];
    jsonDoc["power"] = input[2];
    jsonDoc["voltage_out"] = output[0];
    jsonDoc["current_out"] = output[1];
    jsonDoc["power_out"] = output[2];
    jsonDoc["powerfactor"] = output[3];
    jsonDoc["phase"] = output[4];
    jsonDoc["imaginaryPower"] = output[5];
    jsonDoc["frequency"] = output[6];
    
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