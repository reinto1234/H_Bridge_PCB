#include "webserver.h"
#include "PWM.h"
#include "LittleFS.h"
#include "mutexdefinitions.h"
#include "safety.h"
#include "Input_meas.h"   // for INA228 clear

float HBridgeWebServer::_VRMS = 14.0f;
bool  HBridgeWebServer::_isRunning = false;
AsyncWebServer   HBridgeWebServer::server(80);
WebSocketsServer HBridgeWebServer::webSocket(81);

void HBridgeWebServer::broadcastStatus() {
    StaticJsonDocument<128> doc;
    doc["type"] = "status";
    doc["isRunning"] = _isRunning;
    char buf[128];
    serializeJson(doc, buf);
    webSocket.broadcastTXT(buf);
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

void HBridgeWebServer::initServer() {
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed!");
        return;
    }

    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send(204);
    });
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send(LittleFS, "/index.html", "text/html");
    });
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send(LittleFS, "/style.css", "text/css");
    });
    server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send(LittleFS, "/script.js", "application/javascript");
    });

    // Start
    server.on("/start", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (_isRunning) {
            req->send(400, "text/plain", "Already running");
            return;
        }
        if (req->hasParam("vrms"))
            _VRMS = req->getParam("vrms")->value().toFloat();
        startInverter(_VRMS);
        _isRunning = true;
        broadcastStatus();
        req->send(200, "text/plain", "Inverter Started");
    });

    // Stop
    server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!_isRunning) {
            req->send(400, "text/plain", "Already stopped");
            return;
        }
        stopInverter();
        _isRunning = false;
        broadcastStatus();
        req->send(200, "text/plain", "Inverter Stopped");
    });

    // Acknowledge Trip
    server.on("/ack-trip", HTTP_POST, [](AsyncWebServerRequest *req) {
        pinMode(ESTOP_OUTPUT_PIN, OUTPUT);
        digitalWrite(ESTOP_OUTPUT_PIN, LOW);  // LED off
        g_emergency_stop = false;

        InputMeasurement::clearflags();

        req->send(200, "text/plain", "Trip acknowledged");
    });

    server.begin();
    webSocket.onEvent(onWebSocketEvent);
    webSocket.begin();
    webSocket.enableHeartbeat(15000, 3000, 2);
    Serial.println("HTTP & WebSocket Server Started");
}

void HBridgeWebServer::updateMeasurements(float* in, float* out) {
    StaticJsonDocument<512> doc;
    doc["voltage"] = in[0];
    doc["current"] = in[1];
    doc["power"] = in[2];
    doc["voltage_out"] = out[0];
    doc["current_out"] = out[1];
    doc["power_out"] = out[2];
    doc["powerfactor"] = out[3];
    doc["phase"] = out[4];
    doc["imaginaryPower"] = out[5];
    doc["frequency"] = out[6];
    char buf[512];
    size_t len = serializeJson(doc, buf);
    webSocket.broadcastTXT(buf, len);
}

void HBridgeWebServer::onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Connected from %s\n", num, ip.toString().c_str());
            StaticJsonDocument<128> statusDoc;
            statusDoc["type"] = "status";
            statusDoc["isRunning"] = _isRunning;
            char buffer[128];
            size_t length = serializeJson(statusDoc, buffer);
            webSocket.sendTXT(num, buffer, length);
            break;
        }
        default: break;
    }
}

void HBridgeWebServer::broadcastTrip(const char* reason) {
    StaticJsonDocument<192> doc;
    doc["type"] = "trip";
    doc["reason"] = reason;
    char buf[192];
    size_t len = serializeJson(doc, buf);
    webSocket.broadcastTXT(buf, len);
    _isRunning = false;
    broadcastStatus();
}
