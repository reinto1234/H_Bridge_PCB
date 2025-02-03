#include "webserver.h"

// Define static variables
String HBridgeWebServer::_switchingFrequency;
String HBridgeWebServer::_modulationType;
bool HBridgeWebServer::_isRunning;
AsyncWebServer HBridgeWebServer::server(80);
WebSocketsServer HBridgeWebServer::webSocket(81);
StaticJsonDocument<512> HBridgeWebServer::doc;

void HBridgeWebServer::initWiFi() {
    WiFi.softAP("HBridge_Control", "12345678");
    Serial.println("WiFi Access Point Started. Connect to 'HBridge_Control'");

    // Get and print the IP address
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
}

void HBridgeWebServer::initServer() {
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed!");
        return;
    }
    Serial.println("LittleFS Mounted Successfully");

    // Serve index.html with processor function
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (LittleFS.exists("/index.html")) {
            Serial.println("Serving index.html");
            request->send(LittleFS, "/index.html", String(), false, processor);
        } else {
            Serial.println("index.html not found");
            request->send(404, "text/plain", "File Not Found");
        }
    });

    // Serve CSS
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (LittleFS.exists("/style.css")) {
            Serial.println("Serving style.css");
            request->send(LittleFS, "/style.css", "text/css");
        } else {
            Serial.println("style.css not found");
            request->send(404, "text/plain", "File Not Found");
        }
    });

    // Serve JavaScript
    server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (LittleFS.exists("/script.js")) {
            Serial.println("Serving script.js");
            request->send(LittleFS, "/script.js", "application/javascript");
        } else {
            Serial.println("script.js not found");
            request->send(404, "text/plain", "File Not Found");
        }
    });

    // Handle Updates
    server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("freq")) {
            String freq = request->getParam("freq")->value();
            int freqValue = freq.toInt();
            if (freqValue >= 1000 && freqValue <= 45000) {
                _switchingFrequency = freq;
                Serial.println("Frequency updated to: " + freq);
            } else {
                Serial.println("Invalid frequency value: " + freq);
            }
        }
        if (request->hasParam("modulation")) {
            _modulationType = request->getParam("modulation")->value();
            Serial.println("Modulation type updated to: " + _modulationType);
        }
        request->send(200, "text/plain", "Update Received");
    });

    server.begin();
    Serial.println("HTTP server started");
}

String HBridgeWebServer::processor(const String& var) {
    // Implement your processor function here
    return String();
}
