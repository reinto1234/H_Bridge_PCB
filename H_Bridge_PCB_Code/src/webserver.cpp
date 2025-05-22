/************************************************************************
 * @file webserver.cpp
 * @brief Web server and WebSocket implementation for the H-Bridge Inverter System
 *
 * This file contains the implementation of the web server and WebSocket
 * functionalities, including serving static files and handling HTTP requests.
 ************************************************************************/

#include "webserver.h"
#include "PWM.h"
#include "LittleFS.h"
#include "mutexdefinitions.h"

// Define static variables
uint32_t HBridgeWebServer::_switchingFrequency = 20000; // Default switching frequency
String HBridgeWebServer::_modulationType = "BIPOLAR"; // Default modulation type
bool HBridgeWebServer::_isRunning = false; // Inverter running state
AsyncWebServer HBridgeWebServer::server(80); // Web server instance
WebSocketsServer HBridgeWebServer::webSocket(81); // WebSocket server instance
StaticJsonDocument<512> HBridgeWebServer::doc; // JSON document for communication

// Converts modulation type string to enum value
ModulationType HBridgeWebServer::getModulationType() {
    if (_modulationType == "Bipolar") {
        Serial.println("Modulation Mode: BIPOLAR");
        return BIPOLAR;
    } else {
        Serial.println("Modulation Mode: Unipolar");
        return UNIPOLAR;
    }
}

void HBridgeWebServer::resetDefaults() {
    _switchingFrequency = 20000; // Hz
    _modulationType = "BIPOLAR";
    _isRunning = false;
}

// Initializes WiFi Access Point
void HBridgeWebServer::initWiFi() {
    resetDefaults();
    WiFi.softAP("HBridge_Control", "12345678");
    Serial.println("WiFi Access Point Started. Connect to 'HBridge_Control'");
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
}

// Initializes Web Server and Serves Static Files
void HBridgeWebServer::initServer() {
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed!");
        return;
    }
    Serial.println("LittleFS Mounted Successfully");

    // Serve HTML, CSS, JavaScript, and images
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/index.html", String(), false, processor);
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

    // Handle frequency and modulation updates
    server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!_isRunning) {
            if (request->hasParam("freq")) {
                int freqValue = request->getParam("freq")->value().toInt();
                if (freqValue >= 1000 && freqValue <= 45000) {
                    _switchingFrequency = freqValue;
                }
            }
            if (request->hasParam("modulation")) {
                _modulationType = request->getParam("modulation")->value();
            }
        }
        request->send(200, "text/plain", "Update Received");
    });

    // Start the inverter
    server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!_isRunning) {
            if (request->hasParam("freq")) {
                int freqValue = request->getParam("freq")->value().toInt();
                if (freqValue < 1000 || freqValue > 45000) {
                    request->send(400, "text/plain", "Invalid Frequency.");
                    return;
                }
                _switchingFrequency = freqValue;
            }
            if (request->hasParam("modulation")) {
                _modulationType = request->getParam("modulation")->value();
            }
            ModulationType modType = getModulationType();
            stopInverter();
            startInverter(0.1, 0.01, 0, 1023, modType, _switchingFrequency);
            _isRunning = true;
            request->send(200, "text/plain", "Inverter Started");
        }
    });

    // Stop the inverter
    server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (_isRunning) {
            stopInverter();
            _isRunning = false;
        }
        request->send(200, "text/plain", "Inverter Stopped");
    });

    server.begin();
    Serial.println("HTTP Server Started");
    webSocket.begin();
    Serial.println("WebSocket Server Started");
}

// Web page processor function (placeholder)
String HBridgeWebServer::processor(const String& var) {
    return String();
}

// Updates measurements and broadcasts via WebSocket
void HBridgeWebServer::updateMeasurements(float* input, float* output) {
    StaticJsonDocument<256> jsonDoc;
    if (xSemaphoreTake(measurementinMutex, portMAX_DELAY) == pdTRUE) {
        jsonDoc["voltage"] = input[0];
        jsonDoc["current"] = input[1];
        jsonDoc["power"] = input[2];
        xSemaphoreGive(measurementinMutex);
    }
    if (xSemaphoreTake(measurementoutMutex, portMAX_DELAY) == pdTRUE) {
        jsonDoc["voltage_out"] = output[0];
        jsonDoc["current_out"] = output[1];
        jsonDoc["power_out"] = output[2];
        jsonDoc["powerfactor"] = output[3];
        jsonDoc["phase"] = output[4];
        jsonDoc["imaginaryPower"] = output[5];
        jsonDoc["frequency"] = output[6];
        xSemaphoreGive(measurementoutMutex);
    }
    char buffer[512];
    serializeJson(jsonDoc, buffer);
    webSocket.broadcastTXT(buffer);
}
