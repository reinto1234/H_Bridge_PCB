#include "webserver.h"
#include "PWM.h"
#include "LittleFS.h"

// Define static variables
uint32_t HBridgeWebServer::_switchingFrequency = 20000;
String HBridgeWebServer::_modulationType = "BIPOLAR";
bool HBridgeWebServer::_isRunning = false;
AsyncWebServer HBridgeWebServer::server(80);
WebSocketsServer HBridgeWebServer::webSocket(81);
StaticJsonDocument<512> HBridgeWebServer::doc;

// New helper function to convert _modulationType string to enum
ModulationType HBridgeWebServer::getModulationType() {
    if (_modulationType == "Bipolar") {
        Serial.println("Modulation Mode: BIPOLAR");
        return BIPOLAR;
    } else {
        Serial.println("Modulation Mode: Unipolar");
        return UNIPOLAR;
    }
}


void HBridgeWebServer::initWiFi() {
    WiFi.softAP("HBridge_Control", "12345678");
    Serial.println("WiFi Access Point Started. Connect to 'HBridge_Control'");

    // Get and print the IP address
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
}

void HBridgeWebServer::initServer() {
    // Initialize the file system for serving the website
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed!");
        return;
    }
    Serial.println("LittleFS Mounted Successfully");

    // Serve index.html
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (LittleFS.exists("/index.html")) {
            Serial.println("Serving index.html");
            request->send(LittleFS, "/index.html", String(), false, processor);
        } else {
            Serial.println("index.html not found");
            request->send(404, "text/plain", "File Not Found");
        }
    });

    // Serve CSS file
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (LittleFS.exists("/style.css")) {
            Serial.println("Serving style.css");
            request->send(LittleFS, "/style.css", "text/css");
        } else {
            Serial.println("style.css not found");
            request->send(404, "text/plain", "File Not Found");
        }
    });

    // Serve JavaScript file
    server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (LittleFS.exists("/script.js")) {
            Serial.println("Serving script.js");
            request->send(LittleFS, "/script.js", "application/javascript");
        } else {
            Serial.println("script.js not found");
            request->send(404, "text/plain", "File Not Found");
        }
    });

    // Handle frequency and modulation updates
    server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!_isRunning) {  // Allow updates only if the inverter is not running
            if (request->hasParam("freq")) {
                String freq = request->getParam("freq")->value();
                int freqValue = freq.toInt();
                if (freqValue >= 1000 && freqValue <= 45000) {
                    _switchingFrequency = freqValue;
                    Serial.println("Frequency updated to: " + freq);
                } else {
                    Serial.println("Invalid frequency value: " + freq);
                }
            }
            if (request->hasParam("modulation")) {
                String newModulationType = request->getParam("modulation")->value();
                if (newModulationType == "Bipolar" || newModulationType == "Unipolar") {
                    _modulationType = newModulationType;
                    Serial.println("Modulation type updated to: " + _modulationType);
                } else {
                    Serial.println("Invalid modulation type: " + newModulationType);
                }
            }
        }
        request->send(200, "text/plain", "Update Received");
    });

    // Start the inverter
 server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!_isRunning) {
        // Validate and update frequency
        if (request->hasParam("freq")) {
            String freq = request->getParam("freq")->value();
            int freqValue = freq.toInt();
            if (freqValue >= 1000 && freqValue <= 45000) {
                _switchingFrequency = freqValue;
                Serial.println("Frequency updated to: " + String(freqValue));
            } else {
                Serial.println("Invalid frequency value: " + String(freqValue));
                request->send(400, "text/plain", "Invalid Frequency. Must be between 1000Hz - 45000Hz.");
                return;
            }
        }

        // Validate and update modulation type
        if (request->hasParam("modulation")) {
            String newModulationType = request->getParam("modulation")->value();
            if (newModulationType == "Bipolar" || newModulationType == "Unipolar") {
                _modulationType = newModulationType;
                Serial.println("Modulation type updated to: " + _modulationType);
            } else {
                Serial.println("Invalid modulation type: " + newModulationType);
                request->send(400, "text/plain", "Invalid Modulation Type.");
                return;
            }
        }

        ModulationType modType = getModulationType();

        Serial.println("Starting inverter with:");
        Serial.println("  Frequency: " + String(_switchingFrequency));
        Serial.println("  Modulation: " + _modulationType);

        stopInverter();
        startInverter(0.1, 0.01, 0, 1023, modType, _switchingFrequency);
        _isRunning = true;
        Serial.println("Inverter started!");

        request->send(200, "text/plain", "Inverter Started");
    }
});


    // Stop the inverter
    server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (_isRunning) {
            stopInverter();
            _isRunning = false;
            Serial.println("Inverter stopped!");
        }
        request->send(200, "text/plain", "Inverter Stopped");
    });

    // Start the web server
    server.begin();
    Serial.println("HTTP Server Started");
}

String HBridgeWebServer::processor(const String& var) {
    return String();
}
