#ifndef XENOSHELL_WEBSERVER_H
#define XENOSHELL_WEBSERVER_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <WiFi.h>
#include "PWM.h"

//no comments yet

class HBridgeWebServer {
public:
    static void initWiFi();
    static void initServer();
    static void handleWebSocket();
    static void sendStatusUpdate();

    static void loopWebSocket() {
        webSocket.loop();
    }

private:
    static String processor(const String& var);
    
    static uint32_t _switchingFrequency;
    static String _modulationType;
    static bool _isRunning;
    
    static AsyncWebServer server;
    static WebSocketsServer webSocket;
    static StaticJsonDocument<512> doc;

    static ModulationType getModulationType(); // New helper function
};

#endif // WEBSERVER_H
