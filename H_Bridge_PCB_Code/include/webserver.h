/*************************************************************************
 * @file webserver.h
 * @date 2025/01/31
 *
 ************************************************************************/

#ifndef XENOSHELL_WEBSERVER_H
#define XENOSHELL_WEBSERVER_H

/*************************************************************************
 * Includes
 ************************************************************************/
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <WiFi.h>
#include "PWM.h"

/*************************************************************************
 * Class
 ************************************************************************/
class HBridgeWebServer {
public:
    static void initWiFi();
    static void initServer();
    static void handleWebSocket();
    static void sendStatusUpdate();
    static void resetDefaults();

    static void loopWebSocket() {
        webSocket.loop();
    }

    static void updateMeasurements(float* input, float* output);

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
