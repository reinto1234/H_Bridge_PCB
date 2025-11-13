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

    static void broadcastTrip(const char* reason);

private:
    static String processor(const String& var);
    static void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
    static void broadcastStatus();
    
    static float_t _VRMS;
    static String _modulationType;
    static bool _isRunning;
    
    static AsyncWebServer server;
    static WebSocketsServer webSocket;
    static StaticJsonDocument<512> doc;

};

#endif // WEBSERVER_H
