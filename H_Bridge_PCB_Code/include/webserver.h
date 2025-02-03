#ifndef XENOSHELL_WEBSERVER_H
#define XENOSHELL_WEBSERVER_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <WiFi.h>

class HBridgeWebServer {
public:
    static void initWiFi();
    static void initServer();
    static void handleWebSocket();
    static void sendStatusUpdate();

private:
    static String processor(const String& var);
    
    static String _switchingFrequency;
    static String _modulationType;
    static bool _isRunning;
    
    static AsyncWebServer server;
    static WebSocketsServer webSocket;
    static StaticJsonDocument<512> doc;
};

#endif // WEBSERVER_H
