#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <esp_now.h>
#include "esp_wifi.h" // for esp_wifi_get_channel

#define MESH_SSID "ESP32-WiFi-Setup"

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef MESH_PASS
#define MESH_PASS "changeme"
#endif

// static constexpr uint8_t DEFAULT_ESPNOW_CHANNEL = 1; // use same channel on all nodes
static const uint8_t DEFAULT_BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

class WifiPortal
{
public:
    WifiPortal(const char *apSsid = MESH_SSID,
               const char *apPass = MESH_PASS,
               const char *prefsNS = "wifi")
        : _ns(prefsNS), _apSsid(apSsid), _apPass(apPass) {}

    void beginAndConnect(AsyncWebServer &webServer, uint32_t staTimeoutMs = 15000)
    {
        _webServer = &webServer;
        _prefs.begin(_ns.c_str(), false);
        _ssid = _prefs.getString("ssid", "");
        _pass = _prefs.getString("pass", "");

        LittleFS.begin(false) || LittleFS.begin(true); // idempotent

        if (_ssid.isEmpty())
        {
            Serial.println("WifiPortal: No saved SSID found.");
            _startSetupAP();
        }
        else
        {
            Serial.printf("WifiPortal: Connecting to %s\n", _ssid.c_str());
            _connectToWifi(staTimeoutMs);
        }

        beginEspNowMesh();
        _attachRoutes(); // adds /wifi, /wifi/save, /wifi/reset
    }

    // Clear saved creds (kept public in case you want to call from code)
    void clearCreds()
    {
        _prefs.remove("ssid");
        _prefs.remove("pass");
        _ssid = "";
        _pass = "";
    }

    uint8_t getPrimaryChannel() const { return WiFi.channel(); }

private:
    String _ns;
    String _apSsid, _apPass;
    Preferences _prefs;
    AsyncWebServer *_webServer = nullptr;
    String _ssid, _pass;

    void _saveCreds(const String &s, const String &p)
    {
        _prefs.putString("ssid", s);
        _prefs.putString("pass", p);
        _ssid = s;
        _pass = p;
    }

    // Starts softAP and ESP-NOW mesh
    bool beginEspNowMesh()
    {
        // Force AP to the chosen channel at boot
        uint8_t channel = WiFi.channel();
        if (!WiFi.softAP(_apSsid, _apPass, channel, /*hidden=*/0, /*max conn=*/3))
        {
            Serial.println("WifiPortal: Error initializing softAP for Mesh");
            return false;
        }

        if (esp_now_init() != ESP_OK)
        {
            Serial.println("WifiPortal: Error initializing ESP-NOW");
            return false;
        }

        // Add broadcast peer (keeps API happy across core versions)
        esp_now_peer_info_t peer{};
        memcpy(peer.peer_addr, DEFAULT_BROADCAST_MAC, 6);
        peer.ifidx = WIFI_IF_AP;
        peer.channel = 0; // transmit on own current primary channel. Peers still need to detect change and jump channel
        peer.encrypt = false;
        esp_now_add_peer(&peer);

        Serial.printf("WifiPortal: Mesh SoftAP started. SSID:%s Ch:%u IP:%s\n",
                      _apSsid, channel, WiFi.softAPIP().toString().c_str());
        return true;
    }

    void _startSetupAP()
    {
        Serial.printf("WifiPortal: Starting Wifi Mesh in AP mode for config. SSID: %s password: %s\n", _apSsid, _apPass);
        WiFi.mode(WIFI_AP);
        // esp_wifi_set_ps(WIFI_PS_NONE); // don't go to sleep NB: can't disable sleep with BT enabled
        WiFi.softAP(_apSsid, _apPass); // starting on channel 1 by default
    }

    void _connectToWifi(uint32_t timeoutMs)
    {
        WiFi.mode(WIFI_AP_STA);
        esp_wifi_set_ps(WIFI_PS_NONE); // don't go to sleep
        WiFi.setAutoReconnect(true);
        WiFi.persistent(false); // we storing credentials ourselves in prefs
        WiFi.begin(_ssid.c_str(), _pass.c_str());

        uint32_t start = millis();
        Serial.printf("WifiPortal: Connecting to %s...\n", _ssid.c_str());
        while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs)
        {
            delay(100);
        }

        if (WiFi.status() == WL_CONNECTED)
        {
            Serial.printf("WifiPortal: Connected to %s on channel %d\n", WiFi.SSID().c_str(), WiFi.channel());
            Serial.printf("  IP address: %s\n", WiFi.localIP().toString().c_str());
            Serial.printf("  MAC address: %s\n", WiFi.macAddress().c_str());
        }
        else
        {
            Serial.printf("WifiPortal: Error connecting to configured WiFi AP with SSID: %s\n", _ssid.c_str());
            _startSetupAP();
        }

        return;
    }

    void _attachRoutes()
    {
        if (!_webServer)
            return;

        // Common status handler used for two routes: /wifi/api/status
        auto statusHandler = [this](AsyncWebServerRequest *req)
        {
            uint8_t pri = 0;
            wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
            esp_wifi_get_channel(&pri, &sec);

            const bool sta = (WiFi.status() == WL_CONNECTED);
            const bool ap = (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA);

            auto *res = req->beginResponseStream("application/json");
            res->print('{');

            // STA runtime status
            res->print("\"sta\":{");
            res->printf("\"connected\":%s,", sta ? "true" : "false");
            res->printf("\"ssid\":\"%s\",", sta ? WiFi.SSID().c_str() : "");
            res->printf("\"channel\":%u,", sta ? WiFi.channel() : 0);
            res->printf("\"ip\":\"%s\",", sta ? WiFi.localIP().toString().c_str() : "");
            res->printf("\"mac\":\"%s\"", WiFi.macAddress().c_str());
            res->print("},");

            // AP runtime status
            res->print("\"ap\":{");
            res->printf("\"enabled\":%s,", ap ? "true" : "false");
            res->printf("\"ssid\":\"%s\",", ap ? WiFi.softAPSSID().c_str() : "");
            res->printf("\"channel\":%u,", ap ? pri : 0);
            res->printf("\"ip\":\"%s\",", ap ? WiFi.softAPIP().toString().c_str() : "");
            res->printf("\"clients\":%d", ap ? WiFi.softAPgetStationNum() : 0);
            res->print("},");

            // Persisted credentials (password not exposed)
            res->print("\"saved\":{");
            res->printf("\"ssid\":\"%s\",", _ssid.c_str());
            res->printf("\"hasPass\":%s", _pass.length() ? "true" : "false");
            res->print("}");

            res->print('}');
            req->send(res);
        };

        // Status routes
        _webServer->on("/wifi/api/status", HTTP_GET, statusHandler);

        // Serve static UI under /wifi
        _webServer->serveStatic("/wifi", LittleFS, "/wifi")
            .setDefaultFile("index.html")
            .setFilter([](AsyncWebServerRequest *r)
                       {
                const String& u = r->url();
                return !(u.startsWith("/wifi/api/") || u == "/wifi/api"); });
        ;

        // Save credentials (form: ssid, pass)
        _webServer->on("/wifi/save", HTTP_POST, [this](AsyncWebServerRequest *req)
                       {
        String s, p;
        if (req->hasParam("ssid", true)) s = req->getParam("ssid", true)->value();
        if (req->hasParam("pass", true)) p = req->getParam("pass", true)->value();

        if (s.isEmpty()) { req->send(400, "text/plain", "SSID required"); return; }

        _saveCreds(s, p);

        // Try immediate connect (non-blocking-ish loop already inside)
        _connectToWifi(12000);

        if (WiFi.status() == WL_CONNECTED) {
            req->send(200, "application/json",
                      String("{\"ok\":true,\"connected\":true,\"ip\":\"") + WiFi.localIP().toString() + "\"}");
        } else {
            req->send(200, "application/json", "{\"ok\":true,\"connected\":false}");
        } });

        // Reset/forget saved credentials
        _webServer->on("/wifi/reset", HTTP_POST, [this](AsyncWebServerRequest *req)
                       {
        clearCreds();
        req->send(200, "application/json", "{\"ok\":true}"); });
    }
};
