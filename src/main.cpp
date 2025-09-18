#include <Arduino.h>
#include <ESPAsyncWebServer.h>

#include "StepperControl.h"
#include "WifiPortal.h"

AsyncWebServer webServer(80);
WifiPortal wifiPortal(MESH_SSID, MESH_PASS);

void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("Setup(): start");

  StepperControl::begin(webServer);
  wifiPortal.beginAndConnect(webServer, 10000);
  webServer.begin();

  Serial.println("Setup(): done");
}

void loop()
{
  StepperControl::loop();
}

