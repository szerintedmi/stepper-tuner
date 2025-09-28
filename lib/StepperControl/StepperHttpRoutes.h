#pragma once

#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

#include <AsyncJson.h>
#include <ArduinoJson.h>

#include "StepperMotion.h"

namespace StepperControl
{

class HttpRoutes
{
public:
  explicit HttpRoutes(Motion &motion) : motion(motion) {}

  void attach(AsyncWebServer &server);

private:
  Motion &motion;
  bool fsMounted = false;

  static const char *unitsToString(TargetUnits units);
  static TargetUnits unitsFromString(const char *units);
  static const char *modeToString(RunMode mode);

  void ensureFilesystemMounted();
  void writeState(JsonVariant root);
  void sendState(AsyncWebServerRequest *request);

  void handleSettings(AsyncWebServerRequest *request, JsonVariant &json);
  void handleRun(AsyncWebServerRequest *request, JsonVariant &json);
  void handleDriver(AsyncWebServerRequest *request, JsonVariant &json);
  void handleStop(AsyncWebServerRequest *request, JsonVariant &json);
  void handleReset(AsyncWebServerRequest *request, JsonVariant &json);
  void handleAddMotor(AsyncWebServerRequest *request, JsonVariant &json);
  void handleUpdateMotor(AsyncWebServerRequest *request, JsonVariant &json);
  void handleRemoveMotor(AsyncWebServerRequest *request, JsonVariant &json);

  void sendError(AsyncWebServerRequest *request, int code, const char *message);
};

} // namespace StepperControl
