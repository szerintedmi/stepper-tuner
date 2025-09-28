#include "StepperHttpRoutes.h"

#include <Arduino.h>
#include <cstring>

namespace StepperControl
{

const char *HttpRoutes::unitsToString(TargetUnits units)
{
  switch (units)
  {
  case TargetUnits::Steps:
    return "steps";
  case TargetUnits::Degrees:
    return "degrees";
  case TargetUnits::Revs:
  default:
    return "revs";
  }
}

TargetUnits HttpRoutes::unitsFromString(const char *units)
{
  if (!units)
  {
    return TargetUnits::Revs;
  }
  if (strcmp(units, "steps") == 0)
  {
    return TargetUnits::Steps;
  }
  if (strcmp(units, "degrees") == 0)
  {
    return TargetUnits::Degrees;
  }
  return TargetUnits::Revs;
}

const char *HttpRoutes::modeToString(RunMode mode)
{
  switch (mode)
  {
  case RunMode::Single:
    return "single";
  case RunMode::PingPong:
    return "pingpong";
  case RunMode::Idle:
  default:
    return "idle";
  }
}

void HttpRoutes::ensureFilesystemMounted()
{
  if (fsMounted)
  {
    return;
  }
  if (LittleFS.begin(false) || LittleFS.begin(true))
  {
    fsMounted = true;
  }
  else
  {
    Serial.println("StepperControl: LittleFS mount failed");
  }
}

void HttpRoutes::writeState(JsonVariant root)
{
  StepperState st = motion.state();

  JsonObject obj = root.to<JsonObject>();
  JsonObject settings = obj["settings"].to<JsonObject>();
  settings["stepsPerRev"] = st.settings.stepsPerRev;
  settings["targetUnits"] = unitsToString(st.settings.targetUnits);
  settings["targetRevs"] = st.settings.targetRevs;
  settings["targetDegrees"] = st.settings.targetDegrees;
  settings["targetSteps"] = st.settings.targetSteps;
  settings["maxSpeed"] = st.settings.maxSpeed;
  settings["maxSpeedLimit"] = st.settings.maxSpeedLimit;
  settings["acceleration"] = st.settings.acceleration;
  settings["autoSleep"] = st.settings.autoSleep;

  JsonObject status = obj["status"].to<JsonObject>();
  status["mode"] = modeToString(st.status.mode);
  status["moving"] = st.status.moving;
  status["driverAwake"] = st.status.driverAwake;
  status["direction"] = st.status.direction;
  status["segmentSteps"] = st.status.segmentSteps;
  status["currentPosition"] = st.status.currentPosition;
  status["targetPosition"] = st.status.targetPosition;
  status["distanceToGo"] = st.status.distanceToGo;
  status["speed"] = st.status.speed;

  JsonObject last = status["lastRun"].to<JsonObject>();
  last["valid"] = st.lastRun.valid;
  last["aborted"] = st.lastRun.aborted;
  last["startMs"] = st.lastRun.startMs;
  last["durationMs"] = st.lastRun.durationMs;
  last["steps"] = st.lastRun.steps;
  last["loopMaxGapUs"] = st.lastRun.loopMaxGapUs;
}

void HttpRoutes::sendState(AsyncWebServerRequest *request)
{
  auto *response = new AsyncJsonResponse();
  writeState(response->getRoot());
  response->setLength();
  request->send(response);
}

void HttpRoutes::sendError(AsyncWebServerRequest *request, int code, const char *message)
{
  request->send(code, "text/plain", message ? message : "Error");
}

void HttpRoutes::handleSettings(AsyncWebServerRequest *request, JsonVariant &json)
{
  if (!json.is<JsonObject>())
  {
    sendError(request, 400, "Expected JSON object");
    return;
  }

  StepperState current = motion.state();
  JsonObject obj = json.as<JsonObject>();

  SettingsPatch patch;

  if (!obj["stepsPerRev"].isNull())
  {
    patch.hasStepsPerRev = true;
    patch.stepsPerRev = obj["stepsPerRev"].as<long>();
  }

  if (!obj["targetUnits"].isNull())
  {
    patch.hasTargetUnits = true;
    patch.targetUnits = unitsFromString(obj["targetUnits"].as<const char *>());
  }

  bool hasTargetValue = false;
  bool valueImposesUnits = false;
  double targetValue = 0.0;
  TargetUnits valueUnits = TargetUnits::Revs;

  if (!obj["targetSteps"].isNull())
  {
    targetValue = obj["targetSteps"].as<double>();
    valueUnits = TargetUnits::Steps;
    valueImposesUnits = true;
    hasTargetValue = true;
  }
  else if (!obj["targetDegrees"].isNull())
  {
    targetValue = obj["targetDegrees"].as<double>();
    valueUnits = TargetUnits::Degrees;
    valueImposesUnits = true;
    hasTargetValue = true;
  }
  else if (!obj["targetRevs"].isNull())
  {
    targetValue = obj["targetRevs"].as<double>();
    valueUnits = TargetUnits::Revs;
    valueImposesUnits = true;
    hasTargetValue = true;
  }
  else if (!obj["targetValue"].isNull())
  {
    targetValue = obj["targetValue"].as<double>();
    valueUnits = patch.hasTargetUnits ? patch.targetUnits : current.settings.targetUnits;
    hasTargetValue = true;
  }

  if (hasTargetValue)
  {
    patch.hasTargetValue = true;
    patch.targetValue = targetValue;
    if (valueImposesUnits)
    {
      patch.hasTargetUnits = true;
      patch.targetUnits = valueUnits;
    }
    else if (!patch.hasTargetUnits)
    {
      patch.hasTargetUnits = true;
      patch.targetUnits = valueUnits;
    }
  }

  if (!obj["maxSpeed"].isNull())
  {
    patch.hasMaxSpeed = true;
    patch.maxSpeed = obj["maxSpeed"].as<float>();
  }
  if (!obj["acceleration"].isNull())
  {
    patch.hasAcceleration = true;
    patch.acceleration = obj["acceleration"].as<float>();
  }
  if (!obj["autoSleep"].isNull())
  {
    patch.hasAutoSleep = true;
    patch.autoSleep = obj["autoSleep"].as<bool>();
  }

  motion.applySettings(patch);

  sendState(request);
}

void HttpRoutes::handleRun(AsyncWebServerRequest *request, JsonVariant &json)
{
  if (!json.is<JsonObject>())
  {
    sendError(request, 400, "Expected JSON object");
    return;
  }

  JsonObject obj = json.as<JsonObject>();
  const char *mode = obj["mode"].is<const char *>() ? obj["mode"].as<const char *>() : "single";
  int direction = obj["direction"].is<int>() ? obj["direction"].as<int>() : 1;

  if (strcmp(mode, "stop") == 0)
  {
    motion.stop(true);
    if (motion.autoSleepEnabled())
    {
      motion.setDriverAwake(false);
    }
  }
  else if (strcmp(mode, "single") == 0)
  {
    if (!motion.startSingle(direction))
    {
      sendError(request, 409, "Stepper busy or invalid target");
      return;
    }
  }
  else if (strcmp(mode, "pingpong") == 0)
  {
    if (!motion.startPingPong(direction))
    {
      sendError(request, 409, "Stepper busy or invalid target");
      return;
    }
  }
  else
  {
    sendError(request, 400, "Unknown mode");
    return;
  }

  sendState(request);
}

void HttpRoutes::handleDriver(AsyncWebServerRequest *request, JsonVariant &json)
{
  if (!json.is<JsonObject>())
  {
    sendError(request, 400, "Expected JSON object");
    return;
  }

  JsonObject obj = json.as<JsonObject>();
  if (!obj["awake"].is<bool>())
  {
    sendError(request, 400, "Missing awake bool");
    return;
  }

  bool awake = obj["awake"].as<bool>();
  motion.setDriverAwake(awake);
  sendState(request);
}

void HttpRoutes::attach(AsyncWebServer &server)
{
  ensureFilesystemMounted();

  server.serveStatic("/", LittleFS, "/")
      .setDefaultFile("index.html")
      .setFilter([](AsyncWebServerRequest *r) {
        const String &u = r->url();
        if (u.startsWith("/api/"))
        {
          return false;
        }
        return !(u.startsWith("/wifi/api/") || u == "/wifi/api");
      });

  server.on("/api/state", HTTP_GET, [this](AsyncWebServerRequest *request) {
    sendState(request);
  });

  auto *settingsHandler = new AsyncCallbackJsonWebHandler("/api/settings", [this](AsyncWebServerRequest *request, JsonVariant &json) {
    handleSettings(request, json);
  });
  server.addHandler(settingsHandler);

  auto *runHandler = new AsyncCallbackJsonWebHandler("/api/run", [this](AsyncWebServerRequest *request, JsonVariant &json) {
    handleRun(request, json);
  });
  server.addHandler(runHandler);

  server.on("/api/settings/default/save", HTTP_POST, [this](AsyncWebServerRequest *request) {
    if (!motion.saveDefaults())
    {
      sendError(request, 500, "Failed to save defaults");
      return;
    }
    sendState(request);
  });

  server.on("/api/settings/default/restore", HTTP_POST, [this](AsyncWebServerRequest *request) {
    if (!motion.restoreDefaults())
    {
      sendError(request, 500, "Failed to load defaults");
      return;
    }
    sendState(request);
  });

  server.on("/api/stop", HTTP_POST, [this](AsyncWebServerRequest *request) {
    motion.stop(true);
    if (motion.autoSleepEnabled())
    {
      motion.setDriverAwake(false);
    }
    sendState(request);
  });

  server.on("/api/reset", HTTP_POST, [this](AsyncWebServerRequest *request) {
    motion.reset();
    sendState(request);
  });

  auto *driverHandler = new AsyncCallbackJsonWebHandler("/api/driver", [this](AsyncWebServerRequest *request, JsonVariant &json) {
    handleDriver(request, json);
  });
  server.addHandler(driverHandler);
}

} // namespace StepperControl
