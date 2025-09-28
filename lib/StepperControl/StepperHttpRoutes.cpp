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

  JsonArray motors = obj["motors"].to<JsonArray>();
  for (const auto &motor : st.motors)
  {
    JsonObject node = motors.add<JsonObject>();
    node["id"] = motor.id;
    JsonObject pins = node["pins"].to<JsonObject>();
    pins["step"] = motor.pins.step;
    pins["dir"] = motor.pins.dir;
    pins["sleep"] = motor.pins.sleep;

    node["mode"] = modeToString(motor.mode);
    node["moving"] = motor.moving;
    node["driverAwake"] = motor.driverAwake;
    node["direction"] = motor.direction;
    node["segmentSteps"] = motor.segmentSteps;
    node["currentPosition"] = motor.currentPosition;
    node["targetPosition"] = motor.targetPosition;
    node["distanceToGo"] = motor.distanceToGo;
    node["speed"] = motor.speed;

    JsonObject last = node["lastRun"].to<JsonObject>();
    last["valid"] = motor.lastRun.valid;
    last["aborted"] = motor.lastRun.aborted;
    last["startMs"] = motor.lastRun.startMs;
    last["durationMs"] = motor.lastRun.durationMs;
    last["steps"] = motor.lastRun.steps;
    last["loopMaxGapUs"] = motor.lastRun.loopMaxGapUs;
  }
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
  if (!(obj["motorId"].is<unsigned int>() || obj["motorId"].is<int>()))
  {
    sendError(request, 400, "Missing motorId");
    return;
  }
  uint16_t motorId = static_cast<uint16_t>(obj["motorId"].as<unsigned int>());
  if (!motion.hasMotor(motorId))
  {
    sendError(request, 404, "Unknown motor");
    return;
  }

  const char *mode = obj["mode"].is<const char *>() ? obj["mode"].as<const char *>() : "single";
  int direction = obj["direction"].is<int>() ? obj["direction"].as<int>() : 1;

  if (strcmp(mode, "single") == 0)
  {
    if (!motion.startSingle(motorId, direction))
    {
      sendError(request, 409, "Motor busy or invalid target");
      return;
    }
  }
  else if (strcmp(mode, "pingpong") == 0)
  {
    if (!motion.startPingPong(motorId, direction))
    {
      sendError(request, 409, "Motor busy or invalid target");
      return;
    }
  }
  else if (strcmp(mode, "stop") == 0)
  {
    motion.stop(motorId, true);
    if (motion.autoSleepEnabled())
    {
      motion.setDriverAwake(motorId, false);
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
  if (!(obj["motorId"].is<unsigned int>() || obj["motorId"].is<int>()))
  {
    sendError(request, 400, "Missing motorId");
    return;
  }
  if (!obj["awake"].is<bool>())
  {
    sendError(request, 400, "Missing awake bool");
    return;
  }

  uint16_t motorId = static_cast<uint16_t>(obj["motorId"].as<unsigned int>());
  if (!motion.hasMotor(motorId))
  {
    sendError(request, 404, "Unknown motor");
    return;
  }

  bool awake = obj["awake"].as<bool>();
  motion.setDriverAwake(motorId, awake);
  sendState(request);
}

void HttpRoutes::handleStop(AsyncWebServerRequest *request, JsonVariant &json)
{
  if (!json.is<JsonObject>())
  {
    sendError(request, 400, "Expected JSON object");
    return;
  }

  JsonObject obj = json.as<JsonObject>();
  bool aborted = obj["aborted"].is<bool>() ? obj["aborted"].as<bool>() : true;
  bool sleepDriver = obj["sleepDriver"].is<bool>() ? obj["sleepDriver"].as<bool>() : motion.autoSleepEnabled();

  if (obj["motorId"].isNull())
  {
    motion.stopAll(aborted);
    if (sleepDriver)
    {
      motion.setDriverAwakeAll(false);
    }
  }
  else if (obj["motorId"].is<unsigned int>() || obj["motorId"].is<int>())
  {
    uint16_t motorId = static_cast<uint16_t>(obj["motorId"].as<unsigned int>());
    if (!motion.hasMotor(motorId))
    {
      sendError(request, 404, "Unknown motor");
      return;
    }
    motion.stop(motorId, aborted);
    if (sleepDriver)
    {
      motion.setDriverAwake(motorId, false);
    }
  }
  else
  {
    sendError(request, 400, "Invalid motorId");
    return;
  }

  sendState(request);
}

void HttpRoutes::handleReset(AsyncWebServerRequest *request, JsonVariant &json)
{
  if (!json.is<JsonObject>())
  {
    sendError(request, 400, "Expected JSON object");
    return;
  }

  JsonObject obj = json.as<JsonObject>();
  if (obj["motorId"].isNull())
  {
    motion.resetAll();
  }
  else if (obj["motorId"].is<unsigned int>() || obj["motorId"].is<int>())
  {
    uint16_t motorId = static_cast<uint16_t>(obj["motorId"].as<unsigned int>());
    if (!motion.hasMotor(motorId))
    {
      sendError(request, 404, "Unknown motor");
      return;
    }
    motion.reset(motorId);
  }
  else
  {
    sendError(request, 400, "Invalid motorId");
    return;
  }

  sendState(request);
}

void HttpRoutes::handleAddMotor(AsyncWebServerRequest *request, JsonVariant &json)
{
  if (!json.is<JsonObject>())
  {
    sendError(request, 400, "Expected JSON object");
    return;
  }

  JsonObject obj = json.as<JsonObject>();
  if (!obj["step"].is<int>() || !obj["dir"].is<int>() || !obj["sleep"].is<int>())
  {
    sendError(request, 400, "Missing motor pins");
    return;
  }

  MotorPins pins;
  pins.step = obj["step"].as<int>();
  pins.dir = obj["dir"].as<int>();
  pins.sleep = obj["sleep"].as<int>();

  uint16_t newId = 0;
  if (!motion.addMotor(pins, &newId))
  {
    sendError(request, 409, "Failed to add motor");
    return;
  }

  auto *response = new AsyncJsonResponse();
  JsonVariant root = response->getRoot();
  writeState(root);
  root.as<JsonObject>()["createdMotorId"] = newId;
  response->setLength();
  request->send(response);
}

void HttpRoutes::handleUpdateMotor(AsyncWebServerRequest *request, JsonVariant &json)
{
  if (!json.is<JsonObject>())
  {
    sendError(request, 400, "Expected JSON object");
    return;
  }

  JsonObject obj = json.as<JsonObject>();
  if (!(obj["motorId"].is<unsigned int>() || obj["motorId"].is<int>()))
  {
    sendError(request, 400, "Missing motorId");
    return;
  }
  if (!obj["step"].is<int>() || !obj["dir"].is<int>() || !obj["sleep"].is<int>())
  {
    sendError(request, 400, "Missing motor pins");
    return;
  }

  uint16_t motorId = static_cast<uint16_t>(obj["motorId"].as<unsigned int>());
  if (!motion.hasMotor(motorId))
  {
    sendError(request, 404, "Unknown motor");
    return;
  }

  MotorPins pins;
  pins.step = obj["step"].as<int>();
  pins.dir = obj["dir"].as<int>();
  pins.sleep = obj["sleep"].as<int>();

  if (!motion.updateMotorPins(motorId, pins))
  {
    sendError(request, 409, "Failed to update motor");
    return;
  }

  sendState(request);
}

void HttpRoutes::handleRemoveMotor(AsyncWebServerRequest *request, JsonVariant &json)
{
  if (!json.is<JsonObject>())
  {
    sendError(request, 400, "Expected JSON object");
    return;
  }

  JsonObject obj = json.as<JsonObject>();
  if (!(obj["motorId"].is<unsigned int>() || obj["motorId"].is<int>()))
  {
    sendError(request, 400, "Missing motorId");
    return;
  }

  uint16_t motorId = static_cast<uint16_t>(obj["motorId"].as<unsigned int>());
  if (!motion.hasMotor(motorId))
  {
    sendError(request, 404, "Unknown motor");
    return;
  }

  if (!motion.removeMotor(motorId))
  {
    sendError(request, 409, "Failed to remove motor");
    return;
  }

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

  auto *driverHandler = new AsyncCallbackJsonWebHandler("/api/driver", [this](AsyncWebServerRequest *request, JsonVariant &json) {
    handleDriver(request, json);
  });
  server.addHandler(driverHandler);

  auto *stopHandler = new AsyncCallbackJsonWebHandler("/api/stop", [this](AsyncWebServerRequest *request, JsonVariant &json) {
    handleStop(request, json);
  });
  server.addHandler(stopHandler);

  auto *resetHandler = new AsyncCallbackJsonWebHandler("/api/reset", [this](AsyncWebServerRequest *request, JsonVariant &json) {
    handleReset(request, json);
  });
  server.addHandler(resetHandler);

  auto *motorsAddHandler = new AsyncCallbackJsonWebHandler("/api/motors/add", [this](AsyncWebServerRequest *request, JsonVariant &json) {
    handleAddMotor(request, json);
  });
  server.addHandler(motorsAddHandler);

  auto *motorsUpdateHandler = new AsyncCallbackJsonWebHandler("/api/motors/update", [this](AsyncWebServerRequest *request, JsonVariant &json) {
    handleUpdateMotor(request, json);
  });
  server.addHandler(motorsUpdateHandler);

  auto *motorsRemoveHandler = new AsyncCallbackJsonWebHandler("/api/motors/remove", [this](AsyncWebServerRequest *request, JsonVariant &json) {
    handleRemoveMotor(request, json);
  });
  server.addHandler(motorsRemoveHandler);

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
}

} // namespace StepperControl
