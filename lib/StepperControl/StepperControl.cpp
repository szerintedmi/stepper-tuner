#include "StepperControl.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <FastAccelStepper.h>
#include <LittleFS.h>

namespace StepperControl
{
namespace
{
  constexpr int PIN_STEP = 12;
  constexpr int PIN_DIR = 14;
  constexpr int PIN_SLEEP = 27;

  constexpr long DEFAULT_STEPS_PER_REV = 2038;
  constexpr float DEFAULT_REVS = 1.0f;
  constexpr float DEFAULT_MAX_SPEED = 800.0f;   // steps per second
  constexpr float DEFAULT_ACCEL = 4000.0f;      // steps per second^2
  constexpr float MAX_SPEED_LIMIT = 4000.0f;    // guard rail for UI sliders
  constexpr float MAX_ACCEL = 30000.0f;
  constexpr long MAX_STEPS_PER_REV = 200000;
  constexpr unsigned long DRIVER_WAKE_DELAY_MS = 5;
  constexpr unsigned long AUTO_SLEEP_DELAY_MS = 250;

  enum class TargetUnits
  {
    Revs,
    Steps,
    Degrees
  };

  enum class RunMode
  {
    Idle,
    Single,
    PingPong
  };

  struct StepperConfig
  {
    long stepsPerRev;
    TargetUnits targetUnits;
    float targetRevs;
    float targetDegrees;
    long targetSteps;
    float maxSpeed;
    float acceleration;
    float maxSpeedLimit;
    bool autoSleep;
  };

  struct LastRunInfo
  {
    bool valid;
    bool aborted;
    unsigned long startMs;
    unsigned long durationMs;
    long steps;
    unsigned long loopMaxGapUs;
  };

  struct MotionState
  {
    RunMode mode;
    int currentDirection;
    long anchorPosition;
    long segmentSteps;
    unsigned long segmentStartMs;
    long segmentStartPos;
  };

  FastAccelStepperEngine engine;
  FastAccelStepper *stepper = nullptr;

  StepperConfig config = {
      DEFAULT_STEPS_PER_REV,
      TargetUnits::Revs,
      DEFAULT_REVS,
      DEFAULT_REVS * 360.0f,
      DEFAULT_STEPS_PER_REV,
      DEFAULT_MAX_SPEED,
      DEFAULT_ACCEL,
      MAX_SPEED_LIMIT,
      true};

  MotionState motion = {RunMode::Idle, 1, 0, DEFAULT_STEPS_PER_REV, 0, 0};
  LastRunInfo lastRun = {false, false, 0, 0, 0, 0};

  bool driverAwake = false;
  unsigned long autoSleepRequestMs = 0;

  long roundToLong(double value)
  {
    return value >= 0.0 ? static_cast<long>(value + 0.5) : static_cast<long>(value - 0.5);
  }

  const char *targetUnitsToString(TargetUnits units)
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

  TargetUnits targetUnitsFromString(const char *units)
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

  void setTargetFromRevs(double revs)
  {
    if (revs < 0.0)
    {
      revs = 0.0;
    }
    config.targetUnits = TargetUnits::Revs;
    config.targetRevs = static_cast<float>(revs);
    long steps = roundToLong(revs * static_cast<double>(config.stepsPerRev));
    if (steps < 1)
    {
      steps = 1;
    }
    config.targetSteps = steps;
    config.targetDegrees = config.targetRevs * 360.0f;
  }

  void setTargetFromSteps(long steps)
  {
    if (steps < 1)
    {
      steps = 1;
    }
    config.targetUnits = TargetUnits::Steps;
    config.targetSteps = steps;
    config.targetRevs = static_cast<float>(config.targetSteps) / static_cast<float>(config.stepsPerRev);
    config.targetDegrees = config.targetRevs * 360.0f;
  }

  void setTargetFromDegrees(double degrees)
  {
    if (degrees < 0.0)
    {
      degrees = 0.0;
    }
    config.targetUnits = TargetUnits::Degrees;
    config.targetDegrees = static_cast<float>(degrees);
    double revs = degrees / 360.0;
    config.targetRevs = static_cast<float>(revs);
    long steps = roundToLong(revs * static_cast<double>(config.stepsPerRev));
    if (steps < 1)
    {
      steps = 1;
    }
    config.targetSteps = steps;
  }

  void setStepsPerRev(long steps)
  {
    if (steps < 1)
    {
      steps = 1;
    }
    if (steps > MAX_STEPS_PER_REV)
    {
      steps = MAX_STEPS_PER_REV;
    }
    config.stepsPerRev = steps;

    switch (config.targetUnits)
    {
    case TargetUnits::Steps:
      setTargetFromSteps(config.targetSteps);
      break;
    case TargetUnits::Degrees:
      setTargetFromDegrees(config.targetDegrees);
      break;
    case TargetUnits::Revs:
    default:
      setTargetFromRevs(config.targetRevs);
      break;
    }
  }

  void setMaxSpeedValue(float value)
  {
    if (value < 1.0f)
    {
      value = 1.0f;
    }
    if (value > config.maxSpeedLimit)
    {
      value = config.maxSpeedLimit;
    }
    config.maxSpeed = value;
  }

  void setAccelerationValue(float value)
  {
    if (value < 1.0f)
    {
      value = 1.0f;
    }
    if (value > MAX_ACCEL)
    {
      value = MAX_ACCEL;
    }
    config.acceleration = value;
  }

  void applyMotionSettings()
  {
    if (!stepper)
    {
      return;
    }
    if (config.maxSpeed > config.maxSpeedLimit)
    {
      config.maxSpeed = config.maxSpeedLimit;
    }
    if (config.maxSpeed < 1.0f)
    {
      config.maxSpeed = 1.0f;
    }
    if (config.acceleration < 1.0f)
    {
      config.acceleration = 1.0f;
    }

    if (stepper->setSpeedInHz(static_cast<uint32_t>(config.maxSpeed)) != 0)
    {
      Serial.printf("StepperControl: setSpeedInHz failed for %.2f\n", config.maxSpeed);
    }
    if (stepper->setAcceleration(static_cast<int32_t>(config.acceleration)) != 0)
    {
      Serial.printf("StepperControl: setAcceleration failed for %.2f\n", config.acceleration);
    }
  }

  void recordLastRun(bool aborted)
  {
    if (!stepper)
    {
      return;
    }
    unsigned long now = millis();
    long currentPos = stepper->getCurrentPosition();

    lastRun.valid = true;
    lastRun.aborted = aborted;
    lastRun.startMs = motion.segmentStartMs;
    lastRun.durationMs = (motion.segmentStartMs != 0) ? (now - motion.segmentStartMs) : 0;
    lastRun.steps = currentPos - motion.segmentStartPos;
    lastRun.loopMaxGapUs = 0;

    motion.anchorPosition = currentPos;
    motion.segmentStartPos = currentPos;
    motion.segmentStartMs = now;
  }

  void resetLastRun()
  {
    lastRun.valid = false;
    lastRun.aborted = false;
    lastRun.startMs = millis();
    lastRun.durationMs = 0;
    lastRun.steps = 0;
    lastRun.loopMaxGapUs = 0;
  }

  void stopMotion(bool aborted = true)
  {
    if (!stepper)
    {
      return;
    }

    bool wasRunning = (motion.mode != RunMode::Idle) || stepper->isRunning();
    long currentPos = stepper->getCurrentPosition();
    stepper->forceStopAndNewPosition(currentPos);

    if (wasRunning)
    {
      recordLastRun(aborted);
    }

    motion.mode = RunMode::Idle;
    motion.segmentSteps = config.targetSteps;
  }

  void setDriverAwake(bool awake)
  {
    if (!stepper)
    {
      driverAwake = false;
      return;
    }

    if (awake)
    {
      if (!driverAwake)
      {
        stepper->enableOutputs();
        delay(DRIVER_WAKE_DELAY_MS);
        driverAwake = true;
      }
      return;
    }

    if (!driverAwake)
    {
      return;
    }

    if (stepper->isRunning() || motion.mode != RunMode::Idle)
    {
      stopMotion(true);
    }

    stepper->disableOutputs();
    driverAwake = false;
  }

  bool ensureDriverAwake()
  {
    if (!driverAwake)
    {
      setDriverAwake(true);
    }
    return driverAwake;
  }

  bool beginMove(int direction)
  {
    if (!stepper)
    {
      return false;
    }
    if (!ensureDriverAwake())
    {
      return false;
    }

    motion.currentDirection = (direction >= 0) ? 1 : -1;
    motion.anchorPosition = stepper->getCurrentPosition();
    motion.segmentSteps = config.targetSteps;
    motion.segmentStartPos = motion.anchorPosition;
    motion.segmentStartMs = millis();
    resetLastRun();
    lastRun.startMs = motion.segmentStartMs;

    long target = motion.anchorPosition + motion.currentDirection * motion.segmentSteps;
    MoveResultCode res = stepper->moveTo(target);
    if (res != MOVE_OK)
    {
      Serial.printf("StepperControl: moveTo failed (%d)\n", static_cast<int>(res));
      motion.mode = RunMode::Idle;
      return false;
    }
    return true;
  }

  bool startSingle(int direction)
  {
    if (motion.mode != RunMode::Idle)
    {
      return false;
    }
    if (config.targetSteps < 1)
    {
      return false;
    }
    if (!beginMove(direction))
    {
      return false;
    }

    motion.mode = RunMode::Single;
    return true;
  }

  bool startPingPong(int direction)
  {
    if (motion.mode != RunMode::Idle)
    {
      return false;
    }
    if (config.targetSteps < 1)
    {
      return false;
    }
    if (!beginMove(direction))
    {
      return false;
    }

    motion.mode = RunMode::PingPong;
    return true;
  }

  void continuePingPong()
  {
    if (!stepper || motion.mode != RunMode::PingPong)
    {
      return;
    }

    recordLastRun(false);

    motion.currentDirection = -motion.currentDirection;
    motion.anchorPosition = stepper->getCurrentPosition();
    motion.segmentStartPos = motion.anchorPosition;
    motion.segmentStartMs = millis();
    resetLastRun();
    lastRun.startMs = motion.segmentStartMs;

    long target = motion.anchorPosition + motion.currentDirection * motion.segmentSteps;
    MoveResultCode res = stepper->moveTo(target);
    if (res != MOVE_OK)
    {
      Serial.printf("StepperControl: ping-pong moveTo failed (%d)\n", static_cast<int>(res));
      motion.mode = RunMode::Idle;
    }
  }

  const char *modeToString(RunMode mode)
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

  void fillState(JsonVariant root)
  {
    JsonObject obj = root.to<JsonObject>();
    JsonObject settings = obj["settings"].to<JsonObject>();
    settings["stepsPerRev"] = config.stepsPerRev;
    settings["targetUnits"] = targetUnitsToString(config.targetUnits);
    settings["targetRevs"] = config.targetRevs;
    settings["targetDegrees"] = config.targetDegrees;
    settings["targetSteps"] = config.targetSteps;
    settings["maxSpeed"] = config.maxSpeed;
    settings["maxSpeedLimit"] = config.maxSpeedLimit;
    settings["acceleration"] = config.acceleration;
    settings["autoSleep"] = config.autoSleep;

    JsonObject status = obj["status"].to<JsonObject>();
    bool moving = stepper && stepper->isRunning();
    long currentPosition = stepper ? stepper->getCurrentPosition() : 0;
    long targetPosition = stepper ? stepper->targetPos() : currentPosition;
    long distanceToGo = targetPosition - currentPosition;
    float speed = 0.0f;
    if (stepper)
    {
      speed = static_cast<float>(stepper->getCurrentSpeedInMilliHz(true)) / 1000.0f;
    }

    status["mode"] = modeToString(motion.mode);
    status["moving"] = moving;
    status["driverAwake"] = driverAwake;
    status["direction"] = motion.currentDirection;
    status["segmentSteps"] = motion.segmentSteps;
    status["currentPosition"] = currentPosition;
    status["targetPosition"] = targetPosition;
    status["distanceToGo"] = distanceToGo;
    status["speed"] = speed;

    JsonObject last = status["lastRun"].to<JsonObject>();
    last["valid"] = lastRun.valid;
    last["aborted"] = lastRun.aborted;
    last["startMs"] = lastRun.startMs;
    last["durationMs"] = lastRun.durationMs;
    last["steps"] = lastRun.steps;
    last["loopMaxGapUs"] = lastRun.loopMaxGapUs;
  }

  void sendError(AsyncWebServerRequest *request, int code, const char *message)
  {
    request->send(code, "text/plain", message ? message : "Error");
  }

  void attachRoutes(AsyncWebServer &server)
  {
    if (!(LittleFS.begin(false) || LittleFS.begin(true)))
    {
      Serial.println("StepperControl: LittleFS mount failed");
    }

    server.serveStatic("/", LittleFS, "/")
        .setDefaultFile("index.html")
        .setFilter([](AsyncWebServerRequest *r) {
          const String &u = r->url();
          return !(u.startsWith("/wifi/api/") || u == "/wifi/api");
        });

    server.on("/api/state", HTTP_GET, [](AsyncWebServerRequest *request) {
      AsyncJsonResponse *response = new AsyncJsonResponse();
      fillState(response->getRoot());
      response->setLength();
      request->send(response);
    });

    auto *settingsHandler = new AsyncCallbackJsonWebHandler("/api/settings", [](AsyncWebServerRequest *request, JsonVariant &json) {
      if (!json.is<JsonObject>())
      {
        sendError(request, 400, "Expected JSON object");
        return;
      }

      JsonObject obj = json.as<JsonObject>();

      JsonVariant stepsPerRevVar = obj["stepsPerRev"];
      if (!stepsPerRevVar.isNull())
      {
        setStepsPerRev(stepsPerRevVar.as<long>());
      }

      TargetUnits units = targetUnitsFromString(obj["targetUnits"].is<const char *>() ? obj["targetUnits"].as<const char *>() : targetUnitsToString(config.targetUnits));

      bool hasTargetValue = !obj["targetValue"].isNull() || !obj["targetRevs"].isNull() || !obj["targetSteps"].isNull() || !obj["targetDegrees"].isNull();
      if (!obj["targetValue"].isNull())
      {
        double value = obj["targetValue"].as<double>();
        switch (units)
        {
        case TargetUnits::Steps:
          setTargetFromSteps(roundToLong(value));
          break;
        case TargetUnits::Degrees:
          setTargetFromDegrees(value);
          break;
        case TargetUnits::Revs:
        default:
          setTargetFromRevs(value);
          break;
        }
      }
      else if (hasTargetValue)
      {
        if (!obj["targetSteps"].isNull())
        {
          setTargetFromSteps(obj["targetSteps"].as<long>());
          units = TargetUnits::Steps;
        }
        else if (!obj["targetDegrees"].isNull())
        {
          setTargetFromDegrees(obj["targetDegrees"].as<double>());
          units = TargetUnits::Degrees;
        }
        else if (!obj["targetRevs"].isNull())
        {
          setTargetFromRevs(obj["targetRevs"].as<double>());
          units = TargetUnits::Revs;
        }
      }
      else
      {
        // No new target value supplied, but maybe units changed.
        switch (units)
        {
        case TargetUnits::Steps:
          setTargetFromSteps(config.targetSteps);
          break;
        case TargetUnits::Degrees:
          setTargetFromDegrees(config.targetDegrees);
          break;
        case TargetUnits::Revs:
        default:
          setTargetFromRevs(config.targetRevs);
          break;
        }
      }

      JsonVariant maxSpeedVar = obj["maxSpeed"];
      if (!maxSpeedVar.isNull())
      {
        setMaxSpeedValue(maxSpeedVar.as<float>());
      }
      JsonVariant accelVar = obj["acceleration"];
      if (!accelVar.isNull())
      {
        setAccelerationValue(accelVar.as<float>());
      }
      JsonVariant autoSleepVar = obj["autoSleep"];
      if (autoSleepVar.is<bool>())
      {
        config.autoSleep = autoSleepVar.as<bool>();
      }

      // Persist chosen units after applying values.
      config.targetUnits = units;

      applyMotionSettings();

      AsyncJsonResponse *response = new AsyncJsonResponse();
      fillState(response->getRoot());
      response->setLength();
      request->send(response);
    });
    server.addHandler(settingsHandler);

    auto *runHandler = new AsyncCallbackJsonWebHandler("/api/run", [](AsyncWebServerRequest *request, JsonVariant &json) {
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
        stopMotion(true);
        if (config.autoSleep)
        {
          setDriverAwake(false);
        }
      }
      else if (strcmp(mode, "single") == 0)
      {
        if (!startSingle(direction))
        {
          sendError(request, 409, "Stepper busy or invalid target");
          return;
        }
      }
      else if (strcmp(mode, "pingpong") == 0)
      {
        if (!startPingPong(direction))
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

      AsyncJsonResponse *response = new AsyncJsonResponse();
      fillState(response->getRoot());
      response->setLength();
      request->send(response);
    });
    server.addHandler(runHandler);

    server.on("/api/stop", HTTP_POST, [](AsyncWebServerRequest *request) {
      stopMotion(true);
      if (config.autoSleep)
      {
        setDriverAwake(false);
      }
      AsyncJsonResponse *response = new AsyncJsonResponse();
      fillState(response->getRoot());
      response->setLength();
      request->send(response);
    });

    server.on("/api/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
      stopMotion(false);
      if (stepper)
      {
        stepper->setCurrentPosition(0);
      }
      motion.anchorPosition = 0;
      motion.segmentStartPos = 0;
      motion.segmentStartMs = millis();
      lastRun.valid = false;
      AsyncJsonResponse *response = new AsyncJsonResponse();
      fillState(response->getRoot());
      response->setLength();
      request->send(response);
    });

    auto *driverHandler = new AsyncCallbackJsonWebHandler("/api/driver", [](AsyncWebServerRequest *request, JsonVariant &json) {
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
      setDriverAwake(awake);
      AsyncJsonResponse *response = new AsyncJsonResponse();
      fillState(response->getRoot());
      response->setLength();
      request->send(response);
    });
    server.addHandler(driverHandler);
  }
} // namespace

void begin(AsyncWebServer &server)
{
  Serial.println("StepperControl: begin");

  engine.init();
  stepper = engine.stepperConnectToPin(PIN_STEP);
  if (!stepper)
  {
    Serial.println("StepperControl: Failed to attach stepper to STEP pin");
    return;
  }

  stepper->setDirectionPin(PIN_DIR);
  stepper->setEnablePin(PIN_SLEEP, false); // HIGH keeps the driver awake
  stepper->setAutoEnable(false);
  stepper->disableOutputs();
  driverAwake = false;

  applyMotionSettings();

  attachRoutes(server);
}

void loop()
{
  if (!stepper)
  {
    return;
  }

  bool moving = stepper->isRunning();

  if (motion.mode == RunMode::Single)
  {
    if (!moving)
    {
      recordLastRun(false);
      motion.mode = RunMode::Idle;
    }
  }
  else if (motion.mode == RunMode::PingPong)
  {
    if (!moving)
    {
      continuePingPong();
      moving = stepper->isRunning();
    }
  }

  if (!moving && motion.mode == RunMode::Idle)
  {
    if (driverAwake && config.autoSleep)
    {
      if (autoSleepRequestMs == 0)
      {
        autoSleepRequestMs = millis();
      }
      else if ((millis() - autoSleepRequestMs) >= AUTO_SLEEP_DELAY_MS)
      {
        setDriverAwake(false);
        autoSleepRequestMs = 0;
      }
    }
    else
    {
      autoSleepRequestMs = 0;
    }
  }
  else
  {
    autoSleepRequestMs = 0;
  }
}

} // namespace StepperControl
