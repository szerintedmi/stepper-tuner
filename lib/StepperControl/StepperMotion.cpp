#include "StepperMotion.h"

#include <algorithm>

namespace
{
  constexpr long DEFAULT_STEPS_PER_REV = 2038;
  constexpr float DEFAULT_REVS = 1.0f;
  constexpr float DEFAULT_MAX_SPEED = 800.0f;
  constexpr float DEFAULT_ACCEL = 4000.0f;
  constexpr float MAX_SPEED_LIMIT = 10000.0f;
  constexpr float MAX_ACCEL = 30000.0f;
  constexpr long MAX_STEPS_PER_REV = 200000;
  constexpr unsigned long DRIVER_WAKE_DELAY_MS = 5;
  constexpr unsigned long AUTO_SLEEP_DELAY_MS = 250;
} // namespace

#define STEPPER_PREF_NAMESPACE "motion"
#define STEPPER_PREF_KEY_STEPS "steps"
#define STEPPER_PREF_KEY_SPEED "speed"
#define STEPPER_PREF_KEY_ACCEL "accel"
#define STEPPER_PREF_KEY_SLEEP "sleep"
#define STEPPER_PREF_KEY_MOTORS "motors"
#define STEPPER_PREF_KEY_NEXT_ID "nextId"
#define STEPPER_PREF_KEY_LIMIT_ENABLED "limEn"
#define STEPPER_PREF_KEY_LIMIT_MIN "limMin"
#define STEPPER_PREF_KEY_LIMIT_MAX "limMax"

namespace StepperControl
{

  long Motion::roundToLong(double value)
  {
    return value >= 0.0 ? static_cast<long>(value + 0.5) : static_cast<long>(value - 0.5);
  }

  long Motion::clampStepsPerRev(long steps) const
  {
    if (steps < 1)
    {
      return 1;
    }
    if (steps > MAX_STEPS_PER_REV)
    {
      return MAX_STEPS_PER_REV;
    }
    return steps;
  }

  float Motion::clampMaxSpeed(float value) const
  {
    if (value < 1.0f)
    {
      return 1.0f;
    }
    if (value > config.maxSpeedLimit)
    {
      return config.maxSpeedLimit;
    }
    return value;
  }

  float Motion::clampAcceleration(float value) const
  {
    if (value < 1.0f)
    {
      return 1.0f;
    }
    if (value > MAX_ACCEL)
    {
      return MAX_ACCEL;
    }
    return value;
  }

  bool Motion::ensurePrefs()
  {
    if (prefsOpen)
    {
      return true;
    }
    prefsOpen = prefs.begin(STEPPER_PREF_NAMESPACE, false);
    if (!prefsOpen)
    {
      Serial.println("StepperControl: Preferences begin failed");
    }
    return prefsOpen;
  }

  void Motion::loadDefaults()
  {
    config.maxSpeedLimit = MAX_SPEED_LIMIT;

    long stepsPerRev = DEFAULT_STEPS_PER_REV;
    float maxSpeed = DEFAULT_MAX_SPEED;
    float acceleration = DEFAULT_ACCEL;
    bool autoSleep = true;

    if (ensurePrefs())
    {
      stepsPerRev = clampStepsPerRev(prefs.getLong(STEPPER_PREF_KEY_STEPS, stepsPerRev));
      maxSpeed = clampMaxSpeed(prefs.getFloat(STEPPER_PREF_KEY_SPEED, maxSpeed));
      acceleration = clampAcceleration(prefs.getFloat(STEPPER_PREF_KEY_ACCEL, acceleration));
      autoSleep = prefs.getBool(STEPPER_PREF_KEY_SLEEP, autoSleep);
    }

    config.stepsPerRev = stepsPerRev;
    config.maxSpeed = maxSpeed;
    config.acceleration = acceleration;
    config.autoSleep = autoSleep;

    long defaultHalfSpan = roundToLong(static_cast<double>(config.stepsPerRev) / 2.0);
    long defaultLimitMin = -defaultHalfSpan;
    long defaultLimitMax = defaultHalfSpan;
    bool limitsEnabled = false;
    long limitMin = defaultLimitMin;
    long limitMax = defaultLimitMax;

    if (prefsOpen)
    {
      limitsEnabled = prefs.getBool(STEPPER_PREF_KEY_LIMIT_ENABLED, limitsEnabled);
      limitMin = prefs.getLong(STEPPER_PREF_KEY_LIMIT_MIN, limitMin);
      limitMax = prefs.getLong(STEPPER_PREF_KEY_LIMIT_MAX, limitMax);
    }

    if (limitMin > limitMax)
    {
      limitMin = defaultLimitMin;
      limitMax = defaultLimitMax;
    }

    config.limitsEnabled = limitsEnabled;
    config.limitMin = limitMin;
    config.limitMax = limitMax;

    updateTarget(TargetUnits::Revs, DEFAULT_REVS);
  }

  Motion::TargetState Motion::computeTarget(TargetUnits units, double rawValue, long stepsPerRev) const
  {
    TargetState t;
    t.units = units;

    double revs = 0.0;
    switch (units)
    {
    case TargetUnits::Steps:
      revs = rawValue / static_cast<double>(stepsPerRev);
      break;
    case TargetUnits::Degrees:
      revs = rawValue / 360.0;
      break;
    case TargetUnits::Revs:
    default:
      revs = rawValue;
      break;
    }

    if (revs < 0.0)
    {
      revs = 0.0;
    }

    long steps = roundToLong(revs * static_cast<double>(stepsPerRev));
    if (steps < 0)
    {
      steps = 0;
    }

    float revsFloat = static_cast<float>(revs);
    t.revs = revsFloat;
    t.degrees = revsFloat * 360.0f;
    t.steps = steps;

    switch (units)
    {
    case TargetUnits::Steps:
      t.value = static_cast<double>(steps);
      break;
    case TargetUnits::Degrees:
      t.value = static_cast<double>(t.degrees);
      break;
    case TargetUnits::Revs:
    default:
      t.value = revs;
      break;
    }

    return t;
  }

  MotorManager &Motion::manager()
  {
    if (!motorManager)
    {
      motorManager.reset(new MotorManager(engine, *this));
    }
    return *motorManager;
  }

  const MotorManager &Motion::manager() const
  {
    return *motorManager;
  }

  MotorManager::Motor *Motion::findMotor(uint16_t motorId)
  {
    return motorManager ? motorManager->find(motorId) : nullptr;
  }

  const MotorManager::Motor *Motion::findMotor(uint16_t motorId) const
  {
    return motorManager ? motorManager->find(motorId) : nullptr;
  }

  void Motion::updateTarget(TargetUnits units, double value)
  {
    config.target = computeTarget(units, value, config.stepsPerRev);
    updateIdleSegmentSteps();
  }

  void Motion::updateIdleSegmentSteps()
  {
    for (auto &motor : manager().motors())
    {
      if (motor.motion.mode == RunMode::Idle)
      {
        motor.motion.segmentSteps = config.target.steps;
      }
    }
  }

  bool Motion::applyLimits(MotorManager::Motor &motor, long &targetPosition)
  {
    if (!config.limitsEnabled)
    {
      return false;
    }

    long limited = targetPosition;
    if (limited > config.limitMax)
    {
      limited = config.limitMax;
    }
    if (limited < config.limitMin)
    {
      limited = config.limitMin;
    }

    if (limited == targetPosition)
    {
      return false;
    }

    targetPosition = limited;
    limitEventMotorId = motor.id;
    limitEventSequence++;
    return true;
  }

  void Motion::resetLastRun(MotorManager::Motor &motor)
  {
    motor.lastRun.valid = false;
    motor.lastRun.aborted = false;
    motor.lastRun.startMs = millis();
    motor.lastRun.durationMs = 0;
    motor.lastRun.steps = 0;
    motor.lastRun.loopMaxGapUs = 0;
  }

  void Motion::recordLastRun(MotorManager::Motor &motor, bool aborted)
  {
    if (!motor.stepper)
    {
      return;
    }

    unsigned long now = millis();
    long currentPos = motor.stepper->getCurrentPosition();

    motor.lastRun.valid = true;
    motor.lastRun.aborted = aborted;
    motor.lastRun.startMs = motor.motion.segmentStartMs;
    motor.lastRun.durationMs = (motor.motion.segmentStartMs != 0) ? (now - motor.motion.segmentStartMs) : 0;
    motor.lastRun.steps = currentPos - motor.motion.segmentStartPos;
    motor.lastRun.loopMaxGapUs = 0;

    motor.motion.anchorPosition = currentPos;
    motor.motion.segmentStartPos = currentPos;
    motor.motion.segmentStartMs = now;
  }

  bool Motion::ensureDriverAwake(MotorManager::Motor &motor)
  {
    if (!motor.driverAwake)
    {
      setDriverAwakeForMotor(motor, true);
    }
    return motor.driverAwake;
  }

  void Motion::setDriverAwakeForMotor(MotorManager::Motor &motor, bool awake)
  {
    if (!motor.stepper)
    {
      motor.driverAwake = false;
      motor.autoSleepRequestMs = 0;
      return;
    }

    if (awake)
    {
      if (!motor.driverAwake)
      {
        motor.stepper->enableOutputs();
        delay(DRIVER_WAKE_DELAY_MS);
        motor.driverAwake = true;
      }
      motor.autoSleepRequestMs = 0;
      return;
    }

    if (!motor.driverAwake)
    {
      return;
    }

    stopMotor(motor, true);
    motor.stepper->disableOutputs();
    motor.driverAwake = false;
    motor.autoSleepRequestMs = 0;
  }

  bool Motion::beginMove(MotorManager::Motor &motor, int direction)
  {
    if (!motor.stepper)
    {
      return false;
    }
    if (!ensureDriverAwake(motor))
    {
      return false;
    }

    motor.motion.currentDirection = (direction >= 0) ? 1 : -1;
    motor.motion.anchorPosition = motor.stepper->getCurrentPosition();
    motor.motion.segmentStartPos = motor.motion.anchorPosition;
    motor.motion.segmentStartMs = millis();
    resetLastRun(motor);
    motor.lastRun.startMs = motor.motion.segmentStartMs;

    long requestedSteps = config.target.steps;
    long target = motor.motion.anchorPosition + motor.motion.currentDirection * requestedSteps;
    applyLimits(motor, target);
    long delta = target - motor.motion.anchorPosition;
    if (delta < 0)
    {
      delta = -delta;
    }
    motor.motion.segmentSteps = delta;
    MoveResultCode res = motor.stepper->moveTo(target);
    if (res != MOVE_OK)
    {
      Serial.printf("StepperControl: moveTo failed (%d) for motor %u\n", static_cast<int>(res), motor.id);
      motor.motion.mode = RunMode::Idle;
      return false;
    }
    return true;
  }

  void Motion::continuePingPong(MotorManager::Motor &motor)
  {
    if (!motor.stepper || motor.motion.mode != RunMode::PingPong)
    {
      return;
    }

    recordLastRun(motor, false);

    motor.motion.currentDirection = -motor.motion.currentDirection;
    motor.motion.anchorPosition = motor.stepper->getCurrentPosition();
    motor.motion.segmentStartPos = motor.motion.anchorPosition;
    motor.motion.segmentStartMs = millis();
    resetLastRun(motor);
    motor.lastRun.startMs = motor.motion.segmentStartMs;

    long requestedSteps = config.target.steps;
    long target = motor.motion.anchorPosition + motor.motion.currentDirection * requestedSteps;
    applyLimits(motor, target);
    long delta = target - motor.motion.anchorPosition;
    if (delta < 0)
    {
      delta = -delta;
    }
    motor.motion.segmentSteps = delta;
    if (delta == 0)
    {
      motor.motion.mode = RunMode::Idle;
      return;
    }
    MoveResultCode res = motor.stepper->moveTo(target);
    if (res != MOVE_OK)
    {
      Serial.printf("StepperControl: ping-pong moveTo failed (%d) for motor %u\n", static_cast<int>(res), motor.id);
      motor.motion.mode = RunMode::Idle;
    }
  }

  void Motion::applyMotionSettings(MotorManager::Motor &motor)
  {
    if (!motor.stepper)
    {
      return;
    }

    config.maxSpeed = clampMaxSpeed(config.maxSpeed);
    config.acceleration = clampAcceleration(config.acceleration);

    if (motor.stepper->setSpeedInHz(static_cast<uint32_t>(config.maxSpeed)) != 0)
    {
      Serial.printf("StepperControl: setSpeedInHz failed for motor %u %.2f\n", motor.id, config.maxSpeed);
    }
    if (motor.stepper->setAcceleration(static_cast<int32_t>(config.acceleration)) != 0)
    {
      Serial.printf("StepperControl: setAcceleration failed for motor %u %.2f\n", motor.id, config.acceleration);
    }
  }

  void Motion::applyMotionSettingsAll()
  {
    for (auto &motor : manager().motors())
    {
      applyMotionSettings(motor);
    }
  }

  void Motion::stopMotor(MotorManager::Motor &motor, bool aborted)
  {
    if (!motor.stepper)
    {
      motor.motion.mode = RunMode::Idle;
      motor.motion.segmentSteps = config.target.steps;
      return;
    }

    bool wasRunning = (motor.motion.mode != RunMode::Idle) || motor.stepper->isRunning();
    long currentPos = motor.stepper->getCurrentPosition();
    motor.stepper->forceStopAndNewPosition(currentPos);

    if (wasRunning)
    {
      recordLastRun(motor, aborted);
    }

    motor.motion.mode = RunMode::Idle;
    motor.motion.segmentSteps = config.target.steps;
  }

  void Motion::resetMotor(MotorManager::Motor &motor)
  {
    stopMotor(motor, false);
    if (motor.stepper)
    {
      motor.stepper->setCurrentPosition(0);
    }

    motor.motion.anchorPosition = 0;
    motor.motion.segmentStartPos = 0;
    motor.motion.segmentStartMs = millis();
    motor.lastRun.valid = false;
  }

  void Motion::persistMotors()
  {
    if (!ensurePrefs())
    {
      return;
    }

    String encoded;
    for (const auto &motor : manager().motors())
    {
      if (encoded.length() > 0)
      {
        encoded += ';';
      }
      encoded += String(motor.id);
      encoded += ',';
      encoded += String(motor.pins.step);
      encoded += ',';
      encoded += String(motor.pins.dir);
      encoded += ',';
      encoded += String(motor.pins.sleep);
    }

    prefs.putString(STEPPER_PREF_KEY_MOTORS, encoded);
    prefs.putUInt(STEPPER_PREF_KEY_NEXT_ID, nextMotorId);
  }

  void Motion::loadMotorsFromPrefs()
  {
    manager().motors().clear();

    if (!ensurePrefs())
    {
      return;
    }

    String encoded = prefs.getString(STEPPER_PREF_KEY_MOTORS, "");
    uint32_t storedNextId = prefs.getUInt(STEPPER_PREF_KEY_NEXT_ID, 1);
    nextMotorId = storedNextId == 0 ? 1 : static_cast<uint16_t>(storedNextId);

    uint16_t maxIdSeen = 0;
    int start = 0;
    while (start < encoded.length())
    {
      int end = encoded.indexOf(';', start);
      if (end < 0)
      {
        end = encoded.length();
      }
      String entry = encoded.substring(start, end);
      entry.trim();
      start = end + 1;
      if (entry.length() == 0)
      {
        continue;
      }

      int first = entry.indexOf(',');
      int second = entry.indexOf(',', first + 1);
      int third = entry.indexOf(',', second + 1);
      if (first <= 0 || second <= first || third <= second)
      {
        Serial.printf("StepperControl: Ignoring malformed motor entry '%s'\n", entry.c_str());
        continue;
      }

      long id = entry.substring(0, first).toInt();
      int stepPin = entry.substring(first + 1, second).toInt();
      int dirPin = entry.substring(second + 1, third).toInt();
      int sleepPin = entry.substring(third + 1).toInt();

      MotorManager::Motor motor;
      motor.id = static_cast<uint16_t>(id);
      motor.pins.step = stepPin;
      motor.pins.dir = dirPin;
      motor.pins.sleep = sleepPin;

      manager().motors().push_back(motor);
      if (motor.id > maxIdSeen)
      {
        maxIdSeen = motor.id;
      }
    }

    if (maxIdSeen >= nextMotorId)
    {
      nextMotorId = maxIdSeen + 1;
      if (nextMotorId == 0)
      {
        nextMotorId = maxIdSeen; // avoid wrap
      }
    }

    if (!manager().motors().empty())
    {
      if (!manager().rebuildHardware())
      {
        Serial.println("StepperControl: Failed to rebuild motors from prefs");
      }
      else
      {
        applyMotionSettingsAll();
        updateIdleSegmentSteps();
      }
    }
    else
    {
      manager().detachAll();
    }
  }

  void Motion::begin()
  {
    loadDefaults();

    if (motorManager)
    {
      motorManager->detachAll();
    }
    motorManager.reset(new MotorManager(engine, *this));

    engine.init();
    loadMotorsFromPrefs();
    applyMotionSettingsAll();
    updateIdleSegmentSteps();
  }

  void Motion::loop()
  {
    unsigned long now = millis();
    for (auto &motor : manager().motors())
    {
      if (!motor.stepper)
      {
        continue;
      }

      bool moving = motor.stepper->isRunning();

      if (motor.motion.mode == RunMode::Single)
      {
        if (!moving)
        {
          recordLastRun(motor, false);
          motor.motion.mode = RunMode::Idle;
        }
      }
      else if (motor.motion.mode == RunMode::PingPong)
      {
        if (!moving)
        {
          continuePingPong(motor);
          moving = motor.stepper->isRunning();
        }
      }

      if (!moving && motor.motion.mode == RunMode::Idle)
      {
        if (motor.driverAwake && config.autoSleep)
        {
          if (motor.autoSleepRequestMs == 0)
          {
            motor.autoSleepRequestMs = now;
          }
          else if ((now - motor.autoSleepRequestMs) >= AUTO_SLEEP_DELAY_MS)
          {
            setDriverAwakeForMotor(motor, false);
            motor.autoSleepRequestMs = 0;
          }
        }
        else
        {
          motor.autoSleepRequestMs = 0;
        }
      }
      else
      {
        motor.autoSleepRequestMs = 0;
      }
    }
  }

  StepperState Motion::state() const
  {
    StepperState out;
    out.settings.stepsPerRev = config.stepsPerRev;
    out.settings.targetUnits = config.target.units;
    out.settings.targetRevs = config.target.revs;
    out.settings.targetDegrees = config.target.degrees;
    out.settings.targetSteps = config.target.steps;
    out.settings.maxSpeed = config.maxSpeed;
    out.settings.maxSpeedLimit = config.maxSpeedLimit;
    out.settings.acceleration = config.acceleration;
    out.settings.autoSleep = config.autoSleep;
    out.settings.limitsEnabled = config.limitsEnabled;
    out.settings.limitMin = config.limitMin;
    out.settings.limitMax = config.limitMax;

    if (!motorManager)
    {
      out.limit.triggered = limitEventSequence != lastReportedLimitEventSequence;
      out.limit.sequence = limitEventSequence;
      out.limit.motorId = limitEventMotorId;
      lastReportedLimitEventSequence = limitEventSequence;
      return out;
    }

    const auto &list = motorManager->motors();
    out.motors.reserve(list.size());
    for (const auto &motor : list)
    {
      StepperState::Motor report;
      report.id = motor.id;
      report.pins.step = motor.pins.step;
      report.pins.dir = motor.pins.dir;
      report.pins.sleep = motor.pins.sleep;
      report.mode = motor.motion.mode;
      report.driverAwake = motor.driverAwake;
      report.direction = motor.motion.currentDirection;
      report.segmentSteps = motor.motion.segmentSteps;

      if (motor.stepper)
      {
        bool moving = motor.stepper->isRunning();
        report.moving = moving;
        long currentPosition = motor.stepper->getCurrentPosition();
        long targetPosition = motor.stepper->targetPos();
        report.currentPosition = currentPosition;
        report.targetPosition = targetPosition;
        report.distanceToGo = targetPosition - currentPosition;
        report.speed = static_cast<float>(motor.stepper->getCurrentSpeedInMilliHz(true)) / 1000.0f;
      }

      report.lastRun.valid = motor.lastRun.valid;
      report.lastRun.aborted = motor.lastRun.aborted;
      report.lastRun.startMs = motor.lastRun.startMs;
      report.lastRun.durationMs = motor.lastRun.durationMs;
      report.lastRun.steps = motor.lastRun.steps;
      report.lastRun.loopMaxGapUs = motor.lastRun.loopMaxGapUs;

      out.motors.push_back(report);
    }

    out.limit.triggered = limitEventSequence != lastReportedLimitEventSequence;
    out.limit.sequence = limitEventSequence;
    out.limit.motorId = limitEventMotorId;
    lastReportedLimitEventSequence = limitEventSequence;

    return out;
  }

  bool Motion::addMotor(const MotorPins &pins, uint16_t *newMotorId)
  {
    if (pins.step < 0 || pins.dir < 0 || pins.sleep < 0)
    {
      Serial.println("StepperControl: Invalid motor pins provided");
      return false;
    }

    for (const auto &existing : manager().motors())
    {
      if (existing.pins.step == pins.step)
      {
        Serial.printf("StepperControl: STEP pin %d already used by motor %u\n", pins.step, existing.id);
        return false;
      }
    }

    uint16_t newId = nextMotorId == 0 ? 1 : nextMotorId;
    MotorManager::Motor *motor = manager().addMotor(newId, pins);
    if (!motor)
    {
      return false;
    }

    if (!manager().rebuildHardware())
    {
      manager().removeMotor(newId);
      Serial.println("StepperControl: Failed to rebuild motors after add");
      return false;
    }

    updateIdleSegmentSteps();

    if (newMotorId)
    {
      *newMotorId = newId;
    }

    if (nextMotorId < 0xFFFF)
    {
      ++nextMotorId;
    }
    persistMotors();
    return true;
  }

  bool Motion::hasMotor(uint16_t motorId) const
  {
    return findMotor(motorId) != nullptr;
  }

  bool Motion::removeMotor(uint16_t motorId)
  {
    MotorManager::Motor *motor = findMotor(motorId);
    if (!motor)
    {
      return false;
    }

    stopMotor(*motor, true);
    if (!manager().removeMotor(motorId))
    {
      return false;
    }

    if (!manager().rebuildHardware())
    {
      Serial.println("StepperControl: Failed to rebuild motors after remove");
    }
    updateIdleSegmentSteps();
    persistMotors();
    return true;
  }

  bool Motion::updateMotorPins(uint16_t motorId, const MotorPins &pins)
  {
    MotorManager::Motor *motor = findMotor(motorId);
    if (!motor)
    {
      return false;
    }

    if (motor->pins.step == pins.step && motor->pins.dir == pins.dir && motor->pins.sleep == pins.sleep)
    {
      return true;
    }

    for (const auto &existing : manager().motors())
    {
      if (existing.id != motorId && existing.pins.step == pins.step)
      {
        Serial.printf("StepperControl: STEP pin %d already used by motor %u\n", pins.step, existing.id);
        return false;
      }
    }

    motor->pins = pins;
    if (!manager().rebuildHardware())
    {
      Serial.println("StepperControl: Failed to rebuild motors after pin update");
      return false;
    }
    updateIdleSegmentSteps();
    persistMotors();
    return true;
  }

  bool Motion::startSingle(uint16_t motorId, int direction)
  {
    MotorManager::Motor *motor = findMotor(motorId);
    if (!motor)
    {
      return false;
    }
    if (motor->motion.mode != RunMode::Idle)
    {
      return false;
    }
    if (config.target.steps <= 0)
    {
      return false;
    }
    if (!beginMove(*motor, direction))
    {
      return false;
    }

    motor->motion.mode = RunMode::Single;
    return true;
  }

  bool Motion::startPingPong(uint16_t motorId, int direction)
  {
    MotorManager::Motor *motor = findMotor(motorId);
    if (!motor)
    {
      return false;
    }
    if (motor->motion.mode != RunMode::Idle)
    {
      return false;
    }
    if (config.target.steps <= 0)
    {
      return false;
    }
    if (!beginMove(*motor, direction))
    {
      return false;
    }

    motor->motion.mode = RunMode::PingPong;
    return true;
  }

  void Motion::stop(uint16_t motorId, bool aborted)
  {
    MotorManager::Motor *motor = findMotor(motorId);
    if (!motor)
    {
      return;
    }

    stopMotor(*motor, aborted);
  }

  void Motion::stopAll(bool aborted)
  {
    for (auto &motor : manager().motors())
    {
      stopMotor(motor, aborted);
    }
  }

  void Motion::reset(uint16_t motorId)
  {
    MotorManager::Motor *motor = findMotor(motorId);
    if (!motor)
    {
      return;
    }
    resetMotor(*motor);
  }

  void Motion::resetAll()
  {
    for (auto &motor : manager().motors())
    {
      resetMotor(motor);
    }
  }

  bool Motion::saveDefaults()
  {
    if (!ensurePrefs())
    {
      return false;
    }

    bool ok = true;
    ok &= prefs.putLong(STEPPER_PREF_KEY_STEPS, config.stepsPerRev) > 0;
    ok &= prefs.putFloat(STEPPER_PREF_KEY_SPEED, config.maxSpeed) > 0;
    ok &= prefs.putFloat(STEPPER_PREF_KEY_ACCEL, config.acceleration) > 0;
    ok &= prefs.putBool(STEPPER_PREF_KEY_SLEEP, config.autoSleep) > 0;
    ok &= prefs.putBool(STEPPER_PREF_KEY_LIMIT_ENABLED, config.limitsEnabled) > 0;
    ok &= prefs.putLong(STEPPER_PREF_KEY_LIMIT_MIN, config.limitMin) > 0;
    ok &= prefs.putLong(STEPPER_PREF_KEY_LIMIT_MAX, config.limitMax) > 0;
    persistMotors();
    return ok;
  }

  bool Motion::restoreDefaults()
  {
    stopAll(true);
    loadDefaults();
    applyMotionSettingsAll();
    updateIdleSegmentSteps();
    return true;
  }

  void Motion::setDriverAwake(uint16_t motorId, bool awake)
  {
    MotorManager::Motor *motor = findMotor(motorId);
    if (!motor)
    {
      return;
    }
    setDriverAwakeForMotor(*motor, awake);
  }

  void Motion::setDriverAwakeAll(bool awake)
  {
    for (auto &motor : manager().motors())
    {
      setDriverAwakeForMotor(motor, awake);
    }
  }

  bool Motion::driverAwake(uint16_t motorId) const
  {
    const MotorManager::Motor *motor = findMotor(motorId);
    return motor ? motor->driverAwake : false;
  }

  void Motion::applySettings(const SettingsPatch &patch)
  {
    long nextStepsPerRev = config.stepsPerRev;
    if (patch.hasStepsPerRev)
    {
      nextStepsPerRev = clampStepsPerRev(patch.stepsPerRev);
    }

    TargetUnits nextUnits = config.target.units;
    double nextValue = config.target.value;

    if (patch.hasTargetUnits)
    {
      nextUnits = patch.targetUnits;
      if (!patch.hasTargetValue)
      {
        switch (nextUnits)
        {
        case TargetUnits::Steps:
          nextValue = static_cast<double>(config.target.steps);
          break;
        case TargetUnits::Degrees:
          nextValue = config.target.degrees;
          break;
        case TargetUnits::Revs:
        default:
          nextValue = config.target.revs;
          break;
        }
      }
    }

    if (patch.hasTargetValue)
    {
      nextValue = patch.targetValue;
    }

    config.stepsPerRev = nextStepsPerRev;
    updateTarget(nextUnits, nextValue);

    if (patch.hasMaxSpeed)
    {
      config.maxSpeed = clampMaxSpeed(patch.maxSpeed);
    }
    if (patch.hasAcceleration)
    {
      config.acceleration = clampAcceleration(patch.acceleration);
    }
    if (patch.hasAutoSleep)
    {
      config.autoSleep = patch.autoSleep;
    }

    bool nextLimitsEnabled = config.limitsEnabled;
    long nextLimitMin = config.limitMin;
    long nextLimitMax = config.limitMax;

    if (patch.hasLimitsEnabled)
    {
      nextLimitsEnabled = patch.limitsEnabled;
    }

    if (patch.hasLimitMin)
    {
      nextLimitMin = patch.limitMin;
    }
    if (patch.hasLimitMax)
    {
      nextLimitMax = patch.limitMax;
    }

    if (patch.hasLimitsEnabled && patch.limitsEnabled && !patch.hasLimitMin && !patch.hasLimitMax)
    {
      long halfSpan = roundToLong(static_cast<double>(config.stepsPerRev) / 2.0);
      nextLimitMin = -halfSpan;
      nextLimitMax = halfSpan;
    }

    if (nextLimitMin > nextLimitMax)
    {
      std::swap(nextLimitMin, nextLimitMax);
    }

    config.limitsEnabled = nextLimitsEnabled;
    config.limitMin = nextLimitMin;
    config.limitMax = nextLimitMax;

    applyMotionSettingsAll();
  }

} // namespace StepperControl
