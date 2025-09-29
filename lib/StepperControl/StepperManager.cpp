#include "StepperManager.h"

#include <Arduino.h>
#include <algorithm>
#include <driver/pcnt.h>

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

  long StepperManager::roundToLong(double value)
  {
    return value >= 0.0 ? static_cast<long>(value + 0.5) : static_cast<long>(value - 0.5);
  }

  long StepperManager::clampStepsPerRev(long steps) const
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

  float StepperManager::clampMaxSpeed(float value) const
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

  float StepperManager::clampAcceleration(float value) const
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

  StepperManager::TargetState StepperManager::computeTarget(TargetUnits units, double rawValue, long stepsPerRev) const
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

  void StepperManager::updateTarget(TargetUnits units, double value)
  {
    config.target = computeTarget(units, value, config.stepsPerRev);
    updateIdleSegmentSteps();
  }

  void StepperManager::updateIdleSegmentSteps()
  {
    for (auto &motor : motors)
    {
      if (motor.mode == RunMode::Idle)
      {
        motor.setSegmentSteps(config.target.steps);
      }
    }
  }

  StepperMotor *StepperManager::findMotor(uint16_t motorId)
  {
    for (auto &motor : motors)
    {
      if (motor.id() == motorId)
      {
        return &motor;
      }
    }
    return nullptr;
  }

  const StepperMotor *StepperManager::findMotor(uint16_t motorId) const
  {
    for (const auto &motor : motors)
    {
      if (motor.id() == motorId)
      {
        return &motor;
      }
    }
    return nullptr;
  }

  bool StepperManager::applyLimits(StepperMotor &motor, long &targetPosition)
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
    limitEventMotorId = motor.id();
    limitEventSequence++;
    return true;
  }

  bool StepperManager::ensurePrefs()
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

  void StepperManager::loadDefaults()
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

  void StepperManager::persistMotors()
  {
    if (!ensurePrefs())
    {
      return;
    }

    String encoded;
    for (const auto &motor : motors)
    {
      if (encoded.length() > 0)
      {
        encoded += ';';
      }
      encoded += String(motor.id());
      encoded += ',';
      encoded += String(motor.pins().step);
      encoded += ',';
      encoded += String(motor.pins().dir);
      encoded += ',';
      encoded += String(motor.pins().sleep);
    }

    prefs.putString(STEPPER_PREF_KEY_MOTORS, encoded);
    prefs.putUInt(STEPPER_PREF_KEY_NEXT_ID, nextMotorId);
  }

  void StepperManager::loadMotorsFromPrefs()
  {
    motors.clear();

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
      MotorPins pins;
      pins.step = entry.substring(first + 1, second).toInt();
      pins.dir = entry.substring(second + 1, third).toInt();
      pins.sleep = entry.substring(third + 1).toInt();

      motors.emplace_back(static_cast<uint16_t>(id), pins);
      if (id > maxIdSeen)
      {
        maxIdSeen = static_cast<uint16_t>(id);
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

    if (!motors.empty())
    {
      if (!rebuildHardware())
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
      detachAll();
    }
  }

  bool StepperManager::rebuildHardware()
  {
    detachAll();
    pcnt_isr_service_uninstall();
    engine.init();

    bool ok = true;
    for (auto &motor : motors)
    {
      if (!motor.attach(engine))
      {
        ok = false;
        break;
      }
      motor.resetRuntimeState();
      motor.setSegmentSteps(config.target.steps);
    }

    if (!ok)
    {
      detachAll();
    }

    return ok;
  }

  void StepperManager::detachAll()
  {
    for (auto &motor : motors)
    {
      motor.detach();
    }
  }

  void StepperManager::applyMotionSettings(StepperMotor &motor)
  {
    config.maxSpeed = clampMaxSpeed(config.maxSpeed);
    config.acceleration = clampAcceleration(config.acceleration);

    if (!motor.configureMotion(config.maxSpeed, config.acceleration))
    {
      Serial.printf("StepperControl: Failed to apply motion config for motor %u\n", motor.id());
    }
  }

  void StepperManager::applyMotionSettingsAll()
  {
    for (auto &motor : motors)
    {
      applyMotionSettings(motor);
    }
  }

  bool StepperManager::beginMove(StepperMotor &motor, int direction, RunMode mode)
  {
    if (!motor.isAttached())
    {
      return false;
    }
    if (!motor.ensureDriverAwake(DRIVER_WAKE_DELAY_MS))
    {
      return false;
    }

    unsigned long now = millis();
    long anchor = motor.getCurrentPosition();
    motor.beginSegment(now, anchor, direction);

    long requestedSteps = config.target.steps;
    long target = anchor + motor.currentDirection * requestedSteps;
    applyLimits(motor, target);

    long delta = target - anchor;
    if (delta < 0)
    {
      delta = -delta;
    }
    motor.setSegmentSteps(delta);

    if (delta == 0)
    {
      motor.mode = RunMode::Idle;
      return false;
    }

    MoveResultCode res = motor.moveTo(target);
    if (res != MOVE_OK)
    {
      Serial.printf("StepperControl: moveTo failed (%d) for motor %u\n", static_cast<int>(res), motor.id());
      motor.mode = RunMode::Idle;
      return false;
    }

    motor.mode = mode;
    return true;
  }

  void StepperManager::continuePingPong(StepperMotor &motor)
  {
    if (!motor.isAttached() || motor.mode != RunMode::PingPong)
    {
      return;
    }

    long currentPos = motor.getCurrentPosition();
    motor.recordLastRun(false, millis(), currentPos);

    unsigned long now = millis();
    long anchor = currentPos;
    int nextDirection = -motor.currentDirection;
    motor.beginSegment(now, anchor, nextDirection);

    long requestedSteps = config.target.steps;
    long target = anchor + motor.currentDirection * requestedSteps;
    applyLimits(motor, target);
    long delta = target - anchor;
    if (delta < 0)
    {
      delta = -delta;
    }
    motor.setSegmentSteps(delta);
    if (delta == 0)
    {
      motor.mode = RunMode::Idle;
      return;
    }

    MoveResultCode res = motor.moveTo(target);
    if (res != MOVE_OK)
    {
      Serial.printf("StepperControl: ping-pong moveTo failed (%d) for motor %u\n", static_cast<int>(res), motor.id());
      motor.mode = RunMode::Idle;
    }
  }

  void StepperManager::stopMotor(StepperMotor &motor, bool aborted)
  {
    if (!motor.isAttached())
    {
      motor.mode = RunMode::Idle;
      motor.setSegmentSteps(config.target.steps);
      return;
    }

    bool wasRunning = (motor.mode != RunMode::Idle) || motor.isRunning();
    motor.forceStopAtCurrentPosition();

    if (wasRunning)
    {
      motor.recordLastRun(aborted, millis(), motor.getCurrentPosition());
    }

    motor.mode = RunMode::Idle;
    motor.setSegmentSteps(config.target.steps);
  }

  void StepperManager::resetMotor(StepperMotor &motor)
  {
    stopMotor(motor, false);
    motor.setCurrentPosition(0);
    motor.resetRuntimeState();
    motor.setSegmentSteps(config.target.steps);
  }

  void StepperManager::begin()
  {
    loadDefaults();
    detachAll();
    engine.init();
    loadMotorsFromPrefs();
    applyMotionSettingsAll();
    updateIdleSegmentSteps();
  }

  void StepperManager::loop()
  {
    unsigned long now = millis();
    for (auto &motor : motors)
    {
      if (!motor.isAttached())
      {
        continue;
      }

      bool moving = motor.isRunning();

      if (motor.mode == RunMode::Single)
      {
        if (!moving)
        {
          motor.recordLastRun(false, now, motor.getCurrentPosition());
          motor.mode = RunMode::Idle;
          motor.setSegmentSteps(config.target.steps);
        }
      }
      else if (motor.mode == RunMode::PingPong)
      {
        if (!moving)
        {
          continuePingPong(motor);
          moving = motor.isRunning();
        }
      }

      if (!moving && motor.mode == RunMode::Idle)
      {
        if (motor.driverAwake() && config.autoSleep)
        {
          if (motor.autoSleepRequestMs() == 0)
          {
            motor.startAutoSleepTimer(now);
          }
          else if ((now - motor.autoSleepRequestMs()) >= AUTO_SLEEP_DELAY_MS)
          {
            setDriverAwake(motor.id(), false);
            motor.clearAutoSleepTimer();
          }
        }
        else
        {
          motor.clearAutoSleepTimer();
        }
      }
      else
      {
        motor.clearAutoSleepTimer();
      }
    }
  }

  StepperState StepperManager::state() const
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

    out.limit.triggered = limitEventSequence != lastReportedLimitEventSequence;
    out.limit.sequence = limitEventSequence;
    out.limit.motorId = limitEventMotorId;
    lastReportedLimitEventSequence = limitEventSequence;

    out.motors.reserve(motors.size());
    for (const auto &motor : motors)
    {
      StepperState::Motor report;
      report.id = motor.id();
      report.pins = motor.pins();
      report.mode = motor.mode;
      report.driverAwake = motor.driverAwake();
      report.direction = motor.currentDirection;
      report.segmentSteps = motor.segmentSteps;

      if (motor.isAttached())
      {
        bool running = motor.isRunning();
        report.moving = running;
        long currentPosition = motor.getCurrentPosition();
        long targetPosition = motor.getTargetPosition();
        report.currentPosition = currentPosition;
        report.targetPosition = targetPosition;
        report.distanceToGo = targetPosition - currentPosition;
        report.speed = motor.getCurrentSpeedHz();
      }

      report.lastRun.valid = motor.lastRun.valid;
      report.lastRun.aborted = motor.lastRun.aborted;
      report.lastRun.startMs = motor.lastRun.startMs;
      report.lastRun.durationMs = motor.lastRun.durationMs;
      report.lastRun.steps = motor.lastRun.steps;
      report.lastRun.loopMaxGapUs = motor.lastRun.loopMaxGapUs;

      out.motors.push_back(report);
    }

    return out;
  }

  bool StepperManager::addMotor(const MotorPins &pins, uint16_t *newMotorId)
  {
    if (pins.step < 0 || pins.dir < 0 || pins.sleep < 0)
    {
      Serial.println("StepperControl: Invalid motor pins provided");
      return false;
    }

    for (const auto &existing : motors)
    {
      if (existing.pins().step == pins.step)
      {
        Serial.printf("StepperControl: STEP pin %d already used by motor %u\n", pins.step, existing.id());
        return false;
      }
    }

    uint16_t newId = nextMotorId == 0 ? 1 : nextMotorId;
    motors.emplace_back(newId, pins);

    if (!rebuildHardware())
    {
      motors.pop_back();
      Serial.println("StepperControl: Failed to rebuild motors after add");
      return false;
    }

    applyMotionSettingsAll();
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

  bool StepperManager::hasMotor(uint16_t motorId) const
  {
    return findMotor(motorId) != nullptr;
  }

  bool StepperManager::removeMotor(uint16_t motorId)
  {
    for (auto it = motors.begin(); it != motors.end(); ++it)
    {
      if (it->id() != motorId)
      {
        continue;
      }

      stopMotor(*it, true);
      motors.erase(it);

      if (!rebuildHardware())
      {
        Serial.println("StepperControl: Failed to rebuild motors after remove");
      }

      updateIdleSegmentSteps();
      persistMotors();
      return true;
    }
    return false;
  }

  bool StepperManager::updateMotorPins(uint16_t motorId, const MotorPins &pins)
  {
    for (const auto &existing : motors)
    {
      if (existing.id() != motorId && existing.pins().step == pins.step)
      {
        Serial.printf("StepperControl: STEP pin %d already used by motor %u\n", pins.step, existing.id());
        return false;
      }
    }

    StepperMotor *motor = findMotor(motorId);
    if (!motor)
    {
      return false;
    }

    if (motor->pins().step == pins.step && motor->pins().dir == pins.dir && motor->pins().sleep == pins.sleep)
    {
      return true;
    }

    motor->setPins(pins);
    if (!rebuildHardware())
    {
      Serial.println("StepperControl: Failed to rebuild motors after pin update");
      return false;
    }

    applyMotionSettingsAll();
    updateIdleSegmentSteps();
    persistMotors();
    return true;
  }

  bool StepperManager::startSingle(uint16_t motorId, int direction)
  {
    StepperMotor *motor = findMotor(motorId);
    if (!motor)
    {
      return false;
    }
    if (motor->mode != RunMode::Idle)
    {
      return false;
    }
    if (config.target.steps <= 0)
    {
      return false;
    }
    return beginMove(*motor, direction, RunMode::Single);
  }

  bool StepperManager::startPingPong(uint16_t motorId, int direction)
  {
    StepperMotor *motor = findMotor(motorId);
    if (!motor)
    {
      return false;
    }
    if (motor->mode != RunMode::Idle)
    {
      return false;
    }
    if (config.target.steps <= 0)
    {
      return false;
    }
    return beginMove(*motor, direction, RunMode::PingPong);
  }

  void StepperManager::stop(uint16_t motorId, bool aborted)
  {
    StepperMotor *motor = findMotor(motorId);
    if (!motor)
    {
      return;
    }
    stopMotor(*motor, aborted);
  }

  void StepperManager::stopAll(bool aborted)
  {
    for (auto &motor : motors)
    {
      stopMotor(motor, aborted);
    }
  }

  void StepperManager::reset(uint16_t motorId)
  {
    StepperMotor *motor = findMotor(motorId);
    if (!motor)
    {
      return;
    }
    resetMotor(*motor);
  }

  void StepperManager::resetAll()
  {
    for (auto &motor : motors)
    {
      resetMotor(motor);
    }
  }

  bool StepperManager::saveDefaults()
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

  bool StepperManager::restoreDefaults()
  {
    stopAll(true);
    loadDefaults();
    applyMotionSettingsAll();
    updateIdleSegmentSteps();
    return true;
  }

  void StepperManager::setDriverAwake(uint16_t motorId, bool awake)
  {
    StepperMotor *motor = findMotor(motorId);
    if (!motor)
    {
      return;
    }

    if (awake)
    {
      motor->setDriverAwake(true, DRIVER_WAKE_DELAY_MS);
      return;
    }

    stopMotor(*motor, true);
    motor->setDriverAwake(false, 0);
  }

  void StepperManager::setDriverAwakeAll(bool awake)
  {
    for (auto &motor : motors)
    {
      if (awake)
      {
        motor.setDriverAwake(true, DRIVER_WAKE_DELAY_MS);
      }
      else
      {
        stopMotor(motor, true);
        motor.setDriverAwake(false, 0);
      }
    }
  }

  bool StepperManager::isDriverAwake(uint16_t motorId) const
  {
    const StepperMotor *motor = findMotor(motorId);
    return motor ? motor->driverAwake() : false;
  }

  void StepperManager::applySettings(const SettingsPatch &patch)
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
