#include "StepperMotion.h"

namespace
{
  constexpr int PIN_STEP = 12;
  constexpr int PIN_DIR = 14;
  constexpr int PIN_SLEEP = 27;

  constexpr long DEFAULT_STEPS_PER_REV = 2038;
  constexpr float DEFAULT_REVS = 1.0f;
  constexpr float DEFAULT_MAX_SPEED = 800.0f;
  constexpr float DEFAULT_ACCEL = 4000.0f;
  constexpr float MAX_SPEED_LIMIT = 4000.0f;
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

void Motion::updateTarget(TargetUnits units, double value)
{
  config.target = computeTarget(units, value, config.stepsPerRev);
  if (motion.mode == RunMode::Idle)
  {
    motion.segmentSteps = config.target.steps;
  }
}

void Motion::resetLastRun()
{
  lastRun.valid = false;
  lastRun.aborted = false;
  lastRun.startMs = millis();
  lastRun.durationMs = 0;
  lastRun.steps = 0;
  lastRun.loopMaxGapUs = 0;
}

void Motion::applyMotionSettings()
{
  if (!stepper)
  {
    return;
  }

  config.maxSpeed = clampMaxSpeed(config.maxSpeed);
  config.acceleration = clampAcceleration(config.acceleration);

  if (stepper->setSpeedInHz(static_cast<uint32_t>(config.maxSpeed)) != 0)
  {
    Serial.printf("StepperControl: setSpeedInHz failed for %.2f\n", config.maxSpeed);
  }
  if (stepper->setAcceleration(static_cast<int32_t>(config.acceleration)) != 0)
  {
    Serial.printf("StepperControl: setAcceleration failed for %.2f\n", config.acceleration);
  }
}

void Motion::recordLastRun(bool aborted)
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

bool Motion::ensureDriverAwake()
{
  if (!driverAwakeFlag)
  {
    setDriverAwake(true);
  }
  return driverAwakeFlag;
}

bool Motion::beginMove(int direction)
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
  motion.segmentSteps = config.target.steps;
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

void Motion::continuePingPong()
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

void Motion::begin()
{
  loadDefaults();

  engine.init();
  stepper = engine.stepperConnectToPin(PIN_STEP);
  if (!stepper)
  {
    Serial.println("StepperControl: Failed to attach stepper to STEP pin");
    return;
  }

  stepper->setDirectionPin(PIN_DIR);
  stepper->setEnablePin(PIN_SLEEP, false);
  stepper->setAutoEnable(false);
  stepper->disableOutputs();
  driverAwakeFlag = false;

  applyMotionSettings();
}

void Motion::loop()
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
    if (driverAwakeFlag && config.autoSleep)
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

  out.lastRun.valid = lastRun.valid;
  out.lastRun.aborted = lastRun.aborted;
  out.lastRun.startMs = lastRun.startMs;
  out.lastRun.durationMs = lastRun.durationMs;
  out.lastRun.steps = lastRun.steps;
  out.lastRun.loopMaxGapUs = lastRun.loopMaxGapUs;

  out.status.mode = motion.mode;
  out.status.driverAwake = driverAwakeFlag;
  out.status.direction = motion.currentDirection;
  out.status.segmentSteps = motion.segmentSteps;

  if (stepper)
  {
    bool moving = stepper->isRunning();
    out.status.moving = moving;
    long currentPosition = stepper->getCurrentPosition();
    long targetPosition = stepper->targetPos();
    out.status.currentPosition = currentPosition;
    out.status.targetPosition = targetPosition;
    out.status.distanceToGo = targetPosition - currentPosition;
    out.status.speed = static_cast<float>(stepper->getCurrentSpeedInMilliHz(true)) / 1000.0f;
  }

  return out;
}

bool Motion::startSingle(int direction)
{
  if (motion.mode != RunMode::Idle)
  {
    return false;
  }
  if (config.target.steps <= 0)
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

bool Motion::startPingPong(int direction)
{
  if (motion.mode != RunMode::Idle)
  {
    return false;
  }
  if (config.target.steps <= 0)
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

void Motion::stop(bool aborted)
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
  motion.segmentSteps = config.target.steps;
}

void Motion::reset()
{
  stop(false);
  if (stepper)
  {
    stepper->setCurrentPosition(0);
  }

  motion.anchorPosition = 0;
  motion.segmentStartPos = 0;
  motion.segmentStartMs = millis();
  lastRun.valid = false;
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
  return ok;
}

bool Motion::restoreDefaults()
{
  stop(true);
  loadDefaults();
  applyMotionSettings();
  return true;
}

void Motion::setDriverAwake(bool awake)
{
  if (!stepper)
  {
    driverAwakeFlag = false;
    return;
  }

  if (awake)
  {
    if (!driverAwakeFlag)
    {
      stepper->enableOutputs();
      delay(DRIVER_WAKE_DELAY_MS);
      driverAwakeFlag = true;
    }
    return;
  }

  if (!driverAwakeFlag)
  {
    return;
  }

  if (stepper->isRunning() || motion.mode != RunMode::Idle)
  {
    stop(true);
  }

  stepper->disableOutputs();
  driverAwakeFlag = false;
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

  applyMotionSettings();
}

} // namespace StepperControl
