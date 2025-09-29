#include "StepperMotor.h"

#include <Arduino.h>

namespace StepperControl
{

StepperMotor::StepperMotor(uint16_t id, const MotorPins &pins)
    : id_(id), pins_(pins)
{
}

bool StepperMotor::attach(FastAccelStepperEngine &engine)
{
  detach();

  stepper_ = engine.stepperConnectToPin(static_cast<uint8_t>(pins_.step));
  if (!stepper_)
  {
    Serial.printf("StepperControl: Failed to attach motor %u to STEP pin %d\n", id_, pins_.step);
    return false;
  }

  stepper_->setDirectionPin(static_cast<uint8_t>(pins_.dir));
  stepper_->setEnablePin(static_cast<uint8_t>(pins_.sleep), false);
  stepper_->setAutoEnable(false);
  stepper_->disableOutputs();

  driverAwake_ = false;
  autoSleepRequestMs_ = 0;
  resetRuntimeState();
  return true;
}

void StepperMotor::detach()
{
  if (stepper_)
  {
    stepper_->disableOutputs();
    stepper_->detachFromPin();
    stepper_ = nullptr;
  }
  driverAwake_ = false;
  autoSleepRequestMs_ = 0;
  resetRuntimeState();
}

void StepperMotor::setDriverAwake(bool awake, unsigned long wakeDelayMs)
{
  if (!stepper_)
  {
    driverAwake_ = false;
    autoSleepRequestMs_ = 0;
    return;
  }

  if (awake)
  {
    if (!driverAwake_)
    {
      stepper_->enableOutputs();
      if (wakeDelayMs > 0)
      {
        delay(wakeDelayMs);
      }
      driverAwake_ = true;
    }
    autoSleepRequestMs_ = 0;
    return;
  }

  if (!driverAwake_)
  {
    return;
  }

  stepper_->disableOutputs();
  driverAwake_ = false;
  autoSleepRequestMs_ = 0;
}

bool StepperMotor::ensureDriverAwake(unsigned long wakeDelayMs)
{
  if (!driverAwake_)
  {
    setDriverAwake(true, wakeDelayMs);
  }
  return driverAwake_;
}

MoveResultCode StepperMotor::moveTo(long targetPosition)
{
  if (!stepper_)
  {
    return MOVE_ERR_ACCELERATION_IS_UNDEFINED;
  }
  return stepper_->moveTo(targetPosition);
}

bool StepperMotor::isRunning() const
{
  return stepper_ && stepper_->isRunning();
}

long StepperMotor::currentPosition() const
{
  return stepper_ ? stepper_->getCurrentPosition() : anchorPosition;
}

long StepperMotor::targetPosition() const
{
  return stepper_ ? stepper_->targetPos() : anchorPosition;
}

float StepperMotor::currentSpeedHz() const
{
  if (!stepper_)
  {
    return 0.0f;
  }
  return static_cast<float>(stepper_->getCurrentSpeedInMilliHz(true)) / 1000.0f;
}

void StepperMotor::setCurrentPosition(long position)
{
  if (stepper_)
  {
    stepper_->setCurrentPosition(position);
  }
  anchorPosition = position;
  segmentStartPos = position;
}

void StepperMotor::forceStopAtCurrentPosition()
{
  if (!stepper_)
  {
    return;
  }
  long currentPos = stepper_->getCurrentPosition();
  stepper_->forceStopAndNewPosition(currentPos);
}

bool StepperMotor::configureMotion(float maxSpeedHz, float acceleration)
{
  if (!stepper_)
  {
    return false;
  }

  bool ok = true;
  if (stepper_->setSpeedInHz(static_cast<uint32_t>(maxSpeedHz)) != 0)
  {
    ok = false;
  }
  if (stepper_->setAcceleration(static_cast<int32_t>(acceleration)) != 0)
  {
    ok = false;
  }
  return ok;
}

void StepperMotor::resetRuntimeState()
{
  mode = RunMode::Idle;
  currentDirection = 1;
  anchorPosition = 0;
  segmentSteps = 0;
  segmentStartPos = 0;
  segmentStartMs = millis();

  lastRun.valid = false;
  lastRun.aborted = false;
  lastRun.startMs = millis();
  lastRun.durationMs = 0;
  lastRun.steps = 0;
  lastRun.loopMaxGapUs = 0;
}

void StepperMotor::beginSegment(unsigned long startMs, long anchorPos, int direction)
{
  currentDirection = (direction >= 0) ? 1 : -1;
  anchorPosition = anchorPos;
  segmentStartPos = anchorPos;
  segmentStartMs = startMs;
  lastRun.valid = false;
  lastRun.aborted = false;
  lastRun.startMs = startMs;
  lastRun.durationMs = 0;
  lastRun.steps = 0;
  lastRun.loopMaxGapUs = 0;
}

void StepperMotor::recordLastRun(bool aborted, unsigned long now, long currentPos)
{
  lastRun.valid = true;
  lastRun.aborted = aborted;
  lastRun.durationMs = (segmentStartMs != 0) ? (now - segmentStartMs) : 0;
  lastRun.steps = currentPos - segmentStartPos;
  lastRun.loopMaxGapUs = 0;
  lastRun.startMs = segmentStartMs;

  anchorPosition = currentPos;
  segmentStartPos = currentPos;
  segmentStartMs = now;
}

} // namespace StepperControl
