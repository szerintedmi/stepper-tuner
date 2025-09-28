#include "MotorManager.h"

#include <Arduino.h>
#include <driver/pcnt.h>

#include "StepperMotion.h"

namespace StepperControl
{

MotorManager::MotorManager(FastAccelStepperEngine &engineRef, Motion &owner)
    : engine(engineRef), motion(owner)
{
}

std::vector<MotorManager::Motor> &MotorManager::motors()
{
  return storage;
}

const std::vector<MotorManager::Motor> &MotorManager::motors() const
{
  return storage;
}

MotorManager::Motor *MotorManager::find(uint16_t motorId)
{
  for (auto &motor : storage)
  {
    if (motor.id == motorId)
    {
      return &motor;
    }
  }
  return nullptr;
}

const MotorManager::Motor *MotorManager::find(uint16_t motorId) const
{
  for (const auto &motor : storage)
  {
    if (motor.id == motorId)
    {
      return &motor;
    }
  }
  return nullptr;
}

MotorManager::Motor *MotorManager::addMotor(uint16_t id, const MotorPins &pins)
{
  Motor motor;
  motor.id = id;
  motor.pins = pins;
  storage.push_back(motor);
  return &storage.back();
}

bool MotorManager::removeMotor(uint16_t motorId)
{
  for (auto it = storage.begin(); it != storage.end(); ++it)
  {
    if (it->id == motorId)
    {
      storage.erase(it);
      return true;
    }
  }
  return false;
}

bool MotorManager::updateMotorPins(uint16_t motorId, const MotorPins &pins)
{
  Motor *motor = find(motorId);
  if (!motor)
  {
    return false;
  }
  motor->pins = pins;
  return true;
}

bool MotorManager::rebuildHardware()
{
  detachAll();
  pcnt_isr_service_uninstall();
  engine.init();

  bool ok = true;
  for (auto &motor : storage)
  {
    if (!attachMotor(motor))
    {
      ok = false;
      break;
    }
  }

  if (!ok)
  {
    detachAll();
  }

  return ok;
}

void MotorManager::detachAll()
{
  for (auto &motor : storage)
  {
    detachMotor(motor);
    motor.motion.mode = RunMode::Idle;
  }
}

bool MotorManager::attachMotor(Motor &motor)
{
  motor.stepper = engine.stepperConnectToPin(static_cast<uint8_t>(motor.pins.step));
  if (!motor.stepper)
  {
    Serial.printf("StepperControl: Failed to attach motor %u to STEP pin %d\n", motor.id, motor.pins.step);
    return false;
  }

  motor.stepper->setDirectionPin(static_cast<uint8_t>(motor.pins.dir));
  motor.stepper->setEnablePin(static_cast<uint8_t>(motor.pins.sleep), false);
  motor.stepper->setAutoEnable(false);
  motor.stepper->disableOutputs();
  motor.driverAwake = false;
  motor.autoSleepRequestMs = 0;
  motor.motion.mode = RunMode::Idle;
  motor.motion.currentDirection = 1;
  motor.motion.anchorPosition = 0;
  motor.motion.segmentStartMs = millis();
  motor.motion.segmentStartPos = 0;
  motion.resetLastRun(motor);
  motion.applyMotionSettings(motor);
  return true;
}

void MotorManager::detachMotor(Motor &motor)
{
  if (motor.stepper)
  {
    motor.stepper->disableOutputs();
    motor.stepper->detachFromPin();
    motor.stepper = nullptr;
  }
  motor.driverAwake = false;
  motor.autoSleepRequestMs = 0;
}

} // namespace StepperControl

