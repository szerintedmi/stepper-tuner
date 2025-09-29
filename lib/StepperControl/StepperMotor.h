#pragma once

#include <Arduino.h>
#include <FastAccelStepper.h>

#include "StepperTypes.h"

namespace StepperControl
{

class StepperMotor
{
public:
  StepperMotor(uint16_t id, const MotorPins &pins);

  uint16_t id() const { return id_; }
  const MotorPins &pins() const { return pins_; }
  void setPins(const MotorPins &pins) { pins_ = pins; }

  bool attach(FastAccelStepperEngine &engine);
  void detach();
  bool isAttached() const { return stepper_ != nullptr; }

  bool driverAwake() const { return driverAwake_; }
  void setDriverAwake(bool awake, unsigned long wakeDelayMs);
  bool ensureDriverAwake(unsigned long wakeDelayMs);

  unsigned long autoSleepRequestMs() const { return autoSleepRequestMs_; }
  void startAutoSleepTimer(unsigned long now) { autoSleepRequestMs_ = now; }
  void clearAutoSleepTimer() { autoSleepRequestMs_ = 0; }

  MoveResultCode moveTo(long targetPosition);
  bool isRunning() const;
  long currentPosition() const;
  long targetPosition() const;
  float currentSpeedHz() const;
  void setCurrentPosition(long position);
  void forceStopAtCurrentPosition();

  bool configureMotion(float maxSpeedHz, float acceleration);

  void resetRuntimeState();
  void beginSegment(unsigned long startMs, long anchorPos, int direction);
  void recordLastRun(bool aborted, unsigned long now, long currentPos);

  void setSegmentSteps(long steps) { segmentSteps = steps; }

  RunMode mode = RunMode::Idle;
  int currentDirection = 1;
  long anchorPosition = 0;
  long segmentSteps = 0;
  unsigned long segmentStartMs = 0;
  long segmentStartPos = 0;

  struct LastRunInfo
  {
    bool valid = false;
    bool aborted = false;
    unsigned long startMs = 0;
    unsigned long durationMs = 0;
    long steps = 0;
    unsigned long loopMaxGapUs = 0;
  } lastRun;

private:
  uint16_t id_ = 0;
  MotorPins pins_;
  FastAccelStepper *stepper_ = nullptr;
  bool driverAwake_ = false;
  unsigned long autoSleepRequestMs_ = 0;
};

} // namespace StepperControl

