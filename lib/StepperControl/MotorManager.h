#pragma once

#include <cstdint>
#include <FastAccelStepper.h>
#include <vector>

namespace StepperControl
{

class Motion;

enum class RunMode : uint8_t
{
  Idle,
  Single,
  PingPong
};

struct MotorPins
{
  int step = -1;
  int dir = -1;
  int sleep = -1;
};

class MotorManager
{
public:
  struct MotionState
  {
    RunMode mode = RunMode::Idle;
    int currentDirection = 1;
    long anchorPosition = 0;
    long segmentSteps = 0;
    unsigned long segmentStartMs = 0;
    long segmentStartPos = 0;
  };

  struct LastRunInfo
  {
    bool valid = false;
    bool aborted = false;
    unsigned long startMs = 0;
    unsigned long durationMs = 0;
    long steps = 0;
    unsigned long loopMaxGapUs = 0;
  };

  struct Motor
  {
    uint16_t id = 0;
    MotorPins pins;
    FastAccelStepper *stepper = nullptr;
    bool driverAwake = false;
    unsigned long autoSleepRequestMs = 0;
    MotionState motion;
    LastRunInfo lastRun;
  };

  MotorManager(FastAccelStepperEngine &engine, Motion &motion);

  std::vector<Motor> &motors();
  const std::vector<Motor> &motors() const;

  Motor *find(uint16_t motorId);
  const Motor *find(uint16_t motorId) const;

  Motor *addMotor(uint16_t id, const MotorPins &pins);
  bool removeMotor(uint16_t motorId);
  bool updateMotorPins(uint16_t motorId, const MotorPins &pins);

  bool rebuildHardware();
  void detachAll();

private:
  FastAccelStepperEngine &engine;
  Motion &motion;
  std::vector<Motor> storage;

  bool attachMotor(Motor &motor);
  void detachMotor(Motor &motor);
};

} // namespace StepperControl
