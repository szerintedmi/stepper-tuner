#pragma once

#include <cstdint>
#include <vector>

namespace StepperControl
{

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

enum class TargetUnits
{
  Revs,
  Steps,
  Degrees
};

struct SettingsPatch
{
  bool hasStepsPerRev = false;
  long stepsPerRev = 0;

  bool hasTargetUnits = false;
  TargetUnits targetUnits = TargetUnits::Revs;

  bool hasTargetValue = false;
  double targetValue = 0.0;

  bool hasMaxSpeed = false;
  float maxSpeed = 0.0f;

  bool hasAcceleration = false;
  float acceleration = 0.0f;

  bool hasAutoSleep = false;
  bool autoSleep = false;

  bool hasLimitsEnabled = false;
  bool limitsEnabled = false;

  bool hasLimitMin = false;
  long limitMin = 0;

  bool hasLimitMax = false;
  long limitMax = 0;
};

struct StepperState
{
  struct Settings
  {
    long stepsPerRev = 0;
    TargetUnits targetUnits = TargetUnits::Revs;
    float targetRevs = 0.0f;
    float targetDegrees = 0.0f;
    long targetSteps = 0;
    float maxSpeed = 0.0f;
    float maxSpeedLimit = 0.0f;
    float acceleration = 0.0f;
    bool autoSleep = false;
    bool limitsEnabled = false;
    long limitMin = 0;
    long limitMax = 0;
  } settings;

  struct LimitStatus
  {
    bool triggered = false;
    uint32_t sequence = 0;
    uint16_t motorId = 0;
  } limit;

  struct Motor
  {
    uint16_t id = 0;
    MotorPins pins;
    RunMode mode = RunMode::Idle;
    bool moving = false;
    bool driverAwake = false;
    int direction = 1;
    long segmentSteps = 0;
    long currentPosition = 0;
    long targetPosition = 0;
    long distanceToGo = 0;
    float speed = 0.0f;

    struct LastRun
    {
      bool valid = false;
      bool aborted = false;
      unsigned long startMs = 0;
      unsigned long durationMs = 0;
      long steps = 0;
      unsigned long loopMaxGapUs = 0;
    } lastRun;
  };

  std::vector<Motor> motors;
};

} // namespace StepperControl

