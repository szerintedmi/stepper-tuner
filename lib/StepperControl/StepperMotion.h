#pragma once

#include <Arduino.h>

#include <FastAccelStepper.h>

namespace StepperControl
{

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
  } settings;

  struct LastRun
  {
    bool valid = false;
    bool aborted = false;
    unsigned long startMs = 0;
    unsigned long durationMs = 0;
    long steps = 0;
    unsigned long loopMaxGapUs = 0;
  } lastRun;

  struct Status
  {
    RunMode mode = RunMode::Idle;
    bool moving = false;
    bool driverAwake = false;
    int direction = 1;
    long segmentSteps = 0;
    long currentPosition = 0;
    long targetPosition = 0;
    long distanceToGo = 0;
    float speed = 0.0f;
  } status;
};

class Motion
{
public:
  void begin();
  void loop();

  StepperState state() const;

  bool startSingle(int direction);
  bool startPingPong(int direction);
  void stop(bool aborted);
  void reset();

  void setDriverAwake(bool awake);
  bool driverAwake() const { return driverAwakeFlag; }
  bool autoSleepEnabled() const { return config.autoSleep; }

  void applySettings(const SettingsPatch &patch);

private:
  struct TargetState
  {
    TargetUnits units = TargetUnits::Revs;
    double value = 0.0;
    float revs = 0.0f;
    float degrees = 0.0f;
    long steps = 0;
  };

  struct Config
  {
    long stepsPerRev = 0;
    TargetState target;
    float maxSpeed = 0.0f;
    float acceleration = 0.0f;
    float maxSpeedLimit = 0.0f;
    bool autoSleep = true;
  } config;

  struct MotionState
  {
    RunMode mode = RunMode::Idle;
    int currentDirection = 1;
    long anchorPosition = 0;
    long segmentSteps = 0;
    unsigned long segmentStartMs = 0;
    long segmentStartPos = 0;
  } motion;

  struct LastRunInfo
  {
    bool valid = false;
    bool aborted = false;
    unsigned long startMs = 0;
    unsigned long durationMs = 0;
    long steps = 0;
    unsigned long loopMaxGapUs = 0;
  } lastRun;

  FastAccelStepperEngine engine;
  FastAccelStepper *stepper = nullptr;

  bool driverAwakeFlag = false;
  unsigned long autoSleepRequestMs = 0;

  static constexpr int PIN_STEP = 12;
  static constexpr int PIN_DIR = 14;
  static constexpr int PIN_SLEEP = 27;

  static constexpr long DEFAULT_STEPS_PER_REV = 2038;
  static constexpr float DEFAULT_REVS = 1.0f;
  static constexpr float DEFAULT_MAX_SPEED = 800.0f;
  static constexpr float DEFAULT_ACCEL = 4000.0f;
  static constexpr float MAX_SPEED_LIMIT = 4000.0f;
  static constexpr float MAX_ACCEL = 30000.0f;
  static constexpr long MAX_STEPS_PER_REV = 200000;
  static constexpr unsigned long DRIVER_WAKE_DELAY_MS = 5;
  static constexpr unsigned long AUTO_SLEEP_DELAY_MS = 250;

  static long roundToLong(double value);

  long clampStepsPerRev(long steps) const;
  float clampMaxSpeed(float value) const;
  float clampAcceleration(float value) const;

  TargetState computeTarget(TargetUnits units, double rawValue, long stepsPerRev) const;
  void updateTarget(TargetUnits units, double value);

  void applyMotionSettings();
  void resetLastRun();
  void recordLastRun(bool aborted);

  bool ensureDriverAwake();
  bool beginMove(int direction);
  void continuePingPong();
};

} // namespace StepperControl

