#pragma once

#include <Arduino.h>

#include <FastAccelStepper.h>
#include <Preferences.h>
#include <memory>
#include <vector>

#include "MotorManager.h"

namespace StepperControl
{

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
    struct Pins
    {
      int step = -1;
      int dir = -1;
      int sleep = -1;
    } pins;

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

class Motion
{
public:
  void begin();
  void loop();

  StepperState state() const;

  bool addMotor(const MotorPins &pins, uint16_t *newMotorId = nullptr);
  bool hasMotor(uint16_t motorId) const;
  bool removeMotor(uint16_t motorId);
  bool updateMotorPins(uint16_t motorId, const MotorPins &pins);

  bool startSingle(uint16_t motorId, int direction);
  bool startPingPong(uint16_t motorId, int direction);
  void stop(uint16_t motorId, bool aborted);
  void stopAll(bool aborted);
  void reset(uint16_t motorId);
  void resetAll();

  bool saveDefaults();
  bool restoreDefaults();

  void setDriverAwake(uint16_t motorId, bool awake);
  void setDriverAwakeAll(bool awake);
  bool driverAwake(uint16_t motorId) const;
  bool autoSleepEnabled() const { return config.autoSleep; }

  void applySettings(const SettingsPatch &patch);

private:
  friend class MotorManager;
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
    bool limitsEnabled = false;
    long limitMin = 0;
    long limitMax = 0;
  } config;

  FastAccelStepperEngine engine;
  std::unique_ptr<MotorManager> motorManager;
  uint16_t nextMotorId = 1;

  Preferences prefs;
  bool prefsOpen = false;

  static long roundToLong(double value);

  long clampStepsPerRev(long steps) const;
  float clampMaxSpeed(float value) const;
  float clampAcceleration(float value) const;

  TargetState computeTarget(TargetUnits units, double rawValue, long stepsPerRev) const;
  void updateTarget(TargetUnits units, double value);

  MotorManager &manager();
  const MotorManager &manager() const;
  MotorManager::Motor *findMotor(uint16_t motorId);
  const MotorManager::Motor *findMotor(uint16_t motorId) const;
  void updateIdleSegmentSteps();
  bool applyLimits(MotorManager::Motor &motor, long &targetPosition);

  void resetLastRun(MotorManager::Motor &motor);
  void recordLastRun(MotorManager::Motor &motor, bool aborted);
  bool ensureDriverAwake(MotorManager::Motor &motor);
  bool beginMove(MotorManager::Motor &motor, int direction);
  void continuePingPong(MotorManager::Motor &motor);
  void applyMotionSettings(MotorManager::Motor &motor);
  void applyMotionSettingsAll();
  void setDriverAwakeForMotor(MotorManager::Motor &motor, bool awake);
  void stopMotor(MotorManager::Motor &motor, bool aborted);
  void resetMotor(MotorManager::Motor &motor);

  bool ensurePrefs();
  void loadDefaults();
  void loadMotorsFromPrefs();
  void persistMotors();

  uint32_t limitEventSequence = 0;
  uint16_t limitEventMotorId = 0;
  mutable uint32_t lastReportedLimitEventSequence = 0;
};

} // namespace StepperControl
