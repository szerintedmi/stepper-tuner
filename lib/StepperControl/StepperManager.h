#pragma once

#include <Preferences.h>

#include <vector>

#include "StepperMotor.h"
#include "StepperTypes.h"

namespace StepperControl
{

  class StepperManager
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
    bool startHoming(uint16_t motorId);
    void stop(uint16_t motorId, bool aborted);
    void stopAll(bool aborted);
    void reset(uint16_t motorId);
    void resetAll();

    bool saveDefaults();
    bool restoreDefaults();

    void setDriverAwake(uint16_t motorId, bool awake);
    void setDriverAwakeAll(bool awake);
    bool isDriverAwake(uint16_t motorId) const;
    bool isAutoSleepEnabled() const { return config.autoSleep; }

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
      bool limitsEnabled = false;
      long limitMin = 0;
      long limitMax = 0;
      long homingOvershootSteps = 0;
      long homingBackoffSteps = 0;
    } config;

    FastAccelStepperEngine engine;
    std::vector<StepperMotor> motors;
    uint16_t nextMotorId = 1;

    Preferences prefs;
    bool prefsOpen = false;

    uint32_t limitEventSequence = 0;
    uint16_t limitEventMotorId = 0;
    mutable uint32_t lastReportedLimitEventSequence = 0;

    static long roundToLong(double value);
    long clampStepsPerRev(long steps) const;
    float clampMaxSpeed(float value) const;
    float clampAcceleration(float value) const;

    TargetState computeTarget(TargetUnits units, double rawValue, long stepsPerRev) const;
    void updateTarget(TargetUnits units, double value);
    void syncIdlePlannedSteps();

    StepperMotor *findMotor(uint16_t motorId);
    const StepperMotor *findMotor(uint16_t motorId) const;

    bool applyLimits(StepperMotor &motor, long &targetPosition);

    bool ensurePrefs();
    void loadDefaults();
    void loadMotorsFromPrefs();
    void persistMotors();

    bool rebuildHardware();
    void detachAll();

    bool beginMove(StepperMotor &motor, int direction, RunMode mode);
    void continuePingPong(StepperMotor &motor);
    void continueHoming(StepperMotor &motor, unsigned long now);
    void stopMotor(StepperMotor &motor, bool aborted);
    void resetMotor(StepperMotor &motor);
    void applyMotionSettings(StepperMotor &motor);
    void applyMotionSettingsAll();
  };

} // namespace StepperControl
