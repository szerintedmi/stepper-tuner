#include <AccelStepper.h>

#define PIN_STEP 12 // The ESP32 pin GPIO12 connected to STEP pin of DRV8825 module
#define PIN_DIR 14  // The ESP32 pin GPIO14 connected to DIR pin of DRV8825 module

// Creates an instance
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.print("Setup(): start");

  // set the maximum speed, acceleration factor
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(200);
  // set speed and the target position
  stepper.setSpeed(1000);
  stepper.moveTo(800);

  Serial.print("Setup(): done");
}

void loop()
{

  // // Change direction once the motor reaches target position
  if (stepper.distanceToGo() == 0)
    stepper.moveTo(-stepper.currentPosition());

  stepper.run(); // Move the motor one step
  sleep(10000);
}
