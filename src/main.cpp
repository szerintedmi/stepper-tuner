#include <AccelStepper.h>

#define PIN_STEP 12  // The ESP32 pin GPIO12 connected to STEP pin of DRV8825 module
#define PIN_DIR 14   // The ESP32 pin GPIO14 connected to DIR pin of DRV8825 module
#define PIN_SLEEP 27 // The ESP32 pin GPIO27 connected to SLEEP pin of DRV8825 module

// Creates an instance
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.print("Setup(): start");

  pinMode(PIN_SLEEP, OUTPUT);
  digitalWrite(PIN_SLEEP, HIGH); // Keep driver awake initially

  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(4000);

  stepper.setSpeed(1000);
  stepper.moveTo(800);

  Serial.print("Setup(): done");
}

void loop()
{
  if (stepper.distanceToGo() == 0)
  {
    digitalWrite(PIN_SLEEP, LOW); // Put driver to sleep to reduce heat
    delay(2000);

    // Wake driver and allow time to stabilize (tWAKE ~ 1.7ms)
    digitalWrite(PIN_SLEEP, HIGH);
    delay(2);

    stepper.moveTo(-stepper.currentPosition());
  }

  // Move the motor one step
  stepper.run();
}
