#include <AccelStepper.h>
#include "WifiPortal.h"

#define PIN_STEP 12  // The ESP32 pin GPIO12 connected to STEP pin of DRV8825 module
#define PIN_DIR 14   // The ESP32 pin GPIO14 connected to DIR pin of DRV8825 module
#define PIN_SLEEP 27 // The ESP32 pin GPIO27 connected to SLEEP pin of DRV8825 module

const long STEPS_PER_REV = 2038; // 2038 * 8; // W-20BYJ ???

// Motion state
bool goingToMax = true;        // toggles between 0 and STEPS_PER_REV
unsigned long moveStartMs = 0; // start time of current move

// Creates an instance
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

// ================= Web server & WifiPortal =================

AsyncWebServer webServer(80);
WifiPortal wifiPortal(MESH_SSID, MESH_PASS);

static void attachRoutes()
{
  if (!(LittleFS.begin(false) || LittleFS.begin(true))) // idempotent
  {
    Serial.println("LittleFS mount failed");
    return;
  }

  webServer.serveStatic("/", LittleFS, "/")
      .setDefaultFile("index.html")
      .setFilter([](AsyncWebServerRequest *r)
                 {
                const String& u = r->url();
                return !(u.startsWith("/wifi/api/") || u == "/wifi/api"); });

  ;
}

void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("Setup(): start");

  attachRoutes();
  wifiPortal.beginAndConnect(webServer, /*staTimeoutMs=*/10000);
  webServer.begin();

  pinMode(PIN_SLEEP, OUTPUT);
  digitalWrite(PIN_SLEEP, HIGH); // Keep driver awake initially
  delay(5);                      // tWAKE ~1.7ms

  stepper.setMinPulseWidth(2); // DRV8825 needs ~1.9 µs min

  // Motion settings for geared W-20BYJ
  // vmax=800; accel=4000 , limmit ~300mA --> single: 2.72s ; 8 rotation = 20.55s (2.6s avg)
  // vmax=1000; accel=4000 , limit ~300mA --> single: 2.26s ; 8  rotation = 16.52s (2s avg)
  stepper.setMaxSpeed(1200);    // W-20BYJ~1000 steps/sec
  stepper.setAcceleration(400); // W-20BYJ~800 steps/sec^2 (quicker ramp to viable speed)

  // Full 360° test: move exactly one output-shaft revolution
  stepper.setCurrentPosition(0);
  stepper.moveTo(STEPS_PER_REV);
  Serial.printf("360° test: moving %ld steps...\n", STEPS_PER_REV);

  moveStartMs = millis();

  Serial.println("Setup(): done");
}

void loop()
{
  // Run motion profile toward target; when reached, wait 3s and reverse
  stepper.run();
  if (stepper.distanceToGo() == 0)
  {
    // Print elapsed time for the just-completed rotation (exclude the pause)
    unsigned long elapsed = millis() - moveStartMs;
    float seconds = elapsed / 1000.0f;
    Serial.printf("Full rotation time: %.2f s\n", seconds);

    digitalWrite(PIN_SLEEP, LOW);
    delay(2000);
    digitalWrite(PIN_SLEEP, HIGH);
    delay(5); // allow DRV8825 to fully wake (tWAKE ~1.7ms)

    goingToMax = !goingToMax;
    long nextTarget = goingToMax ? STEPS_PER_REV : 0;
    Serial.printf("Reversing to target: %ld steps\n", nextTarget);

    stepper.moveTo(nextTarget);

    moveStartMs = millis();
  }
  delay(10);
}
