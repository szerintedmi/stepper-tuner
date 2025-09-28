#include "StepperControl.h"

#include <Arduino.h>

#include "StepperHttpRoutes.h"
#include "StepperMotion.h"

namespace StepperControl
{
namespace
{
  Motion motion;
  HttpRoutes routes(motion);
}

void begin(AsyncWebServer &server)
{
  Serial.println("StepperControl: begin");
  motion.begin();
  routes.attach(server);
}

void loop()
{
  motion.loop();
}

} // namespace StepperControl

