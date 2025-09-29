#include "StepperControl.h"

#include <Arduino.h>

#include "StepperHttpRoutes.h"
#include "StepperManager.h"

namespace StepperControl
{
namespace
{
  StepperManager manager;
  HttpRoutes routes(manager);
}

void begin(AsyncWebServer &server)
{
  Serial.println("StepperControl: begin");
  manager.begin();
  routes.attach(server);
}

void loop()
{
  manager.loop();
}

} // namespace StepperControl
