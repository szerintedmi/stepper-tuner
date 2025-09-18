#pragma once

#include <ESPAsyncWebServer.h>

namespace StepperControl
{
  void begin(AsyncWebServer &server);
  void loop();
}

