# 4wires-stepper

ESP32/PlatformIO project that exposes a web UI and JSON API for driving a 4-wire stepper motor via FastAccelStepper.

- Async web server serves LittleFS-hosted control panel plus Wi-Fi setup portal.
- REST endpoints cover motion presets, run/stop, driver sleep, and status reporting.
- Configurable target units (revs/degrees/steps), speed, acceleration, and auto-sleep guardrails.

## Quick start

- Install PlatformIO, then `pio run -e esp32dev` to build.
- Upload firmware with `pio run -e esp32dev -t upload` and the UI assets with `pio run -e esp32dev -t uploadfs`.
- Connect to the ESP32 AP (`ESP32-WiFi-Setup` by default) to finalize Wi-Fi or browse to the served UI.
