# Stepper Tuner – Agent Brief

## 1. Repo Map & Purpose

- ESP32/PlatformIO firmware that serves a web UI + JSON API to orchestrate 4-wire stepper via FastAccelStepper (see `./README.md`).
- `src/`: Arduino entry points (`setup`/`loop`) wiring StepperControl + WifiPortal.
- `lib/StepperControl`: motion orchestration, REST handlers, drivetrain limits, and persisting motion defaults via ESP `Preferences`.
- `lib/WifiPortal`: Wi-Fi AP/setup helper wrapping ESPAsyncWebServer + ESP-NOW mesh bootstrap.
- `include/`: headers meant for global include; `secrets.h` overrides AP creds (keep private values out of commits).
- `data_src/`: uncompressed web assets; build step mirrors/gzips into `data/` for LittleFS image.
  - 'data_src/index.html` – main web UI.
  - 'data_src/wifi` – Wi-Fi setup portal UI.

- `tools/`: build helpers (`gzip_fs.py` pre-action for filesystem image).

Note: This section might have drift from reality

## 2. Build / Upload / Verify (possibly out of date)

- `pio run -e esp32dev` – default build (uses `.platformio` in-project core dir; safe in sandbox).
- if you change `data_src/`, then run the pre-action (`pio run -e esp32dev -t buildfs`) to regenerate the LittleFS image, and avoid touching `data/` artifacts directly.
- Never upload firmware or filesystem, leave it to the user.
- Tests directory is scaffolding only; no automated unit tests yet.
- No repo lint tooling; rely on compiler and targeted reviews when refactoring.

## 3. Code Style, Commits, PRs

- Keep it simple: less code and less abstractions/layers are better
- Separation of concerns among modules - encaplsulate responsibilities when doing refactor or additions
- Keep comments minimal: good code with descriptive function and variable names is comment/documentation itself - only add comment where it's absolutely necessary and keep it concise
- Prefer small, single-purpose, focused changes unless asked to refactor.
- Touch web assets carefully: keep HTML/CSS compact, run gzip step via PlatformIO rather than manual `.gz` edits.
- Make sure you follow markdown formatting rules , eg: Headings should be surrounded by blank lines

## 4. Gotchas & Protected Areas (possibly out of date)

- `tools/gzip_fs.py` deletes/regenerates `data/`; edit source files in `data_src/` only.
- Respect `./platformio.ini`
- Avoid mutating generated `.pio`, `.platformio`, or `data/` artifacts

## 5. Key References

- Project overview & quick start: `./README.md`
- Access point/secrets guidance: `include/README`.
- Web portal behavior & mesh bootstrap: `lib/WifiPortal/WifiPortal.h`.
- Motion/REST surface: `lib/StepperControl/StepperControl.cpp` & `lib/StepperControl/StepperHttpRoutes.*`.
- Motion defaults persistence: `lib/StepperControl/StepperMotion.*` exposes `saveDefaults` / `restoreDefaults` and stores steps-per-rev, max speed, acceleration, and auto-sleep in NVS (`Preferences`).
- UI defaults workflow: `data_src/index.html` has a "Save defaults" button (with confirmation) and "Reset to default" that call `/api/settings/default/save|restore`.

Note: This section might have drift from reality

## 6. Agent Operating Principles

- Be direct and concise
- Propose a plan first unless it's a trivial change
- Ask questions when user intent is unclear instead of guessing or making assumptions
- Minimize diff footprint; preserve formatting and comments unless cleanup is requested
- Always run builds (system and filesystem) after changes to verify your changes still can be built
- Never try to deploy
- Keep secrets, credentials, and generated assets out of history
- Avoid large dependency additions; ask before adding.
- if you notice a bug or potential important refactor/simplification opportunity then mention it unless you have been asked for a trivial change only. Balance between sidetracking current task vs. importance (eg. severty of bug of necessity of refactor)
- Keep in mind that references, repo map in this `AGENTS.md` might be out of date. Suggest AGENTS.md updates when needed, including findings / discoveries you made which would help your future work.
