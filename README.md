# 🔋 Battery-Constrained Maze Runner — Group 6

A robotic delivery system built around a **remote-brain architecture**: an M5StickC Plus-driven rover navigates a physical maze while a C++ routing server on a PC computes optimal, battery-aware paths that the rover polls over Wi-Fi. A browser dashboard visualizes the maze, the robot's live position, and its simulated battery state in real time.

Built as part of the IOT project for the **Interdisciplinary Center for Smart Technologies (ICST)**, Taub Faculty of Computer Science, Technion.

> [!NOTE]
> This project treats "battery" as a **simulated resource tied to grid distance** — energy drains strictly per grid cell traveled, not per motor cycle or wheel slip. This keeps the pathfinding problem clean: it's a constrained shortest-path problem, not a physics simulation.

---

## 🧠 System Architecture

```
┌───────────────────┐    HTTP :8085     ┌────────────────────────┐
│  index.html         │ ─────────────────▶ │      main.cpp             │
│  Dashboard (PC)      │ ◀───────────────── │  Routing Server (C++)     │
└───────────────────┘   (send_maze,       │  MazeSolver engine        │
                          solve, reset,     └───────────┬────────────┘
                          get_telemetry…)               │
                                             ▲           │  M5 polls this same
                                     UDP :8086│           │  server over HTTP GET
                                    telemetry │           │  (get_instructions,
                                     (push)   │           ▼  get_calibrate, get_reset)
                                   ┌───────────────────────┐
                                   │     M5StickC Plus       │
                                   │     (RoverC mecanum)    │
                                   │  Wi-Fi + motors + gyro   │
                                   └────────────┬────────────┘
                                                │ UART (PACKET:/DIST: protocol)
                                                ▼
                                   ┌───────────────────────┐
                                   │   ESP32 co-processor    │
                                   │   ToF · LED · MP3        │
                                   └───────────────────────┘
```

- **PC (Remote Brain):** the routing server (`main.cpp` + `MazeSolver`) receives the maze layout from the dashboard, runs pathfinding under a battery-capacity constraint, and holds the resulting instructions ready for the rover to pick up. It's a pull model on the rover's side: the M5 itself polls the server over HTTP for instructions, calibration, and reset requests, and separately pushes live telemetry to the server over UDP and posts event notifications (`notify_event`) over HTTP. The server also serves the dashboard its own set of endpoints (`send_maze`, `solve`, `get_maze`, `get_telemetry`, etc.).
- **M5StickC Plus:** the actual Wi-Fi endpoint of the robot. Polls the PC server for its route, drives the RoverC mecanum motors directly over I2C, and runs gyro-based heading lock and the emergency strafe/turn state machine.
- **ESP32 co-processor:** has no Wi-Fi role at all — it's a UART peripheral to the M5, handling the "senses": three VL53L1X ToF distance sensors, a WS2812B LED ring for battery/status indication, and an MP3 module for audio cues.
- **Dashboard (`index.html`):** a single-file browser app for maze setup, path visualization, live tracking, and manual route drawing.

### Why two controllers instead of one

The M5StickC Plus alone would be the simplest choice — its onboard gyroscope is proven accurate, it handles Wi-Fi and its own power natively, and it has a built-in screen for logs. The problem is pin budget: once the RoverC chassis and motors claim their pins, there aren't enough digital I/O lines left on the M5 to also drive three ToF sensors (each needing its own `XSHUT` pin), the LED ring, and the MP3 module. Wiring all of that externally would mean pulling the M5 out of its fixed seat in the chassis, exposing loose wiring, and risking the gyroscope's mounting accuracy.

Going the other direction — ESP32-only — was also considered and rejected as high-risk: it would require an unproven external gyroscope and display, uncertain mounting stability, and a single processor and single battery feeding the drive motors, sensors, LEDs, and audio all at once. That shared electrical load and the temptation to "just use a bigger battery" adds chassis weight, which stresses the small drive motors and degrades navigation precision.

The two-controller split resolves this cleanly: the M5 stays factory-mounted, powered by the chassis, and dedicated purely to navigation and PC communication. The ESP32 gets its own separate battery for sensors, LEDs, and audio, so a momentary voltage dip from the motors never resets or glitches sensor readings. As a side benefit for the project fair: at the end of a maze run the M5 can be lifted off the floor as a self-contained unit — its own battery and screen let it show run logs and stats without being tethered to a laptop.

---

## 👥 User Roles

| Role | Responsibilities |
|---|---|
| **Dispatcher** | Monitors the live dashboard and robot status, watches the battery LED ring, operates the mission in real time. |
| **Maze Setter** | Inputs the physical maze layout and charging station coordinates, configures battery parameters, calibrates hardware, reviews run logs. |

---

## ✨ Core Features

- **Grid-based maze mapping** — Maze Setter inputs the physical layout and charging station coordinates directly into the dashboard.
- **Battery-constrained pathfinding** — the routing server rejects/reroutes any path exceeding the robot's remaining battery, measured strictly in grid cells traveled.
- **Continuous Operation mode** — battery drain can be disabled entirely for long-haul test runs.
- **Live battery gauge** — the WS2812B ring displays battery percentage as a proportion of lit (green) pixels, updated during movement.
- **Charging sequence feedback** — at a charging station the robot pauses ~4s, blinks blue twice, and plays a "charging complete" cue.
- **Mission audio cues** — distinct voice lines for mission start, destination reached, and route failure.
- **Mission failure alerts** — LEDs turn red and the robot announces a failure if the destination is unreachable within the current battery budget.
- **Path visualization** — the dashboard draws the calculated optimal route before the robot moves, so it can be verified visually first.
- **Live telemetry tracking** — the robot's current grid cell is highlighted on the dashboard map in real time as it moves.
- **Wi-Fi drop resilience** — if Wi-Fi drops mid-mission, the robot autonomously completes its already-buffered route while the dashboard shows a "Connection Lost" warning.
- **Run analytics logging** — a text log is generated per run: total time, battery units consumed, and total grid tiles traveled.

---

## 📁 Repository Structure

```
├── main.cpp              # PC-side routing server (HTTP :8085 + UDP telemetry :8086)
├── MazeSolver.h / .cpp    # Pathfinding engine (graph condensing, BFS/Dijkstra, battery-aware solve)
├── index.html             # Browser dashboard — maze setup, live tracking, manual routing
├── External Libraries/    # Vendored httplib.h and nlohmann json.hpp
│
├── M5/                    # M5StickC Plus firmware (RoverC mecanum controller)
│   ├── M5.ino
│   ├── COMM_PC.ino         # Wi-Fi link to the PC routing server
│   ├── COMM_ESP32.ino      # Serial link to the ESP32 co-processor
│   ├── GYROSCOPE.ino       # Heading lock / drift correction
│   └── BATTERY_ENGINE.ino  # Onboard battery bookkeeping
│
├── ESP32/                  # ESP32 co-processor firmware
│   ├── ESP32.ino
│   ├── SENSORS.ino         # VL53L1X ToF sensor handling
│   ├── LED.ino              # WS2812B ring control
│   ├── MP3.ino               # UART MP3 module / audio cues
│   ├── RECHARGING_LOGIC.ino
│   ├── PROTOCOL.h            # Shared command + audio-cue constants
│   └── parameters.h           # Pin mapping, PID/turn tuning, thresholds
│
├── maze_solver/ , maze_server/   # Compiled server/solver binaries
├── TESTING_COMM/                  # Standalone comms test harness (PC ⇄ M5)
├── Unit Tests/                     # Per-component hardware validation tests
├── Documentation/                  # Wiring diagram + operating instructions
├── flutter_app/                    # (planned) companion mobile app
└── routing-algorithm-proof.pdf      # Technical write-up of the pathfinding engine
```

---

## 🔌 Hardware

| Part | Purpose |
|---|---|
| ESP32 DevKit V1 | UART-connected co-processor for sensors, LED ring, and audio (no Wi-Fi role) |
| M5StickC Plus + RoverC | Wi-Fi endpoint, motor control, and mecanum-wheel base |
| TB6612FNG Dual Motor Driver | Efficient, low-heat motor voltage/direction control |
| WS2812B RGB LED Ring | Visual battery gauge + mission status |
| OPEN_SMART MP3 Player Board | UART audio cues with built-in amplifier |
| VL53L1X ToF Distance Sensors ×3 | Narrow-beam collision safety buffer |
| 2× 18650 Li-ion Cells + Holders | ~7.4V primary power source |
| TP4056 Charging Module | Safe individual 18650 recharging between runs |

Physical constants used throughout the routing/firmware logic: **300 mm** corridor width, **95×110 mm** rover footprint, **40 mm** ToF sensor inset.

### Wiring

| ESP32 pin / power rail | Connected to |
|---|---|
| 5V supply | ESP32 Vin, MP3 VCC, LED 5V, M5Stick 5V |
| 3.3V supply | ToF sensor array VCC (all 3) |
| Common GND | ESP32 GND, MP3 GND, LED GND, sensors GND |
| D27 | MP3 player TX |
| D14 | MP3 player RX |
| D16 (RX2) | M5Stick G33 (TX) |
| D17 (TX2) | M5Stick G32 (RX) |
| D13 | LED ring data in (Din) |
| D21 | ToF sensors SDA (all 3 shared) |
| D22 | ToF sensors SCL (all 3 shared) |
| D5 | ToF sensor 1 `XSHUT` (left) |
| D23 | ToF sensor 2 `XSHUT` (front) |
| D18 | ToF sensor 3 `XSHUT` (right) |

Full diagram: `Documentation/`.

---

## 🌐 Communication Protocol

All HTTP traffic hits the same server on port 8085, but the dashboard and the M5 talk to different endpoints on it:

- **Dashboard → Server (HTTP POST):** `/send_maze`, `/calibrate`, `/reset`, `/start_journey`, `/solve` — the operator's actions.
- **Dashboard ← Server (HTTP GET):** `/get_maze`, `/get_telemetry` — dashboard polls these to render the map and live position.
- **M5 → Server (HTTP GET, polled by the rover every ~1.5s):** `/get_instructions` (returns newline-delimited text like `START:1,0,0,8`, `STEP:...`, `CMD:START_JOURNEY`, `CMD:ROUTE_ERROR`), `/get_calibrate`, `/get_reset` — the M5 pulls its commands rather than the server pushing them.
- **M5 → Server (HTTP POST):** `/notify_event` — the M5 reports mission events (e.g. `ARRIVED_STATION`, `CHARGING_START`, `MISSION_COMPLETE`) as they happen.
- **M5 → Server (UDP, port 8086):** a continuous telemetry push every 250 ms (step index, yaw, ToF distances, robot state), cached server-side and served to the dashboard via `/get_telemetry`.
- **M5 ⇄ ESP32 (UART, text protocol):** M5 sends `PACKET:<cmdId>,<p1>,<p2>` for audio/charging/LED commands; ESP32 streams `DIST:<left>,<front>,<right>` back every 60 ms. Shared command IDs and audio-cue IDs are defined in `ESP32/PROTOCOL.h`.

---

## 🎯 M5 Autonomous Control Loop (Kinematics)

The M5 runs a strict, sequential super-loop (no parallel threading for movement) so that hardware safety checks and distance telemetry always override standard kinematic calculations. Per cycle (`delay(2)` at the end, so ≈2 ms):

1. **Orientation polling (~2 ms):** `M5.update()` + `updateYaw()` integrate `gyroZ` over microsecond time deltas into a continuous heading.
2. **Interrupt & telemetry handling:** checks abort signals, reads the `DIST: left, front, right` UART stream from the ESP32, handles async Wi-Fi/HTTP polling.
3. **Emergency crab-walk override (~2 ms):** if wall clearance on either side drops to ≤75 mm, forward kinematics are halted instantly in favor of direct lateral (mecanum) strafing away from the wall — persisting until clearance recovers to ≥90 mm (a 15 mm hysteresis band to prevent oscillation).
4. **State machine execution** (`WALKING_FWD`, `TURNING`, `CHARGING_EXEC`, etc.):
   - **Sensor-steering ("Argentina Logic"), every ≥50 ms:** derives smoothed left/right wall-approach velocities from a 10-sample rolling average, then nudges the target heading by ±0.75° toward whichever wall is closer and being approached too fast (>20 cm/s) or drifted from (<−20 cm/s).
   - **Proportional heading control:** angular error `E_θ = θ_target − θ_yaw`, gain `Kp = 0.4` clamped to ±30, applied as differential motor speed around `V_base`.
   - **Battery deduction:** 1 unit per 1.5 s of continuous locomotion.
   - **Obstacle detection:** ignores the front sensor during an initial blind window (1500 ms on grid 1, 1300 ms afterward) to avoid false stops over floor tiles/lines.
   - **Turn verification:** active braking (15 ms reverse thrust at power 25) once angular error ≤2°, plus an anti-stuck ramp that boosts turn speed by +1 (up to +30) if yaw barely changes over a 300 ms window.
5. **Display refresh (≥100 ms)** and **cycle sleep (`delay(2)`).**

Full derivation and formulas: see `routing-algorithm-proof.pdf` and the M5 kinematics whitepaper in `Documentation/`.

---

## 🚀 Getting Started

1. **Flash firmware** — open `M5/M5.ino` in the Arduino IDE for the M5StickC Plus, and `ESP32/ESP32.ino` for the ESP32 co-processor. Pin mappings and tunable constants live in `ESP32/parameters.h`.
2. **Build the routing server** — compile `main.cpp` (depends only on the vendored `External Libraries/httplib.h` and `json.hpp`, no external package manager needed). Run it — it starts the HTTP API on `:8085` and the UDP telemetry listener on `:8086`.
3. **Open the dashboard** — open `index.html` in a browser on the same machine as the routing server (it talks to `http://localhost:8085`).
4. **Set up the maze** — as the Maze Setter, input the grid layout and charging station coordinates in the dashboard, then send it to the server.
5. **Run a mission** — as the Dispatcher, review the visualized optimal path, then start the journey and monitor live telemetry and the battery LED ring.

See `Documentation/` for the full wiring diagram and `Unit Tests/` for per-component hardware validation before a full run.

---

## 📚 References

- [Robot Maze Runner — IOT Spring 2024 (Technion)](https://www.youtube.com/watch?v=giC5lmQVR_E) — general system reference
- [FastLED / WS2812B on ESP32 guide](https://www.sunfounder.com/blogs/news/esp32-with-ws2812b-neopixel-leds-complete-beginner-s-guide) — LED ring control
- [YX5300/YX6300 Serial MP3 UART tutorial](https://randomnerdtutorials.com/esp32-yx5300-yx6300-mp3-player-arduino/) — audio cues
- [VL53L1X Laser ToF — Technion IOT example](https://github.com/ICST-Technion/IOT-Examples/tree/main/hardware%20unit%20tests/VL53L1X) — distance sensing

---

*Project by Group 6 — Nicole Shaer, Basel Shokha, Shoki AbuShkara. Lecturer: Itai Dabran · Instructors: Tom Sofer, Ilay Yavlovich. Part of ICST, the Interdisciplinary Center for Smart Technologies, Taub Faculty of Computer Science, Technion.*
