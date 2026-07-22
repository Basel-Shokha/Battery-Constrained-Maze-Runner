# 🔋 Battery-Constrained Maze Runner — Group 6

A robotic delivery system built around a **remote-brain architecture**: an ESP32-driven robot navigates a physical maze while a C++ routing server on a PC computes optimal, battery-aware paths and streams live commands over Wi-Fi. A browser dashboard visualizes the maze, the robot's live position, and its simulated battery state in real time.

Built as part of the IOT project for the **Interdisciplinary Center for Smart Technologies (ICST)**, Taub Faculty of Computer Science, Technion.

> [!NOTE]
> This project treats "battery" as a **simulated resource tied to grid distance** — energy drains strictly per grid cell traveled, not per motor cycle or wheel slip. This keeps the pathfinding problem clean: it's a constrained shortest-path problem, not a physics simulation.

---

## 🧠 System Architecture

```
┌─────────────────┐   HTTP (8085)    ┌──────────────────────┐
│  index.html      │ ───────────────▶ │   main.cpp            │
│  Dashboard (PC)   │ ◀─────────────── │   Routing Server (C++) │
└─────────────────┘   UDP  (8086)    │   MazeSolver engine   │
                       telemetry      └──────────┬────────────┘
                                                  │ Wi-Fi (custom text protocol)
                                                  ▼
                                        ┌───────────────────┐
                                        │   M5StickC Plus     │
                                        │   (RoverC mecanum)  │
                                        └─────────┬──────────┘
                                                  │ UART / I2C
                                                  ▼
                                        ┌───────────────────┐
                                        │   ESP32 co-processor │
                                        │   ToF · LED · MP3    │
                                        └───────────────────┘
```

- **PC (Remote Brain):** the routing server (`main.cpp` + `MazeSolver`) receives the maze layout from the dashboard, runs pathfinding under a battery-capacity constraint, and streams movement instructions to the robot. It also receives live telemetry over UDP and serves it back to the dashboard over HTTP.
- **M5StickC Plus:** onboard controller for the RoverC mecanum chassis. Handles Wi-Fi comms with the PC, motor control, and gyro-based heading lock.
- **ESP32 co-processor:** handles the "senses" — three VL53L1X ToF distance sensors for wall/collision detection, a WS2812B LED ring for battery/status indication, and an MP3 module for audio cues.
- **Dashboard (`index.html`):** a single-file browser app for maze setup, path visualization, live tracking, and manual route drawing.

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
| ESP32 DevKit V1 | Wi-Fi command reception, motor/LED/audio driving |
| M5StickC Plus + RoverC | Mecanum-wheel base and onboard controller |
| TB6612FNG Dual Motor Driver | Efficient, low-heat motor voltage/direction control |
| WS2812B RGB LED Ring | Visual battery gauge + mission status |
| OPEN_SMART MP3 Player Board | UART audio cues with built-in amplifier |
| VL53L1X ToF Distance Sensors ×3 | Narrow-beam collision safety buffer |
| 2× 18650 Li-ion Cells + Holders | ~7.4V primary power source |
| TP4056 Charging Module | Safe individual 18650 recharging between runs |

Physical constants used throughout the routing/firmware logic: **300 mm** corridor width, **95×110 mm** rover footprint, **40 mm** ToF sensor inset.

---

## 🌐 Communication Protocol

- **PC → M5 (Wi-Fi):** newline-delimited text instructions (e.g. `START:1,0,0,8`), including movement, journey start, and route-error commands.
- **PC ⇄ Dashboard (HTTP, port 8085):** REST-style endpoints — `/send_maze`, `/solve`, `/start_journey`, `/calibrate`, `/reset`, `/get_maze`, `/get_instructions`, `/get_telemetry`.
- **Robot → PC (UDP, port 8086):** live telemetry stream (step index, yaw, ToF distances, robot state).
- **M5 ⇄ ESP32 (Serial):** shared command IDs and audio-cue IDs defined in `ESP32/PROTOCOL.h`.

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

*Project by Group 6 (Basel, Nicole, Shoki) — supervised by Tom. Part of ICST, the Interdisciplinary Center for Smart Technologies, Taub Faculty of Computer Science, Technion.*
