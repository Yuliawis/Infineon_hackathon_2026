# Maze Solver Car

Autonomous 4-wheel car that explores and solves a grid maze (60 cm cells) using the Trémaux algorithm. Built for the Hackathon Self-Driving Car challenge on a PSoC 6 dual-core MCU.

## Hardware

| Part | Role |
|---|---|
| PSoC 6 BLE Prototyping Kit (CY8CPROTO-063-BLE) | MCU: CM4 runs the car logic, CM0+ runs BLE |
| 4 DC motors + PCA9685 PWM driver (I2C) | Drive and in-place turns |
| HC-SR04 ultrasonic sensor on a pan/tilt servo head | Distance to walls (front, left, right) |
| MPU6050 gyro/accelerometer | Heading (yaw) for turns and straight driving |
| WS2812B LED strip | Status colors |
| Buzzer | Sound |

Pin assignment: [doc/drawing/PortsAndPins_Assignment_2026.JPG](doc/drawing/PortsAndPins_Assignment_2026.JPG).

Board API reference: [doc/API_UserManual.md](doc/API_UserManual.md).

## Project structure

```
proj_cm4/              Main application (CM4)
  drivers/             HCSR04, MPU6050, PCA9685, Servo
  modules/explore/     Exploration loop: scan -> choose -> turn -> drive
  modules/maze/        Maze map and Trémaux decisions
  modules/car/         Motors, gyro turns, straight driving with distance control
  modules/motion/      Yaw / tilt from the MPU6050
  modules/head/        Pan/tilt servos of the sensor head
  modules/distance/    Median-filtered ultrasonic distance
  modules/ledctrl/     LED strip
proj_cm0p/             BLE stack and log forwarding (CM0+)
cores_shared/          IPC definitions shared by both cores
tools/                 Laptop tools: BLE log/telemetry, live maze viewer
doc/                   Manual, schematic, pin diagrams
```

## Algorithm

The car tracks its position on a 30×30 cell grid, starting in the middle and facing north. Each step ([explore.c](proj_cm4/modules/explore/explore.c)):

1. **Fix heading.** If the gyro yaw is more than 6° off, turn back so walls are measured square.
2. **Scan.** The head looks left, right and front; a side closer than the threshold (front 400 mm, sides 450 mm) is a wall, otherwise open. Results are stored per cell edge.
3. **Choose** ([maze.c](proj_cm4/modules/maze/maze.c)), Trémaux rules with at most 2 marks per passage:
   - Arrived in an already visited cell and the way back has 1 mark → go back.
   - Otherwise take an open passage with 0 marks, preferring right, left, forward, back.
   - Otherwise take one with 1 mark.
   - Nothing available → the run is finished (red LED).
4. **Turn** in place by 0 / ±90 / 180° using the gyro, relative to a commanded heading, so turn errors do not accumulate.
5. **Drive one cell** holding the heading with the gyro. With a wall ahead, the car stops at the nearest cell center by the front distance. Without one, it drives blind for a fixed time. If a wall is too close, the car stays and marks that side as a wall.
6. Add a mark to the passage and update the position.

Tunable constants are at the top of [explore.h](proj_cm4/modules/explore/explore.h), [maze.h](proj_cm4/modules/maze/maze.h) and [car.c](proj_cm4/modules/car/car.c) (cell time, wall thresholds, sensor offset, turn/drive speeds). Calibrate them on the real car.

### Status LEDs

| Color | Meaning |
|---|---|
| off | 3 s pause before the start |
| blue | scanning the cell |
| green | turning, or driving by the sensor |
| purple | driving on time, no wall ahead (blind) |
| white | wall too close: drives only up to the margin |
| red | finished, nowhere to go |

## Setup

Requires ModusToolbox 3.9+ (see [INSTALL.md](INSTALL.md)) and the kit connected over USB. Switch the car on with its power button.

```bash
make build      # build both cores
make program    # build and flash both cores
```

Laptop tools (Python 3):

```bash
pip install -r requirements.txt
```

The car starts moving 5 s after power-on. Place it in the start cell aligned with the walls, and keep it still during gyro calibration.

### Build options

Pass the same options to `make program`, e.g. `make program BLE_LOG=0`.

| Option | Default | Meaning |
|---|---|---|
| `BLE_LOG=1` / `0` | `1` | Send the log of both cores over BLE (needed by both tools) |
| `RETARGET_CORE=CM4` / `CM0P` / empty | `CM4` | Core that owns the USB debug UART; empty = no UART log |

## Scripts

Both tools talk to the car over BLE. The car accepts one connection at a time, so **run only one tool at once**. Do not pair the car in the system Bluetooth settings. If it is not found, close other connected apps or run `bluetoothctl disconnect 00:A0:50:10:29:39`.

### [tools/ble_serial.py](tools/ble_serial.py): log and telemetry

```bash
python3 tools/ble_serial.py                               # find the car by name (MazeCar-T5-2)
python3 tools/ble_serial.py --address 00:A0:50:10:29:39   # or by address
python3 tools/ble_serial.py --ui                          # window: values, compass, plots
python3 tools/ble_serial.py --ui --window 30              # plots show the last 30 s (0 = whole run)
python3 tools/ble_serial.py --ui --replay log.txt         # no car: show a saved log
python3 tools/ble_serial.py --csv run1.csv                # also save telemetry as time_s,key,value
```

Prints the log, reconnects on drop. Telemetry lines look like `[12.345] T hdg=90 dist=320` (`-1` = no value). Stop with `Ctrl-C`.

### [tools/maze_viewer.py](tools/maze_viewer.py): live maze map

```bash
python3 tools/maze_viewer.py                      # connect before the car starts moving (it waits 5 s)
python3 tools/maze_viewer.py --replay log.txt     # replay saved log lines
python3 tools/maze_viewer.py --log                # also print the car's other log lines
```

Draws walls, unscanned sides, visited cells, Trémaux marks (orange = 1, red = 2) and the robot, plus current heading and front distance. The map is built from `@R` / `@C` / `@M` log lines (format in [maze.h](proj_cm4/modules/maze/maze.h)).
