# Hackathon Self-Driving Car

Please refer to [documentation](/doc/API_UserManual.md) for the complete information.


## Setup

Requires ModusToolbox 3.9 (see [INSTALL.md](INSTALL.md)) and the PSoC 6 BLE kit connected over USB.

Build both cores:

```bash
make build
```

Build and flash both cores:

```bash
make program
```

Install the laptop tools (Python 3):

```bash
pip install -r requirements.txt
```


## BLE debug

The firmware sends all `DBG_PRINTF` output of both cores over BLE. Logging is set with build arguments (pass the
same ones to `make program`, e.g. `make program BLE_LOG=0`):

| Argument | Default | Meaning |
|---|---|---|
| `BLE_LOG=1` / `0` | `1` | send the log of both cores over BLE (needed by both laptop tools) |
| `RETARGET_CORE=CM4` / `CM0P` / empty | `CM4` | core that owns the USB debug UART; empty = no UART log |

```bash
python3 tools/ble_serial.py                               # finds the car by name (MazeCar-T5-2)
python3 tools/ble_serial.py --address 00:A0:50:10:29:39   # or by address
python3 tools/ble_serial.py --ui                          # telemetry window: values, compass, plots
python3 tools/ble_serial.py --ui --window 30              # plots show the last 30 s (0 = whole run)
python3 tools/ble_serial.py --ui --replay log.txt         # no car: show a saved log
```

The script connects automatically, prints the log and reconnects if the connection drops. Stop it with `Ctrl-C`.
The `--ui` window gets one plot per telemetry key (new keys appear by themselves). It also shows a heading compass,
the current values, the link state and the last log lines. Gaps in a plot mean the car had no value (`-1`).

CM4 log lines start with `[s.mmm]`, the time since boot (CM0+ lines have no time). Telemetry lines have the form
`[12.345] T hdg=90 dist=320`. Add `--csv run1.csv` to also save them as `time_s,key,value` rows (`-1` = no value).
The log written before you connect is kept (the first 2 KB) and sent once the script subscribes.

- Do not pair the car in the system Bluetooth settings, the firmware does not support pairing.
- The car accepts one connection at a time and stops advertising while connected. If the script keeps printing
  `car not found`, close other apps connected to it or run `bluetoothctl disconnect 00:A0:50:10:29:39`.


## Live maze map

While exploring, the car logs every map change as a short text line (`@R` reset, `@C` cell scanned, `@M` move; see
`proj_cm4/modules/maze/maze.h`). They travel in the BLE log, so `BLE_LOG=1` (default) is required. Show the map in a window:

```bash
python3 tools/maze_viewer.py                      # connect before the car starts moving (it waits 5 s)
python3 tools/maze_viewer.py --replay log.txt     # no car: replay saved log lines
python3 tools/maze_viewer.py --log                # also print the car's other log lines
```

It draws walls, unmeasured sides, visited / seen cells, the Tremaux marks (orange 1, red 2) and the robot. Next to
the map it shows the current heading and front distance from the telemetry (gray = no value or no fresh data). The window
title shows the link state (scanning, connected, no data yet, waiting for the run). If the viewer connected after the
run started, it still draws what arrives and adds `(run start missed)` to the title. The viewer does not print logs by
default. Do not run it together with `ble_serial.py`: the car accepts one connection at a time.


## Status LEDs

| Color | When |
|---|---|
| off | 3 s pause before the start |
| blue | scanning the cell |
| green | turning and driving by the sensor |
| purple | driving on time, no wall ahead (blind) |
| white | wall ahead too close: drives only up to the margin; stays white 300 ms if the next cell was not reached |
| red | finished, nowhere to go |
