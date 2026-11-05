# Hackathon Self-Driving Car

Please refer to [documentation](/doc/API_UserManual.md) for the complete information.

## Setup

Requires ModusToolbox 3.9 (see [INSTALL.md](INSTALL.md)) and the PSoC 6 BLE kit connected over USB.

Build both cores:

```bash
make build
```

Flash both cores:

```bash
make program
```

Install the laptop tools (Python 3):

```bash
pip install -r requirements.txt
```

## BLE debug

The firmware sends all `DBG_PRINTF` output of both cores over BLE. It is enabled by default; disable it with `make build BLE_LOG=0`.

```bash
python3 tools/ble_serial.py                              # finds the car by name (MazeCar-T5)
python3 tools/ble_serial.py --address 00:A0:50:10:29:38  # or by address
```

The script connects automatically, prints the log and reconnects if the connection drops. Stop it with `Ctrl-C`.

- Do not pair the car in the system Bluetooth settings, the firmware does not support pairing.
- The car accepts one connection at a time and stops advertising while connected. If the script keeps printing
  `car not found`, close other apps connected to it or run `bluetoothctl disconnect 00:A0:50:10:29:38`.

## Live maze map

While exploring, the car logs every map change as a short text line (`@R` reset, `@C` cell scanned, `@M` move; see
`proj_cm4/modules/maze/maze.h`). They travel in the BLE log, so `BLE_LOG=1` (default) is required. Show the map in a window:

```bash
python3 tools/maze_viewer.py                      # connect before the car starts moving (it waits 3 s)
python3 tools/maze_viewer.py --replay log.txt     # no car: replay saved log lines
python3 tools/maze_viewer.py --log                # also print the car's other log lines
```

It draws walls, unmeasured sides, visited / seen cells, the Tremaux marks (orange 1, red 2) and the robot. The viewer
does not print logs by default, so do not run it together with `ble_serial.py` (one connection at a time).
