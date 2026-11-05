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

CM4 log lines start with `[s.mmm]`, the time since boot (CM0+ lines have no time). Telemetry lines have the form
`[12.345] T hdg=90 dist=320`. Add `--csv run1.csv` to also save them as `time_s,key,value` rows (`-1` = no value).
The log written before you connect is kept (the first 2 KB) and sent once the script subscribes.

- Do not pair the car in the system Bluetooth settings, the firmware does not support pairing.
- The car accepts one connection at a time and stops advertising while connected. If the script keeps printing
  `car not found`, close other apps connected to it or run `bluetoothctl disconnect 00:A0:50:10:29:38`.
