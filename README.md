# Hackathon Self-Driving Car

Please refer to [documentation](/doc/API_UserManual.md) for the complete information.

## Reading debug logs over BLE

The firmware sends all `DBG_PRINTF` output of both cores over BLE (enabled by default, disable with `make build BLE_LOG=0`).
To read it on a laptop:

```bash
pip install -r requirements.txt
python3 tools/ble_serial.py                              # finds the car by name (MazeCar-T5)
python3 tools/ble_serial.py --address 00:A0:50:10:29:38  # or by address
```

The script connects automatically, prints the log and reconnects if the connection drops. Stop it with `Ctrl-C`.

- Do not pair the car in the system Bluetooth settings, the firmware does not support pairing.
- The car accepts one connection at a time and stops advertising while connected. If the script keeps printing
  `car not found`, close other apps connected to it or run `bluetoothctl disconnect 00:A0:50:10:29:38`.

The same logs from CM4 (the default UART owner) are also on the USB serial port: `tio -b 115200 /dev/ttyACM0`.
