#!/usr/bin/env python3
"""Connect to the car over BLE and print its debug log (DBG_PRINTF output of both cores).

The firmware sends the log as Nordic UART Service (NUS) TX notifications when built with BLE_LOG=1 (default).
Reconnects automatically. Stop with Ctrl-C.

    python3 tools/ble_serial.py                         # finds the car by name
    python3 tools/ble_serial.py --address 00:A0:50:10:29:38
    python3 tools/ble_serial.py --csv run1.csv          # also save telemetry lines to a CSV file

Telemetry lines look like "[12.345] T hdg=90 dist=320" (time in seconds since boot, then key=value pairs).
With --csv each pair becomes one row "time_s,key,value"; -1 means the car had no value.
"""

import argparse
import asyncio
import csv
import re
import sys

from bleak import BleakClient, BleakScanner

NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # car -> laptop (notify)

TELEMETRY_RE = re.compile(r"^\[(\d+\.\d{3})\] T ((?:\w+=-?\d+ ?)+)$")


class TelemetryCsv:
    """Collects notification bytes into lines and writes the telemetry ones to a CSV file."""

    def __init__(self, path: str) -> None:
        self.file = open(path, "a", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        if self.file.tell() == 0:
            self.writer.writerow(["time_s", "key", "value"])
        self.pending = ""

    def feed(self, text: str) -> None:
        self.pending += text
        *lines, self.pending = self.pending.split("\n")
        for line in lines:
            match = TELEMETRY_RE.match(line.rstrip("\r"))
            if match:
                for pair in match.group(2).split():
                    key, value = pair.split("=", 1)
                    self.writer.writerow([match.group(1), key, value])
        self.file.flush()

    def reset(self) -> None:
        self.pending = ""  # a line cut by a disconnect is incomplete


def status(msg: str) -> None:
    print(f"[ble_serial] {msg}", file=sys.stderr, flush=True)


async def find_car(args):
    if args.address:
        return await BleakScanner.find_device_by_address(args.address, timeout=args.scan_timeout)
    return await BleakScanner.find_device_by_filter(
        lambda dev, adv: args.name in (dev.name, adv.local_name), timeout=args.scan_timeout
    )


async def show_log(device, telemetry) -> None:
    disconnected = asyncio.Event()

    def on_log(_char, data: bytearray) -> None:
        text = data.decode("utf-8", errors="replace")
        sys.stdout.write(text)
        sys.stdout.flush()
        if telemetry:
            telemetry.feed(text)

    async with BleakClient(device, disconnected_callback=lambda _c: disconnected.set()) as client:
        status(f"connected to {device.name or device.address}")
        await client.start_notify(NUS_TX_UUID, on_log)
        await disconnected.wait()


async def main() -> None:
    parser = argparse.ArgumentParser(description="Print the car's debug log received over BLE.")
    parser.add_argument("--name", default="MazeCar-T5", help="advertised car name (default: %(default)s)")
    parser.add_argument("--address", help="connect by BLE address instead of name, e.g. 00:A0:50:10:29:38")
    parser.add_argument("--scan-timeout", type=float, default=10.0, help="seconds per scan attempt")
    parser.add_argument("--csv", metavar="FILE", help="append telemetry lines to this CSV file")
    args = parser.parse_args()
    telemetry = TelemetryCsv(args.csv) if args.csv else None

    while True:
        status(f"scanning for {args.address or args.name} ...")
        try:
            device = await find_car(args)
            if device is None:
                status("car not found, retrying")
                continue
            if telemetry:
                telemetry.reset()
            await show_log(device, telemetry)
            status("disconnected, reconnecting")
        except Exception as exc:  # range, interference or the car resetting: just try again
            status(f"BLE error: {exc}, retrying")
        await asyncio.sleep(1.0)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
