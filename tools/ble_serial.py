#!/usr/bin/env python3
"""Connect to the car over BLE and print its debug log (DBG_PRINTF output of both cores).

The firmware sends the log as Nordic UART Service (NUS) TX notifications when built with BLE_LOG=1 (default).
Reconnects automatically. Stop with Ctrl-C.

    python3 tools/ble_serial.py                         # finds the car by name
    python3 tools/ble_serial.py --address 00:A0:50:10:29:38
"""

import argparse
import asyncio
import sys

from bleak import BleakClient, BleakScanner

NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # car -> laptop (notify)


def status(msg: str) -> None:
    print(f"[ble_serial] {msg}", file=sys.stderr, flush=True)


async def find_car(args):
    if args.address:
        return await BleakScanner.find_device_by_address(args.address, timeout=args.scan_timeout)
    return await BleakScanner.find_device_by_filter(
        lambda dev, adv: args.name in (dev.name, adv.local_name), timeout=args.scan_timeout
    )


async def show_log(device) -> None:
    disconnected = asyncio.Event()

    def on_log(_char, data: bytearray) -> None:
        sys.stdout.write(data.decode("utf-8", errors="replace"))
        sys.stdout.flush()

    async with BleakClient(device, disconnected_callback=lambda _c: disconnected.set()) as client:
        status(f"connected to {device.name or device.address}")
        await client.start_notify(NUS_TX_UUID, on_log)
        await disconnected.wait()


async def main() -> None:
    parser = argparse.ArgumentParser(description="Print the car's debug log received over BLE.")
    parser.add_argument("--name", default="MazeCar-T5", help="advertised car name (default: %(default)s)")
    parser.add_argument("--address", help="connect by BLE address instead of name, e.g. 00:A0:50:10:29:38")
    parser.add_argument("--scan-timeout", type=float, default=10.0, help="seconds per scan attempt")
    args = parser.parse_args()

    while True:
        status(f"scanning for {args.address or args.name} ...")
        try:
            device = await find_car(args)
            if device is None:
                status("car not found, retrying")
                continue
            await show_log(device)
            status("disconnected, reconnecting")
        except Exception as exc:  # range, interference or the car resetting: just try again
            status(f"BLE error: {exc}, retrying")
        await asyncio.sleep(1.0)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
