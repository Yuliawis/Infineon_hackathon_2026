"""Bluetooth terminal for the car: shows the same log as the debug UART.

On the car, DBG_PRINTF() writes to the UART and to Bluetooth (Nordic UART Service) alike. This
script connects, prints every line with a timestamp, and writes the same into a log file.
Read only, nothing is sent to the car. Ctrl+C to quit.

Install once:
    pip install bleak

Run:
    python tools/ble_terminal.py                 # looks for the car by name
    python tools/ble_terminal.py --name "Other"  # another name
    python tools/ble_terminal.py --address AA:BB:CC:DD:EE:FF
"""

import argparse
import asyncio
import datetime
import pathlib
import sys

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    sys.exit("bleak is missing, install it with:  pip install bleak")

DEFAULT_NAME = "Volkswagen Polo"  # DeviceName in proj_cm0p/device.cybt
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # car -> PC, notifications


def timestamp() -> str:
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


class LineLog:
    """Notifications arrive in pieces of up to 20 bytes; this joins them back into lines."""

    def __init__(self, log_path: pathlib.Path):
        self.pending = ""
        self.file = log_path.open("a", encoding="utf-8")
        self.file.write(f"# connected {datetime.datetime.now().isoformat()}\n")

    def feed(self, data: bytearray) -> None:
        self.pending += data.decode("utf-8", errors="replace")
        while "\n" in self.pending:
            line, self.pending = self.pending.split("\n", 1)
            self.emit(line.rstrip("\r"))

    def emit(self, line: str) -> None:
        if not line:
            return
        text = f"{timestamp()}  {line}"
        print(text, flush=True)
        self.file.write(text + "\n")
        self.file.flush()

    def close(self) -> None:
        if self.pending:
            self.emit(self.pending)
        self.file.close()


async def find_car(name: str, address: str | None):
    if address:
        print(f"connecting to {address} ...")
        return address

    print(f'scanning for "{name}" ...')
    device = await BleakScanner.find_device_by_name(name, timeout=15.0)
    if device is None:
        print("not found. Devices nearby:")
        for d in await BleakScanner.discover(timeout=5.0):
            print(f"  {d.address}  {d.name}")
        sys.exit("pass the right one with --name or --address")
    print(f"found {device.address}")
    return device


async def main() -> None:
    parser = argparse.ArgumentParser(description="BLE log terminal for the car")
    parser.add_argument("--name", default=DEFAULT_NAME, help="advertised name of the car")
    parser.add_argument("--address", help="BLE address, skips the scan by name")
    parser.add_argument("--log", help="log file (default: logs/ble_<date>_<time>.log)")
    args = parser.parse_args()

    log_path = pathlib.Path(args.log) if args.log else (
        pathlib.Path("logs") / datetime.datetime.now().strftime("ble_%Y%m%d_%H%M%S.log"))
    log_path.parent.mkdir(parents=True, exist_ok=True)

    target = await find_car(args.name, args.address)
    disconnected = asyncio.Event()

    async with BleakClient(target, disconnected_callback=lambda _c: disconnected.set()) as client:
        log = LineLog(log_path)
        print(f"connected, logging to {log_path}  (Ctrl+C to quit)")
        await client.start_notify(NUS_TX_UUID, lambda _h, data: log.feed(data))
        try:
            await disconnected.wait()
            print("disconnected")
        finally:
            log.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nbye")
