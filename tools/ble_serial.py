#!/usr/bin/env python3
"""Connect to the car over BLE and print its debug log (DBG_PRINTF output of both cores).

The firmware sends the log as Nordic UART Service (NUS) TX notifications when built with BLE_LOG=1 (default).
Reconnects automatically. Stop with Ctrl-C (or close the window with --ui).

    python3 tools/ble_serial.py                         # finds the car by name
    python3 tools/ble_serial.py --address 00:A0:50:10:29:39
    python3 tools/ble_serial.py --csv run1.csv          # also save telemetry lines to a CSV file
    python3 tools/ble_serial.py --ui                    # live window: current values, compass, plots
    python3 tools/ble_serial.py --ui --window 30        # plots show the last 30 s (0 = whole run)
    python3 tools/ble_serial.py --ui --replay run.log   # show a saved log instead of BLE

Telemetry lines look like "[12.345] T hdg=90 dist=320" (time in seconds since boot, then key=value pairs).
With --csv each pair becomes one row "time_s,key,value"; -1 means the car had no value.
With --ui every key gets its own plot, so new telemetry fields show up without changing this script.
"""

import argparse
import asyncio
import csv
import math
import re
import sys
import threading
import time
from collections import deque
from typing import Callable, Optional

from bleak import BleakClient, BleakScanner

NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # car -> laptop (notify)

# The "[s.mmm] " stamp is optional (DEBUG_TIMESTAMP=0 in the firmware): then the laptop clock is used
TELEMETRY_RE = re.compile(r"^(?:\[(\d+(?:\.\d+)?)\] )?T ((?:\w+=-?\d+ ?)+)$")
NO_VALUE = -1  # the firmware sends -1 when it has no value (sensor not ready, no echo, ...)

# Known keys: (plot title, y range or None for auto). Unknown keys are plotted as they are.
KEY_INFO = {
    "hdg": ("heading, deg (+ = left)", (-180.0, 180.0)),
    "dist": ("front distance, mm", None),
}


def parse_telemetry(line: str) -> Optional[tuple[Optional[float], dict[str, int]]]:
    """'[12.345] T hdg=90 dist=320' -> (12.345, {'hdg': 90, 'dist': 320}), or None if not telemetry."""
    match = TELEMETRY_RE.match(line.rstrip("\r"))
    if not match:
        return None
    values = {}
    for pair in match.group(2).split():
        key, value = pair.split("=", 1)
        values[key] = int(value)
    return (float(match.group(1)) if match.group(1) else None), values


class LineSplitter:
    """Notifications are ~20 bytes: join them back into text lines."""

    def __init__(self, on_line: Callable[[str], None]) -> None:
        self.pending = ""
        self.on_line = on_line

    def feed(self, text: str) -> None:
        self.pending += text
        *lines, self.pending = self.pending.split("\n")
        for line in lines:
            self.on_line(line.rstrip("\r"))

    def reset(self) -> None:
        self.pending = ""  # a line cut by a disconnect is incomplete


class TelemetryCsv:
    """Writes the telemetry lines to a CSV file, one row per key=value pair."""

    def __init__(self, path: str) -> None:
        self.file = open(path, "a", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        if self.file.tell() == 0:
            self.writer.writerow(["time_s", "key", "value"])
        self.t0 = time.monotonic()

    def line(self, line: str) -> None:
        parsed = parse_telemetry(line)
        if parsed is None:
            return
        t, values = parsed
        stamp = f"{t:.3f}" if t is not None else f"{time.monotonic() - self.t0:.3f}"
        for key, value in values.items():
            self.writer.writerow([stamp, key, value])
        self.file.flush()


class TelemetryStore:
    """Telemetry history for the UI. Thread safe: the BLE thread writes, the GUI thread reads."""

    def __init__(self, log_lines: int = 14) -> None:
        self.lock = threading.Lock()
        self.version = 0
        self.series: dict[str, tuple[list[float], list[float]]] = {}  # key -> (times, values; NaN = no value)
        self.last: dict[str, int] = {}
        self.log: deque[str] = deque(maxlen=log_lines)
        self.samples = 0
        self.last_rx = None  # laptop time of the last received line
        self.link = "starting"
        self.connected = False
        self.t0 = time.monotonic()

    def set_link(self, text: str, connected: bool) -> None:
        with self.lock:
            self.link = text
            self.connected = connected
            self.version += 1

    def line(self, line: str) -> None:
        parsed = parse_telemetry(line)
        with self.lock:
            self.last_rx = time.monotonic()
            if parsed is None:
                if line.strip():
                    self.log.append(line.strip())
                    self.version += 1
                return
            t, values = parsed
            if t is None:
                t = time.monotonic() - self.t0
            # Time went back: the car was reset. Start the plots again.
            newest = max((s[0][-1] for s in self.series.values() if s[0]), default=None)
            if newest is not None and t < newest - 1.0:
                self.series.clear()
                self.last.clear()
                self.log.append("--- car restarted ---")
            for key, value in values.items():
                times, vals = self.series.setdefault(key, ([], []))
                times.append(t)
                vals.append(math.nan if value == NO_VALUE else float(value))
                self.last[key] = value
            self.samples += 1
            self.version += 1

    def snapshot(self):
        with self.lock:
            return (
                {k: (list(ts), list(vs)) for k, (ts, vs) in self.series.items()},
                dict(self.last),
                list(self.log),
                self.samples,
                self.last_rx,
                self.link,
                self.connected,
            )


def status(msg: str) -> None:
    print(f"[ble_serial] {msg}", file=sys.stderr, flush=True)


async def find_car(args):
    if args.address:
        return await BleakScanner.find_device_by_address(args.address, timeout=args.scan_timeout)
    return await BleakScanner.find_device_by_filter(
        lambda dev, adv: args.name in (dev.name, adv.local_name), timeout=args.scan_timeout
    )


async def show_log(device, on_text: Callable[[str], None], on_connected: Callable[[], None]) -> None:
    disconnected = asyncio.Event()

    def on_log(_char, data: bytearray) -> None:
        text = data.decode("utf-8", errors="replace")
        sys.stdout.write(text)
        sys.stdout.flush()
        on_text(text)

    async with BleakClient(device, disconnected_callback=lambda _c: disconnected.set()) as client:
        status(f"connected to {device.name or device.address}")
        on_connected()
        await client.start_notify(NUS_TX_UUID, on_log)
        await disconnected.wait()


async def ble_forever(args, splitter: LineSplitter, store: Optional[TelemetryStore]) -> None:
    def link(text: str, connected: bool) -> None:
        if store:
            store.set_link(text, connected)

    while True:
        status(f"scanning for {args.address or args.name} ...")
        link(f"scanning for {args.address or args.name}", False)
        try:
            device = await find_car(args)
            if device is None:
                status("car not found, retrying")
                continue
            splitter.reset()
            link(f"connecting to {device.name or device.address}", False)
            await show_log(device, splitter.feed, lambda: link(f"connected to {device.name or device.address}", True))
            status("disconnected, reconnecting")
            link("disconnected, reconnecting", False)
        except Exception as exc:  # range, interference or the car resetting: just try again
            status(f"BLE error: {exc}, retrying")
            link("BLE error, retrying", False)
        await asyncio.sleep(1.0)


# ----------------------------------------------------------------------------- UI
def run_ui(store: TelemetryStore, window_s: float, start_ble: Optional[Callable[[], None]],
           save: Optional[str] = None) -> None:
    import matplotlib

    if save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec

    fig = plt.figure(figsize=(13, 7.5))
    fig.canvas.manager.set_window_title("Car telemetry")
    state = {"keys": None, "version": -1, "axes": {}, "lines": {}, "compass": None, "needle": None, "text": None}

    def build(keys: list[str]) -> None:
        fig.clear()
        rows = max(len(keys), 1)
        gs = GridSpec(rows * 2, 2, figure=fig, width_ratios=[2.6, 1], left=0.06, right=0.95, top=0.91,
                      bottom=0.07, hspace=0.9, wspace=0.12)
        state["axes"], state["lines"] = {}, {}
        first = last_ax = None
        for i, key in enumerate(keys):
            ax = fig.add_subplot(gs[i * 2:i * 2 + 2, 0], sharex=first)
            first = first or ax
            last_ax = ax
            title, _ = KEY_INFO.get(key, (key, None))
            ax.set_title(title, loc="left", fontsize=10)
            ax.grid(True, alpha=0.3)
            (line,) = ax.plot([], [], lw=1.4, color=f"C{i}")
            state["axes"][key], state["lines"][key] = ax, line
        if last_ax is not None:
            last_ax.set_xlabel("car time, s")
        else:
            ax = fig.add_subplot(gs[:, 0])
            ax.set_axis_off()
            state["axes"]["_empty"] = ax

        # right column: compass on top, text panel below
        compass = fig.add_subplot(gs[0:rows, 1], projection="polar")
        compass.set_theta_zero_location("N")
        compass.set_theta_direction(1)  # counter-clockwise: positive heading = turned left
        compass.set_yticks([])
        compass.set_ylim(0, 1)
        compass.set_xticks([math.radians(a) for a in (0, 90, 180, 270)])
        compass.set_xticklabels(["0", "+90 L", "180", "-90 R"], fontsize=8)
        compass.set_title("heading", fontsize=10, pad=14)
        (needle,) = compass.plot([], [], lw=3, color="C0")
        state["compass"], state["needle"] = compass, needle

        panel = fig.add_subplot(gs[rows:, 1])
        panel.set_axis_off()
        state["text"] = panel.text(0, 1, "", va="top", ha="left", family="monospace", fontsize=9,
                                   transform=panel.transAxes)
        state["keys"] = keys

    def visible(ts: list[float], vs: list[float], key: str) -> tuple[list[float], list[float]]:
        if key == "hdg":  # do not draw a vertical line where the heading wraps from +180 to -180
            out_t, out_v = [], []
            for i, (t, v) in enumerate(zip(ts, vs)):
                if i and not math.isnan(v) and not math.isnan(vs[i - 1]) and abs(v - vs[i - 1]) > 180:
                    out_t.append(t)
                    out_v.append(math.nan)
                out_t.append(t)
                out_v.append(v)
            return out_t, out_v
        return ts, vs

    def refresh() -> None:
        series, last, log, samples, last_rx, link, connected = store.snapshot()
        keys = sorted(series, key=lambda k: (k not in KEY_INFO, list(KEY_INFO).index(k) if k in KEY_INFO else 0, k))
        if keys != state["keys"]:
            build(keys)
            state["version"] = -1
        changed = store.version != state["version"]
        state["version"] = store.version

        if changed:
            t_end = max((ts[-1] for ts, _ in series.values() if ts), default=0.0)
            t_start = t_end - window_s if window_s > 0 else min((ts[0] for ts, _ in series.values() if ts), default=0.0)
            for key in keys:
                ts, vs = series[key]
                lo = 0
                while lo < len(ts) and ts[lo] < t_start:
                    lo += 1
                pt, pv = visible(ts[lo:], vs[lo:], key)
                line, ax = state["lines"][key], state["axes"][key]
                line.set_data(pt, pv)
                ax.set_xlim(t_start, max(t_end, t_start + 1.0))
                fixed = KEY_INFO.get(key, (key, None))[1]
                finite = [v for v in pv if not math.isnan(v)]
                if fixed:
                    ax.set_ylim(*fixed)
                elif finite:
                    lo_v, hi_v = min(finite), max(finite)
                    pad = max((hi_v - lo_v) * 0.1, 1.0)
                    ax.set_ylim(lo_v - pad, hi_v + pad)

            hdg = last.get("hdg")
            if hdg is not None and hdg != NO_VALUE:
                state["needle"].set_data([math.radians(hdg)] * 2, [0, 0.9])
            else:
                state["needle"].set_data([], [])

        # the text panel is redrawn every tick: the "last data" age keeps changing
        age = f"{time.monotonic() - last_rx:.1f} s ago" if last_rx is not None else "never"
        lines = [f"link: {link[:40]}", f"last data: {age}", f"samples: {samples}", ""]
        if last:
            for key in keys:
                value = last[key]
                lines.append(f"{key:>6} = {'--' if value == NO_VALUE else value}")
        elif connected:
            lines.append("connected, no telemetry yet")
        else:
            lines.append("no telemetry yet")
        lines += ["", "log:"] + [entry[:46] for entry in log]
        state["text"].set_text("\n".join(lines))
        stale = last_rx is None or time.monotonic() - last_rx > 2.0
        state["text"].set_color("#b71c1c" if (not connected or stale) and start_ble else "black")
        fig.canvas.draw_idle()

    if save:
        refresh()
        fig.savefig(save, dpi=100)
        return
    if start_ble:
        start_ble()
    timer = fig.canvas.new_timer(interval=250)
    timer.add_callback(refresh)
    timer.start()
    refresh()
    try:
        plt.show()
    except KeyboardInterrupt:
        pass


# ----------------------------------------------------------------------------- main
def main() -> None:
    parser = argparse.ArgumentParser(description="Print the car's debug log received over BLE.")
    parser.add_argument("--name", default="MazeCar-T5-2", help="advertised car name (default: %(default)s)")
    parser.add_argument("--address", help="connect by BLE address instead of name, e.g. 00:A0:50:10:29:39")
    parser.add_argument("--scan-timeout", type=float, default=10.0, help="seconds per scan attempt")
    parser.add_argument("--csv", metavar="FILE", help="append telemetry lines to this CSV file")
    parser.add_argument("--ui", action="store_true", help="show telemetry in a window: values, compass, plots")
    parser.add_argument("--window", type=float, default=60.0, help="with --ui: seconds shown in the plots, 0 = all")
    parser.add_argument("--replay", metavar="FILE", help="with --ui: show a saved log file instead of BLE")
    parser.add_argument("--save", metavar="PNG", help="with --replay: save the picture and exit (no window)")
    args = parser.parse_args()
    if args.replay and not args.ui:
        parser.error("--replay needs --ui")
    if args.save and not args.replay:
        parser.error("--save needs --replay")

    csv_out = TelemetryCsv(args.csv) if args.csv else None
    store = TelemetryStore() if args.ui else None

    def on_line(line: str) -> None:
        if csv_out:
            csv_out.line(line)
        if store:
            store.line(line)

    splitter = LineSplitter(on_line)

    if not args.ui:
        try:
            asyncio.run(ble_forever(args, splitter, None))
        except KeyboardInterrupt:
            pass
        return

    if args.replay:
        store.set_link(f"replay of {args.replay}", True)
        with open(args.replay, "r", errors="replace") as f:
            for raw in f:
                splitter.feed(raw)
        run_ui(store, args.window, None, args.save)
        return

    def start_ble() -> None:
        threading.Thread(target=lambda: asyncio.run(ble_forever(args, splitter, store)), daemon=True).start()

    run_ui(store, args.window, start_ble)


if __name__ == "__main__":
    main()
