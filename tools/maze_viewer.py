#!/usr/bin/env python3
"""Show the maze the car is exploring, live, in a window.

The firmware logs every map change as a text line over the BLE log (same stream as tools/ble_serial.py):

    @R x y         map reset, start cell
    @C x y edges   cell (x, y) scanned; edges = 2 bits per side N, E, S, W (0 unknown, 1 wall, 2 open)
    @M x y dir m   car left cell (x, y) towards dir; that passage now has m Tremaux marks

The window shows walls, unknown sides, the state of every cell, the Tremaux marks and the robot.
Connect before the car starts moving. Close the window or press Ctrl-C to stop.

    python3 tools/maze_viewer.py                         # finds the car by name
    python3 tools/maze_viewer.py --address 00:A0:50:10:29:38
    python3 tools/maze_viewer.py --replay log.txt        # no car: replay a saved log (a UART capture works too)
    python3 tools/maze_viewer.py --log                   # also print the car's other log lines
"""

import argparse
import asyncio
import sys
import threading
from typing import Callable, Optional

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrow, Rectangle

from ble_serial import NUS_TX_UUID, find_car, status

NORTH, EAST, SOUTH, WEST = range(4)
DIR_NAME = "NESW"
DX = (0, 1, 0, -1)
DY = (-1, 0, 1, 0)  # y grows to the south (like the firmware map)

EDGE_UNKNOWN, EDGE_WALL, EDGE_OPEN = range(3)
MAX_MARKS = 2

COLOR_VISITED = "#dcedc8"
COLOR_FRONTIER = "#fff3c4"  # seen as open from a neighbour, not scanned yet
COLOR_CURRENT = "#ffc9c9"
MARK_STYLE = {  # marks -> (color, line width)
    0: ("#9e9e9e", 1.0),
    1: ("#f59e0b", 4.0),
    2: ("#d32f2f", 4.0),
}


class MazeMap:
    """The map built from the events. Thread safe: BLE thread writes, GUI thread reads."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.version = 0
        self._clear()

    def _clear(self) -> None:
        self.edges: dict[tuple[int, int], list[int]] = {}
        self.marks: dict[tuple[int, int], list[int]] = {}
        self.visited: set[tuple[int, int]] = set()
        self.start: Optional[tuple[int, int]] = None
        self.pos: Optional[tuple[int, int]] = None
        self.heading = NORTH
        self.moves = 0

    def _cell(self, table: dict, x: int, y: int) -> list[int]:
        return table.setdefault((x, y), [0, 0, 0, 0])

    def _set_pair(self, table: dict, x: int, y: int, d: int, value: int) -> None:
        """Set a value on a side of a cell and on the opposite side of its neighbour."""
        self._cell(table, x, y)[d] = value
        self._cell(table, x + DX[d], y + DY[d])[(d + 2) % 4] = value

    def feed(self, line: str) -> bool:
        """Apply one log line. Returns True if it was a map event."""
        if not line.startswith("@"):
            return False
        parts = line[1:].split()
        try:
            kind, nums = parts[0], [int(p) for p in parts[1:]]
            with self.lock:
                if kind == "R" and len(nums) == 2:
                    self._clear()
                    x, y = nums
                    self.start = self.pos = (x, y)
                    self._set_pair(self.edges, x, y, SOUTH, EDGE_OPEN)  # the entrance
                    self._set_pair(self.marks, x, y, SOUTH, MAX_MARKS)
                elif kind == "C" and len(nums) == 3:
                    x, y, edges = nums
                    for d in range(4):
                        self._set_pair(self.edges, x, y, d, (edges >> (2 * d)) & 3)
                    self.visited.add((x, y))
                elif kind == "M" and len(nums) == 4:
                    x, y, d, m = nums
                    self._set_pair(self.marks, x, y, d, m)
                    self.pos = (x + DX[d], y + DY[d])
                    self.heading = d
                    self.moves += 1
                else:
                    return False
                self.version += 1
        except (ValueError, IndexError):
            return False
        return True


class LineSplitter:
    """Notifications are 20 bytes or so: join them back into text lines."""

    def __init__(self, on_line: Callable[[str], None]) -> None:
        self.buf = ""
        self.on_line = on_line

    def feed(self, data: bytes) -> None:
        self.buf += data.decode("utf-8", errors="replace")
        *lines, self.buf = self.buf.split("\n")
        for line in lines:
            self.on_line(line.strip())


def draw(ax, maze: MazeMap) -> None:
    ax.clear()
    with maze.lock:
        edges = {k: list(v) for k, v in maze.edges.items()}
        marks = {k: list(v) for k, v in maze.marks.items()}
        visited = set(maze.visited)
        start, pos, heading, moves = maze.start, maze.pos, maze.heading, maze.moves

    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    if start is None:
        ax.set_title("waiting for the car ...")
        ax.set_xlim(-1, 1)
        ax.set_ylim(1, -1)
        return

    outside = (start[0], start[1] + 1)  # the cell behind the entrance: not part of the maze
    # visited cells, plus the cells a known open passage leads to (not the ones behind a wall)
    seen = {c for c, e in edges.items() if EDGE_OPEN in e}
    cells = (seen | visited | {start}) - {outside}
    xs = [c[0] for c in cells]
    ys = [c[1] for c in cells]
    x0, x1 = min(xs) - 1.5, max(xs) + 1.5
    y0, y1 = min(ys) - 1.5, max(ys) + 1.5
    # at least 6x6 cells so the first steps do not look huge
    x0, x1 = x0 - max(0, (6 - (x1 - x0)) / 2), x1 + max(0, (6 - (x1 - x0)) / 2)
    y0, y1 = y0 - max(0, (6 - (y1 - y0)) / 2), y1 + max(0, (6 - (y1 - y0)) / 2)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y1, y0)  # inverted: north is up

    # cell fills
    for (x, y) in cells:
        if (x, y) == pos:
            color = COLOR_CURRENT
        elif (x, y) in visited:
            color = COLOR_VISITED
        else:
            color = COLOR_FRONTIER
        ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor=color, edgecolor="none", zorder=1))
    ax.text(start[0], start[1], "S", ha="center", va="center", fontsize=14, color="#555", zorder=2)

    # passages: Tremaux marks, drawn between the two cell centres
    for (x, y), cell_edges in edges.items():
        for d in (EAST, SOUTH):  # each passage once
            if cell_edges[d] != EDGE_OPEN or (x + DX[d], y + DY[d]) == outside:
                continue
            m = min(marks.get((x, y), [0] * 4)[d], MAX_MARKS)
            color, width = MARK_STYLE[m]
            ax.plot([x, x + DX[d]], [y, y + DY[d]], color=color, linewidth=width, solid_capstyle="round", zorder=3)
    # the entrance behind the start cell: a short stub
    ax.plot([start[0], start[0]], [start[1], start[1] + 0.5], color=MARK_STYLE[MAX_MARKS][0], linewidth=2, zorder=3)

    # walls and unknown sides, on the cell borders
    sides = {  # direction -> (x offset, y offset) of the two border endpoints
        NORTH: ((-0.5, -0.5), (0.5, -0.5)),
        EAST: ((0.5, -0.5), (0.5, 0.5)),
        SOUTH: ((-0.5, 0.5), (0.5, 0.5)),
        WEST: ((-0.5, -0.5), (-0.5, 0.5)),
    }
    for (x, y) in visited:
        for d, state in enumerate(edges[(x, y)]):
            (ax0, ay0), (ax1, ay1) = sides[d]
            if state == EDGE_WALL:
                ax.plot([x + ax0, x + ax1], [y + ay0, y + ay1], color="black", linewidth=3.5, solid_capstyle="projecting", zorder=4)
            elif state == EDGE_UNKNOWN:
                ax.plot([x + ax0, x + ax1], [y + ay0, y + ay1], color="#9e9e9e", linewidth=1.5, linestyle=":", zorder=4)
    # robot
    if pos is not None:
        dx, dy = DX[heading] * 0.3, DY[heading] * 0.3
        ax.add_patch(
            FancyArrow(pos[0] - dx / 2, pos[1] - dy / 2, dx, dy, width=0.22, head_width=0.5, head_length=0.28,
                       length_includes_head=False, color="#1565c0", zorder=6)
        )

    legend = [
        Line2D([0], [0], color="black", lw=3.5, label="wall"),
        Line2D([0], [0], color="#9e9e9e", lw=1.5, ls=":", label="side not measured"),
        Line2D([0], [0], color=MARK_STYLE[0][0], lw=1, label="open, 0 marks"),
        Line2D([0], [0], color=MARK_STYLE[1][0], lw=4, label="1 mark"),
        Line2D([0], [0], color=MARK_STYLE[2][0], lw=4, label="2 marks (closed)"),
        Rectangle((0, 0), 1, 1, facecolor=COLOR_VISITED, label="visited"),
        Rectangle((0, 0), 1, 1, facecolor=COLOR_FRONTIER, label="seen, not visited"),
        Rectangle((0, 0), 1, 1, facecolor=COLOR_CURRENT, label="robot"),
    ]
    ax.legend(handles=legend, loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=9, frameon=False)
    ax.set_title(f"cell ({pos[0]}, {pos[1]})   heading {DIR_NAME[heading]}   moves {moves}   visited {len(visited)}")


# ----------------------------------------------------------------------------- input sources
def handle_line(maze: MazeMap, line: str, show_log: bool) -> None:
    if not maze.feed(line) and show_log and line:
        print(line, flush=True)


def replay(maze: MazeMap, path: str, show_log: bool) -> None:
    with open(path, "r", errors="replace") as f:
        for raw in f:
            handle_line(maze, raw.strip(), show_log)


async def ble_session(device, maze: MazeMap, show_log: bool) -> None:
    from bleak import BleakClient

    disconnected = asyncio.Event()
    splitter = LineSplitter(lambda line: handle_line(maze, line, show_log))

    async with BleakClient(device, disconnected_callback=lambda _c: disconnected.set()) as client:
        status(f"connected to {device.name or device.address}")
        await client.start_notify(NUS_TX_UUID, lambda _c, data: splitter.feed(bytes(data)))
        await disconnected.wait()


async def ble_forever(args, maze: MazeMap) -> None:
    while True:
        status(f"scanning for {args.address or args.name} ...")
        try:
            device = await find_car(args)
            if device is None:
                status("car not found, retrying")
                continue
            await ble_session(device, maze, args.log)
            status("disconnected, reconnecting")
        except Exception as exc:  # range, interference or the car resetting: just try again
            status(f"BLE error: {exc}, retrying")
        await asyncio.sleep(1.0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Show the maze the car explores, received over BLE.")
    parser.add_argument("--name", default="MazeCar-T5", help="advertised car name (default: %(default)s)")
    parser.add_argument("--address", help="connect by BLE address instead of name, e.g. 00:A0:50:10:29:38")
    parser.add_argument("--scan-timeout", type=float, default=10.0, help="seconds per scan attempt")
    parser.add_argument("--replay", metavar="FILE", help="read the map events from a text file instead of BLE")
    parser.add_argument("--log", action="store_true", help="also print the car's other log lines")
    parser.add_argument("--save", metavar="PNG", help="with --replay: save the final picture and exit (no window)")
    args = parser.parse_args()

    if args.save:
        plt.switch_backend("Agg")

    maze = MazeMap()
    if args.replay:
        replay(maze, args.replay, args.log)
    else:
        threading.Thread(target=lambda: asyncio.run(ble_forever(args, maze)), daemon=True).start()

    fig, ax = plt.subplots(figsize=(9, 6))
    fig.subplots_adjust(left=0.03, right=0.78, top=0.93, bottom=0.03)

    if args.save:
        draw(ax, maze)
        fig.savefig(args.save, dpi=110)
        return

    drawn = {"version": -1}

    def refresh() -> None:
        if drawn["version"] != maze.version or drawn["version"] < 0:
            drawn["version"] = maze.version
            draw(ax, maze)
            fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=200)
    timer.add_callback(refresh)
    timer.start()
    refresh()
    try:
        plt.show()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
