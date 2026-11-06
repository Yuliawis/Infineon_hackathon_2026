#ifndef EXPLORE_H
#define EXPLORE_H

#include <stdbool.h>

// Maze exploration run (Tremaux): waits, drives into the first cell, then one cell per step.
// Uses maze (map and decisions), car (moves), head, distance and the LED strip (status color).
// Needs working motion and distance sensors.

#define EXPLORE_START_DELAY_MS (5000u)  // from power-on to the first move (gyro calibration included)
#define EXPLORE_CELL_MM        (600u)   // cell size
#define EXPLORE_ENTRY_MM       (300u)   // from the start position to the center of the first cell. CALIBRATE.

// Without a wall ahead the car can't measure the drive, so it drives for a fixed time. CALIBRATE on the car:
// the time one cell takes at DRIVE_SPEED (it also sets the safety time limit of the drives with the sensor).
#define EXPLORE_CELL_TIME_MS   (1050u)
#define EXPLORE_ENTRY_TIME_MS  (EXPLORE_CELL_TIME_MS * EXPLORE_ENTRY_MM / EXPLORE_CELL_MM)  // or set it by hand
// Before each scan: if the car points more than this away from its heading, it turns back first, so the
// walls are measured square. Above TURN_TOLERANCE_DEG (4), else it would fix after every turn.
#define EXPLORE_HEADING_FIX_DEG (6)
#define EXPLORE_PRINT_MAP     (0)      // 1 = print the maze map after each step
#define EXPLORE_REDRAW_ANSI    (0)      // 1 = clear the terminal before each map (needs a terminal with ANSI)

// One blocking step: look around, update the map, choose the way, turn and drive one cell.
// The first call also does the start (delay, entry). Returns false once the run is finished.
bool Explore_Step(void);

#endif /* EXPLORE_H */
