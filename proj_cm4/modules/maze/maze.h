#ifndef MAZE_H
#define MAZE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "distance.h"

// Tremaux maze map: 60x60 cm cells, marks (0..2) on passages. NORTH = direction the car faces at the start.
#define MAZE_SIZE    (30u)
#define MAZE_START_X (MAZE_SIZE / 2u)
#define MAZE_START_Y (MAZE_SIZE / 2u)

#define MAZE_MAX_MARKS (2u)

// Map changes are logged as text lines (they travel over the BLE log, see tools/maze_viewer.py):
//   @R x y         map reset, start cell
//   @C x y edges   cell scanned; edges = 2 bits per side N,E,S,W (0 unknown, 1 wall, 2 open)
//   @M x y dir m   car left cell (x,y) towards dir; the passage now has m marks

// Wall if closer than this (mm). Calibrate on the field.
#define MAZE_FRONT_WALL_MM (400u)
#define MAZE_SIDE_WALL_MM  (450u)

// Servo travel time, then distance filter fill time
#define MAZE_HEAD_SETTLE_MS (500u)
#define MAZE_MEASURE_MS     (DISTANCE_FILL_MS)  // follows the filter size and the sensor period

// 1 = every scan turns the head and measures all three sides, also in visited cells and on sides known from
// the map (about 2 s more per step, a second chance to see a wall). 0 = only the unknown sides of new cells.
#define MAZE_SCAN_ALWAYS (1)

typedef enum { MAZE_NORTH = 0, MAZE_EAST, MAZE_SOUTH, MAZE_WEST, MAZE_DIR_NONE } Maze_Dir;

///////////////////// MAP /////////////////////////////////////////////////////
void Maze_Init(void);

// Walls are relative to the car heading
void Maze_SetWalls(bool wall_front, bool wall_left, bool wall_right);

// Fix one side of the current cell to a wall (absolute direction), e.g. when the car could not drive there
void Maze_MarkWall(Maze_Dir dir);

// Measures left, right and front with the head (all of them with MAZE_SCAN_ALWAYS, else only the unknown
// sides of a new cell). The new readings replace what the map had. Blocking.
void Maze_ScanCell(void);

///////////////////// TREMAUX /////////////////////////////////////////////////
// Tremaux: next direction, or MAZE_DIR_NONE
Maze_Dir Maze_ChooseNext(void);

// Degrees for Car_TurnBy(): 0, +90, -90, 180
int16_t Maze_TurnDegrees(Maze_Dir dir);

// Call after the car arrived in the next cell
bool Maze_Advance(Maze_Dir dir);

///////////////////// STATE ///////////////////////////////////////////////////
uint8_t Maze_GetX(void);
uint8_t Maze_GetY(void);
Maze_Dir Maze_GetHeading(void);

///////////////////// DEBUG ///////////////////////////////////////////////////
void Maze_Print(void);

#ifdef __cplusplus
}
#endif

#endif /* MAZE_H */
