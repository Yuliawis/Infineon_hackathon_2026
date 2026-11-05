#ifndef MAZE_H
#define MAZE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Tremaux maze map: 60x60 cm cells, marks (0..2) on passages. NORTH = direction the car faces at the start.
#define MAZE_SIZE    (30u)
#define MAZE_START_X (MAZE_SIZE / 2u)
#define MAZE_START_Y (MAZE_SIZE / 2u)

#define MAZE_MAX_MARKS (2u)

// Wall if closer than this (mm). Calibrate on the field.
#define MAZE_FRONT_WALL_MM (400u)
#define MAZE_SIDE_WALL_MM  (450u)

// Servo travel time, then distance filter fill time
#define MAZE_HEAD_SETTLE_MS (500u)
#define MAZE_MEASURE_MS     (250u)

typedef enum { MAZE_NORTH = 0, MAZE_EAST, MAZE_SOUTH, MAZE_WEST, MAZE_DIR_NONE } Maze_Dir;

///////////////////// MAP /////////////////////////////////////////////////////
void Maze_Init(void);

// Walls are relative to the car heading
void Maze_SetWalls(bool wall_front, bool wall_left, bool wall_right);

// Measures only the unknown sides. Blocking.
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
