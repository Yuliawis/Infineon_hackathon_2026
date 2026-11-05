#include <stdint.h>
#include "explore.h"
#include "maze.h"
#include "car.h"
#include "head.h"
#include "distance.h"
#include "ledctrl.h"
#include "debug_uart.h"
#include "cm4_cmd.h"

// Status color for the whole strip (G, R, B)
static void setStatusLeds(uint8_t g, uint8_t r, uint8_t b) {
    Leds_FillSolidColor(g, r, b);
    Leds_Update();
}

// Hand the logs of this step to CM0+ right away: the main loop forwards at most 254 bytes per iteration,
// and one iteration is a whole step. Map events (@R/@C/@M) must reach the viewer before the car moves on.
static void flushLogs(void) {
#if defined(BLE_LOG_ENABLED)
    CM4_FlushLogsToCM0();
#endif
}

// Print the whole map, with a step number and the chosen move.
static void printMaze(uint32_t step, Maze_Dir next) {
#if EXPLORE_PRINT_MAP
    static const char* const DIR_NAME[] = {"N", "E", "S", "W", "-"};
#if EXPLORE_REDRAW_ANSI
    DBG_PRINTF_LOCAL("\033[2J\033[H");
#endif
    DBG_PRINTF_LOCAL("\n=== Maze step %lu, next: %s ===\n", (unsigned long) step, DIR_NAME[next]);
    Maze_Print();
#else
    (void) step;
    (void) next;
#endif
    flushLogs();
}

// Drive forward. The head must look ahead and the distance filter must hold fresh front values:
// if the head had to move or the car turned (refresh), wait for that first.
static void driveForward(uint16_t mm, bool refresh) {
    if (Head_GetPan() != 0) {
        Head_Look(0, 0);
        Car_Wait(MAZE_HEAD_SETTLE_MS);  // Car_Wait keeps the gyro and the debug print running
        refresh = true;
    }
    if (refresh) {
        Distance_Start();
        Car_Wait(MAZE_MEASURE_MS);
    }
    Car_DriveForward(mm);
}

bool Explore_Step(void) {
    static bool started = false;
    static bool finished = false;
    static uint32_t step = 0;

    if (finished) {
        return false;
    }

    if (!started) {
        started = true;
        DBG_PRINTF("Maze: start in %u ms\n", (unsigned) EXPLORE_START_DELAY_MS);
        Car_Wait(EXPLORE_START_DELAY_MS);
        Car_ResetHeading();                    // the car stands aligned to the walls: this is heading 0
        driveForward(EXPLORE_ENTRY_MM, true);  // to the center of the first cell
        Maze_Init();
    }

    setStatusLeds(0, 0, 60);  // blue: scanning
    Maze_ScanCell();
    flushLogs();  // the new cell goes to the laptop before the car starts moving
    Maze_Dir next = Maze_ChooseNext();
    printMaze(step++, next);

    if (next == MAZE_DIR_NONE) {
        finished = true;
        setStatusLeds(0, 60, 0);  // red: nowhere to go
        DBG_PRINTF("Maze: no way to go, finished\n");
        return false;
    }

    setStatusLeds(60, 0, 0);  // green: moving
    int16_t turn = Maze_TurnDegrees(next);
    if (turn != 0) {
        Car_TurnBy(turn);
    }
    driveForward(EXPLORE_CELL_MM, turn != 0);
    (void) Maze_Advance(next);
    flushLogs();
    return true;
}
