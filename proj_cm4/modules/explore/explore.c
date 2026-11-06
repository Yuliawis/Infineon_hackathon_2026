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

#define LED_FLASH_MS (300u)

// Drive mode from Car_DriveForward(): green = by the sensor, purple = blind on time, white = wall too close
static void onDriveMode(Car_DriveMode mode) {
    switch (mode) {
        case CAR_DRIVE_SENSOR: setStatusLeds(60, 0, 0); break;
        case CAR_DRIVE_TIMER:  setStatusLeds(0, 60, 60); break;
        case CAR_DRIVE_SHORT:  setStatusLeds(60, 60, 60); break;
        default: break;
    }
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
// No wall seen ahead: the car drives blind for blind_ms (one cell), the next step goes on from the new cell.
static bool driveForward(uint16_t mm, uint32_t blind_ms, bool refresh) {
    if (Head_GetPan() != 0) {
        Head_Look(0, 0);
        Car_Wait(MAZE_HEAD_SETTLE_MS);
        refresh = true;
    }
    if (refresh) {
        Distance_Start();
        Car_Wait(MAZE_MEASURE_MS);
    }
    return Car_DriveForward(mm, EXPLORE_CELL_MM, blind_ms);  // with a wall ahead: stop in a cell center
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
        // Look ahead now, so the head settles and the distance filter fills during the delay, not after it
        Head_Look(0, 0);
        // The delay counts from power-on (Timing_Init), so the gyro calibration is part of it, not added to it
        uint32_t now_ms = Timing_GetMilliseconds();
        uint32_t wait_ms = (now_ms < EXPLORE_START_DELAY_MS) ? EXPLORE_START_DELAY_MS - now_ms : 0u;
        DBG_PRINTF("Maze: start in %lu ms\n", (unsigned long) wait_ms);
        Car_Wait(wait_ms);
        Car_SetDriveModeHook(onDriveMode);
        Car_ResetHeading();                    // the car stands aligned to the walls: this is heading 0
        // to the center of the first cell; no refresh: the filter kept measuring ahead during the delay
        (void) driveForward(EXPLORE_ENTRY_MM, EXPLORE_ENTRY_TIME_MS, false);
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
    if (driveForward(EXPLORE_CELL_MM, EXPLORE_CELL_TIME_MS, turn != 0)) {
        (void) Maze_Advance(next);
    } else {
        DBG_PRINTF("Maze: wall ahead too close, the car stayed in this cell\n");
        Maze_MarkWall(next);  // otherwise the same direction would be chosen again and again
        setStatusLeds(60, 60, 60);  // white flash: did not reach the next cell
        Car_Wait(LED_FLASH_MS);
    }
    flushLogs();
    return true;
}
