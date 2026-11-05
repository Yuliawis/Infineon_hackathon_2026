/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for CM4 in the the Dual CPU Empty 
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2020-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "car.h"
#include "motion.h"
#include "maze.h"
#include "head.h"
#include "distance.h"
#include "ledctrl.h"
#include "i2c_common.h"
#include "debug_uart.h"
#include "cm4_ipc.h"
#include "cm4_cmd.h"


#define MOTION_PRINT_PERIOD_MS   (500u)
#define DISTANCE_PRINT_PERIOD_MS (500u)

// Maze exploration (Tremax). The car builds the map with the ultrasonic sensor and prints it to the UART.
#define MAZE_EXPLORE             (1)       // 0 = old behavior (periodic distance/yaw prints only)
#define MAZE_START_DELAY_MS      (3000u)   // time to put the car down before it starts moving
#define MAZE_CELL_MM             (600u)    // cell size
#define MAZE_ENTRY_MM            (300u)    // from the start position to the center of the first cell. CALIBRATE.
#define MAZE_REDRAW_ANSI         (0)       // 1 = clear the terminal before each map (needs a terminal with ANSI)

#if MAZE_EXPLORE
static void mazeExploreStep(void);
static void flushBleLog(void);
#endif

int main(void) {
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS) {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    (void) Cy_IPC_Pipe_RegisterCallback(CY_IPC_EP_CYPIPE_ADDR, CM4_MessageCallback, CY_IPC_EP_CYPIPE_CM0_ADDR);

#if defined(RETARGET_CORE_CM4)
    if (Debug_Init() != CY_RSLT_SUCCESS) {
        CY_ASSERT(0);
    }
#endif
    DBG_PRINTF("CM4 started\n");

    if (I2C_Main_Start() != CY_SCB_I2C_SUCCESS) {
        CY_ASSERT(0);
    }

    Motor_Init();
    if (!Head_Init()) {
        DBG_PRINTF("Head init failed\n");
    }
    (void) Leds_Init();

    Timing_Init();

    // DISTANCE_MODE_AUTO filters in the background, DISTANCE_MODE_MANUAL needs Distance_Update() in the loop
    bool distanceReady = Distance_Init(DISTANCE_MODE_AUTO);
    if (distanceReady) {
        Distance_Start();
    } else {
        DBG_PRINTF("Ultrasonic init failed\n");
    }

    // Keep the car still during gyro calibration
    bool motionReady = Motion_Init();
    DBG_PRINTF(motionReady ? "Motion ready\n" : "Motion init failed\n");

#if MAZE_EXPLORE
    bool mazeReady = distanceReady && motionReady;
    if (!mazeReady) {
        DBG_PRINTF("Maze: distance or motion is not ready, exploration is off\n");
    }
#endif

    // Main loop. Your code goes here.
    for (;;) {
        // Check for new messages from CM0 core and process them. This is the most important task.
        if (CM4_isDataAvailableFromCM0()) {
            CM4_ProcessIncomingIPCMessage(CM4_GetCM0Message());
        }

#if MAZE_EXPLORE
        // One step is blocking (scan, turn, drive one cell), so the IPC messages above wait for it
        flushBleLog();
        if (mazeReady) {
            mazeExploreStep();
        }
#else
#if defined(BLE_LOG_ENABLED)
        CM4_ForwardLogsToCM0();
#endif

        if (distanceReady) {
            static uint32_t lastDistancePrintMs = 0;
            uint32_t nowMs = Timing_GetMilliseconds();

            (void) Distance_Update();  // does nothing in the automatic mode
            if (nowMs - lastDistancePrintMs >= DISTANCE_PRINT_PERIOD_MS) {
                uint16_t filteredMm;
                bool hasDistance = Distance_GetMm(&filteredMm);
                lastDistancePrintMs = nowMs;
                // -1 means that there is no value (no echo, or the distance is out of range)
                DBG_PRINTF("distance %d\n", hasDistance ? (int) filteredMm : -1);

                // LED + Distance test: red at 20 mm or closer, fading to green at 200 mm or farther. Tune it as you prefer.
                if (hasDistance) {
                    const uint16_t nearMm = 20;
                    const uint16_t farMm = 500;
                    const uint8_t maxLevel = 100;  // brightness cap (out of 255)

                    // Clamp the distance into the [nearMm, farMm] range
                    uint16_t mm = filteredMm;
                    if (mm < nearMm) {
                        mm = nearMm;
                    }

                    if (mm > farMm) {
                        mm = farMm;
                    }

                    // Linear blend: green grows with distance, red takes the rest
                    uint8_t green = (uint8_t) ((uint32_t) (mm - nearMm) * maxLevel / (farMm - nearMm));
                    uint8_t red = maxLevel - green;

                    Leds_FillSolidColor(green, red, 0);  // argument order is G, R, B
                } else {
                    Leds_FillSolidColor(0, 100, 100);  // no valid reading: purple (red + blue)
                }
                Leds_Update();
            }
        }


        if (motionReady && Motion_Update()) {
            static uint32_t lastPrintMs = 0;
            uint32_t nowMs = Timing_GetMilliseconds();
            if (nowMs - lastPrintMs >= MOTION_PRINT_PERIOD_MS) {
                lastPrintMs = nowMs;
                DBG_PRINTF("yaw %d deg, rate %d dps\n", Motion_GetYaw(), Motion_GetYawRate());
            }
        }
#endif  // MAZE_EXPLORE
    }
}

#if MAZE_EXPLORE
// The main loop forwards at most 254 bytes of log per iteration, and one iteration is a whole maze step,
// so push the map out right away.
static void flushBleLog(void) {
#if defined(BLE_LOG_ENABLED)
    uint8_t probe;
    for (uint8_t tries = 0; tries < 40u && Debug_BleLogPeek(&probe, 1u) > 0u; ++tries) {
        CM4_ForwardLogsToCM0();
        Cy_SysLib_Delay(5);
    }
#endif
}

// Status color for the whole strip (G, R, B)
static void setStatusLeds(uint8_t g, uint8_t r, uint8_t b) {
    Leds_FillSolidColor(g, r, b);
    Leds_Update();
}

// Print the whole map, with a step number and the chosen move.
static void printMaze(uint32_t step, Maze_Dir next) {
    static const char* const DIR_NAME[] = {"N", "E", "S", "W", "-"};
#if MAZE_REDRAW_ANSI
    DBG_PRINTF_LOCAL("\033[2J\033[H");
#endif
    DBG_PRINTF_LOCAL("\n=== Maze step %lu, next: %s ===\n", (unsigned long) step, DIR_NAME[next]);
    Maze_Print();
    flushBleLog();
}

// Drive forward. The head must look ahead and the distance filter must hold fresh front values:
// if the head had to move or the car turned (refresh), wait for that first.
static void driveForward(uint16_t mm, bool refresh) {
    if (Head_GetPan() != 0) {
        Head_Look(0, 0);
        Cy_SysLib_Delay(MAZE_HEAD_SETTLE_MS);
        refresh = true;
    }
    if (refresh) {
        Distance_Start();
        Cy_SysLib_Delay(MAZE_MEASURE_MS);
    }
    Car_DriveForward(mm);
}

// One step of the exploration: look around, update the map, choose the way by Tremaux, move one cell.
static void mazeExploreStep(void) {
    static bool started = false;
    static bool finished = false;
    static uint32_t step = 0;

    if (finished) {
        return;
    }

    if (!started) {
        started = true;
        DBG_PRINTF("Maze: start in %u ms\n", (unsigned) MAZE_START_DELAY_MS);
        Cy_SysLib_Delay(MAZE_START_DELAY_MS);
        driveForward(MAZE_ENTRY_MM, true);    // to the center of the first cell
        Maze_Init();
    }

    setStatusLeds(0, 0, 60);    // blue: scanning
    Maze_ScanCell();
    flushBleLog();  // the new cell goes to the laptop before the car starts moving
    Maze_Dir next = Maze_ChooseNext();
    printMaze(step++, next);

    if (next == MAZE_DIR_NONE) {
        finished = true;
        setStatusLeds(0, 60, 0);    // red: nowhere to go
        DBG_PRINTF("Maze: no way to go, finished\n");
        return;
    }

    setStatusLeds(60, 0, 0);    // green: moving
    int16_t turn = Maze_TurnDegrees(next);
    if (turn != 0) {
        Car_TurnBy(turn);
    }
    driveForward(MAZE_CELL_MM, turn != 0);
    (void) Maze_Advance(next);
    flushBleLog();
}
#endif  // MAZE_EXPLORE
