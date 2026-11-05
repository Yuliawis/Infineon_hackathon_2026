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
#include "head.h"
#include "music.h"
#include "ledctrl.h"
#include "i2c_common.h"
#include "debug_uart.h"
#include "cm4_ipc.h"

/* The autonomous maze run. sense, drive and navigator each need their update called from the main
 * loop; config.h holds the tunables. */
#include "config.h"
#include "drive.h"
#include "navigator.h"
#include "sense.h"

#include "cm0_cm4_shared_cmd.h"
#include "cm4_command_list.h"

/* Only used by the demo prints in the loop below, which are commented out while the car drives
 * itself: printing on every pass would slow the loop down and the gyro integration with it. */
/* #define MOTION_PRINT_PERIOD_MS   (500u) */
/* #define DISTANCE_PRINT_PERIOD_MS (500u) */

ipc_msg_t ipcMsgForCM0 = {/* IPC structure to be sent to CM0 */
                          .clientId = IPC_CM4_TO_CM0_CLIENT_ID,
                          .userCode = 0,
                          .intrMask = CY_SYS_CYPIPE_INTR_MASK,
                          .buffer = {},
                          .len = 0};

// Private function declarations for handling IPC messages and CM4 commands.
static void processIncomingIPCMessage(ipc_msg_t* msg);
static void processCM4Command(enum cm4CommandList cmd);

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
    Sound_Init();
    (void) Leds_Init();

    Timing_Init();

    /* The ultrasonic sensor now belongs to the sense module, which Navigator_Init() sets up below.
     * Starting it here as well would clear the filter behind that module's back. */
    /*
    // DISTANCE_MODE_AUTO filters in the background, DISTANCE_MODE_MANUAL needs Distance_Update() in the loop
    bool distanceReady = Distance_Init(DISTANCE_MODE_AUTO);
    if (distanceReady) {
        Distance_Start();
    } else {
        DBG_PRINTF("Ultrasonic init failed\n");
    }
    */

    // Keep the car still during gyro calibration
    bool motionReady = Motion_Init();
    DBG_PRINTF(motionReady ? "Motion ready\n" : "Motion init failed\n");

    // Temporary head test (after gyro calibration, so the movement does not disturb it)
    /*
    DBG_PRINTF("Head test\n");
    Head_Look(90, 0);
    Cy_SysLib_Delay(1800);
    Head_Look(-90, 0);
    Cy_SysLib_Delay(1800);
    Head_Look(0, 0);
    Cy_SysLib_Delay(800);
    Head_Look(0, 10);
    Cy_SysLib_Delay(1800);
    Head_Look(0, -20);
    Cy_SysLib_Delay(1800);
    Head_Look(0, 0);
    Cy_SysLib_Delay(1800);
    DBG_PRINTF("Head test done\n");
    */


    // Music_FurElise();

    /*
    Motor_Move(2000, 0, 0, 0);
    CyDelay(1000);
    Motor_Move(0, 2000, 0, 0);
    CyDelay(1000);
    Motor_Move(0, 0, 2000, 0);
    CyDelay(1000);
    Motor_Move(0, 0, 0, 2000);
    CyDelay(1000);
    Motor_Move(0, 0, 0, 0);
    CyDelay(1000);
    */

    /* Blocking turn demo, replaced by the non-blocking turn inside the drive module */
    /*
    Car_TurnBy(45);
    Car_TurnBy(-45);
    */

    /* Set up the maze run. Needs everything above: I2C, the motors, the head, the timer and the
     * gyro. Sense_Init() inside takes over the ultrasonic sensor. */
    const bool navigatorReady = Navigator_Init();
    if (!navigatorReady) {
        DBG_PRINTF("Navigator init failed, the car will not drive\n");
    }

    /* The run starts on its own after a pause, so there is time to put the car down and move the USB
     * cable out of the way. The car stands still during the pause.
     *
     * To start the run from the phone instead, comment out these three lines: CM4_COMMAND_RUN_START
     * does the same thing, and CM4_COMMAND_RUN_STOP halts the car from a distance. */
    if (navigatorReady && motionReady) {
        DBG_PRINTF("Run starts in %lu ms\n", (unsigned long) CONFIG_RUN_START_DELAY_MS);
        Cy_SysLib_Delay(CONFIG_RUN_START_DELAY_MS);
        Navigator_Start();
    }

    // Main loop. Your code goes here.
    for (;;) {
        // Check for new messages from CM0 core and process them. This is the most important task.
        if (CM4_isDataAvailableFromCM0()) {
            processIncomingIPCMessage(CM4_GetCM0Message());
        }

        /* The maze run, in this order on every single pass.
         *
         * Motion_Update() first: it integrates the gyro heading, and both the turns and the straight
         * driving steer by it, so a pass that skips it steers on stale data. It does nothing more
         * often than every 10 ms on its own.
         *
         * Navigator_Update() last, so that it sees the results sense and drive finished in this same
         * pass instead of one pass later.
         *
         * Nothing here blocks, which is the whole reason the modules are written as state machines. */
        (void) Motion_Update();
        Sense_Update();
        Drive_Update();
        Navigator_Update();

        /* Demo code from the template. It prints on the debug UART and paints the LEDs by distance.
         * Left here on purpose as a reference for the APIs, but switched off: it reads the ultrasonic
         * sensor that the sense module now owns, and printing every 500 ms competes with the state
         * machines for the loop. */
        /*
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
        */
    }
}

static void processIncomingIPCMessage(ipc_msg_t* msg) {
    // In general, impossible situation, but never trust anyone.
    if (msg != NULL) {
        // Process messages with at least 1 byte of payload
        if (msg->len > 0) {
            // Decode message code. You would mostly like to use Commands
            switch (msg->userCode) {
                case IPC_USR_CODE_CMD: {
                    // Only [0] element was used for simplicity.
                    // IDEA: You could expand it to accept strings or more complex data packets!
                    // You can define array of strings and perform memcmp. This operation is a bit more costful.
                    processCM4Command((enum cm4CommandList) msg->buffer[0]);
                    break;
                }
                case IPC_USR_CODE_REQ: {
                    // Not implemented. You can be very creative here!
                    break;
                }
                default: break;
            }
        }
    }
}

static void processCM4Command(enum cm4CommandList cmd) {
    switch (cmd) {
        case CM4_COMMAND_LED_ENA: {
            Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, 0);
            Cy_GPIO_Write(CYBSP_USER_LED2_PORT, CYBSP_USER_LED2_NUM, 0);
            break;
        }
        case CM4_COMMAND_LED_DIS: {
            Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, 1);
            Cy_GPIO_Write(CYBSP_USER_LED2_PORT, CYBSP_USER_LED2_NUM, 1);
            break;
        }
        case CM4_COMMAND_CAR_SAY: {
            if (CM4_IsCM0Ready()) {
                ipcMsgForCM0.userCode = IPC_USR_CODE_CMD;
                ipcMsgForCM0.len = 0x01;
                ipcMsgForCM0.buffer[0] = (uint8_t) CM0_SHARED_CAR_SAY;
                CM4_SendCM0Message(&ipcMsgForCM0);
            }
            break;
        }
        case CM4_COMMAND_RUN_START: {
            Navigator_Start();
            break;
        }
        case CM4_COMMAND_RUN_STOP: {
            /* The remote kill switch: stops the motors wherever the car is. */
            Navigator_Stop();
            break;
        }
        case CM4_COMMAND_ECHO: {
            if (CM4_IsCM0Ready()) {
                ipcMsgForCM0.userCode = IPC_USR_CODE_CMD;
                ipcMsgForCM0.len = CM4_GetCM0Message()->len;
                ipcMsgForCM0.buffer[0] = (uint8_t) CM0_SHARED_BLE_NTF_RELAY;
                memcpy(&ipcMsgForCM0.buffer[1], &CM4_GetCM0Message()->buffer[1], ipcMsgForCM0.len - 1);
                CM4_SendCM0Message(&ipcMsgForCM0);
            }
            break;
        }
        default: break;
    }
}


/* [] END OF FILE */
