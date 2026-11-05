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
#include "distance.h"
#include "ledctrl.h"
#include "i2c_common.h"
#include "debug_uart.h"
#include "cm4_ipc.h"
#include "cm4_cmd.h"
#include "explore.h"
#include "telemetry.h"

int main(void) {
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS) {
        CY_ASSERT(0);
    }

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

    Telemetry_Init(motionReady, distanceReady);
    Car_SetIdleHook(Telemetry_Poll);

    bool mazeReady = distanceReady && motionReady;
    if (!mazeReady) {
        DBG_PRINTF("Maze: distance or motion is not ready, exploration is off\n");
    }

    for (;;) {
        if (CM4_isDataAvailableFromCM0()) {
            CM4_ProcessIncomingIPCMessage(CM4_GetCM0Message());
        }

#if defined(BLE_LOG_ENABLED)
        // Hand buffered log text to CM0+ for BLE (non-blocking, at most 254 bytes per call)
        CM4_ForwardLogsToCM0();
#endif

        // One step is blocking (scan, turn, drive one cell), so the IPC messages above wait for it
        if (mazeReady) {
            (void) Explore_Step();
        }

        Telemetry_Poll();
    }
}
