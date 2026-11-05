#include <stdint.h>
#include "telemetry.h"
#include "car.h"
#include "motion.h"
#include "distance.h"
#include "debug_uart.h"
#include "cm4_cmd.h"

#if PRINT_HEADING_DISTANCE
static bool motionOk = false;
static bool distanceOk = false;
#endif

void Telemetry_Init(bool motionReady, bool distanceReady) {
#if PRINT_HEADING_DISTANCE
    motionOk = motionReady;
    distanceOk = distanceReady;
#else
    (void) motionReady;
    (void) distanceReady;
#endif
}

// Runs inside the motor control loops too, so it must stay short: no Motion_Update() here
// (it would steal gyro samples from the turn loop), no waiting.
void Telemetry_Poll(void) {
#if PRINT_HEADING_DISTANCE
    static uint32_t lastPrintMs = 0;
    uint32_t nowMs = Timing_GetMilliseconds();
    if (nowMs - lastPrintMs < TELEMETRY_PERIOD_MS) {
        return;
    }
    lastPrintMs = nowMs;

    uint16_t mm = 0;
    bool hasDistance = distanceOk && Distance_GetMm(&mm);
    // -1 means that there is no value (not ready, no echo, or out of range)
    DBG_PRINTF("T hdg=%d dist=%d\n", motionOk ? (int) Motion_GetYaw() : -1, hasDistance ? (int) mm : -1);
#if defined(BLE_LOG_ENABLED)
    CM4_ForwardLogsToCM0();  // one non-blocking push; the blocking loops do not forward logs by themselves
#endif
#endif
}
