#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>

// Periodic telemetry lines "T key=value ..." in the debug log (UART and BLE).
// tools/ble_serial.py --csv saves them to a CSV file.

// 1 = print "T hdg=<deg> dist=<mm>" every TELEMETRY_PERIOD_MS (-1 = no value)
#define PRINT_HEADING_DISTANCE (1)
#define TELEMETRY_PERIOD_MS    (250u)

// Tell which sensors work; values of the others are printed as -1.
void Telemetry_Init(bool motionReady, bool distanceReady);

// Print when the period is over and forward the log to CM0+ (BLE). Short and non-blocking:
// call it from the main loop and pass it to Car_SetIdleHook(), so it also runs while the car moves.
void Telemetry_Poll(void);

#endif /* TELEMETRY_H */
