#ifndef DISTANCE_H
#define DISTANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "HCSR04.h"

// Distance to an obstacle in millimeters from the ultrasonic sensor on the head (HCSR04 driver),
// filtered with a median filter. The sensor measures on its own after Distance_Start(), every
// DISTANCE_PERIOD_MS (set by HCSR04_PERIOD_US in HCSR04.h).
// A measurement without an echo or out of range counts as "very far" in the median, so the result is
// "no value" only when more than half of the last DISTANCE_FILTER_SIZE measurements have no echo.

// Outside of this range the echo is not trusted
#define DISTANCE_MIN_MM (20u)
#define DISTANCE_MAX_MM (4000u)

// About 343 m/s at 20 C
#define DISTANCE_SOUND_SPEED_MPS (343u)

// Measurements in the median filter. Odd, from 1 (no filtering) to 15. Ignores up to half of them
// (5: two bad ones), but while moving the value is about half of them old (5: ~2 periods).
#define DISTANCE_FILTER_SIZE (5u)

// Time between two measurements
#define DISTANCE_PERIOD_MS (HCSR04_PERIOD_US / 1000u)

// Wait after Distance_Start() until the filter is full of new measurements: one period per measurement,
// plus a margin for the echo of the last one. Use it instead of own numbers, so it follows the two above.
#define DISTANCE_FILL_MS (DISTANCE_FILTER_SIZE * DISTANCE_PERIOD_MS + 20u)

// How the filter gets the measurements, chosen in Distance_Init()
typedef enum {
    DISTANCE_MODE_AUTO,   // in the background, in the interrupts of the sensor, nothing to call
    DISTANCE_MODE_MANUAL  // only when Distance_Update() is called, often enough (faster than every DISTANCE_PERIOD_MS)
} Distance_Mode;

// Returns false when the mode is wrong or the sensor setup in the Device Configurator is wrong (see HCSR04.h).
bool Distance_Init(Distance_Mode mode);
// Start also empties the filter
void Distance_Start(void);
void Distance_Stop(void);

// Manual mode: takes the new measurement of the sensor, if there is one. Returns true if it did.
// Automatic mode: does nothing and returns false.
bool Distance_Update(void);
// Return false when there is no valid value (no echo, or out of range)
bool Distance_GetMm(uint16_t* distance_mm);     // median of the last measurements
bool Distance_GetRawMm(uint16_t* distance_mm);  // the last single measurement

// Measurements processed since Distance_Start()
uint32_t Distance_GetCount(void);

// Echo pulse (the sound goes there and back) to millimeters, no range check
uint32_t Distance_EchoMicrosecondsToMm(uint32_t echo_us);

#ifdef __cplusplus
}
#endif

#endif /* DISTANCE_H */
