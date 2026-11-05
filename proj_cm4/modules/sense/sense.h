#ifndef SENSE_H
#define SENSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "decide.h"
#include <stdbool.h>
#include <stdint.h>

/* Turns the hardware into the three states the right hand rule needs: for the right, the front and
 * the left of the car, is there a wall, is it open, or does the sensor not know.
 *
 * There is one ultrasonic sensor on a servo, so the three directions are measured one after another,
 * and the head has to travel between them. One direction costs about 300 ms: the servo travel plus
 * enough measurements to flush the median filter.
 *
 * Nothing here blocks. A scan is a state machine: start it, call Sense_Update() from the main loop
 * until Sense_IsDone(), then read the result. The main loop stays free to call Motion_Update(), which
 * is what integrates the gyro heading, and to serve BLE. A blocking wait of 300 ms would lose both.
 *
 * This module does not decide anything and does not drive. It only looks. */

/* Scanning order, which is also the priority order of the right hand rule. */
typedef enum {
    SENSE_DIR_RIGHT = 0,
    SENSE_DIR_FRONT,
    SENSE_DIR_LEFT,
    SENSE_DIR_COUNT
} Sense_Direction;

/* The outcome of one complete scan. */
typedef struct {
    /* What the rule needs. A direction that was skipped or gave no echo is DECIDE_UNKNOWN. */
    Decide_SideState state[SENSE_DIR_COUNT];

    /* Measured distance, only meaningful where state is DECIDE_WALL or DECIDE_OPEN. The navigator
     * needs the front distance in millimeters to work out how long the next step may be. */
    uint16_t distance_mm[SENSE_DIR_COUNT];

    /* False for a direction the lazy scan never looked at. Both a skipped direction and a direction
     * without an echo end up as DECIDE_UNKNOWN, and only this flag tells the two apart, which matters
     * when reading a log. */
    bool measured[SENSE_DIR_COUNT];
} Sense_Result;

///////////////////// INIT ////////////////////////////////////////////////////

/* Takes over the ultrasonic sensor: initializes it in automatic mode and starts it, then aims the
 * head at the right wall. Needs cybsp_init(), I2C_Main_Start(), Motor_Init(), Head_Init() and
 * Timing_Init() before. Returns false when the sensor cannot be set up, which is a wiring or
 * configurator problem, not a track situation. */
bool Sense_Init(void);

///////////////////// SCANNING ////////////////////////////////////////////////

/* Begin a new scan. Clears the previous result and aims the head at the first direction. Calling it
 * while a scan is running restarts the scan from the beginning. */
void Sense_StartScan(void);

/* Measure one direction only, and stop there.
 *
 * Needed because a full scan is lazy: when the right side turns out to be open, the front is never
 * measured, and the car still has to know the distance to the front wall to drive up to the middle
 * of the opening before it turns. That distance has to be fresh anyway, measured where the car
 * actually stands, not where it stood during the scan.
 *
 * The result keeps the states of the other directions from the scan before, so read the direction
 * that was asked for. An invalid direction does nothing. */
void Sense_StartScanDirection(Sense_Direction direction);

/* Advance the scan by one step. Call it from the main loop as often as possible. Cheap and safe to
 * call when no scan is running. */
void Sense_Update(void);

/* True once the scan has finished and the result can be read. */
bool Sense_IsDone(void);

/* The result of the last finished scan. Never NULL. Reading it before the first scan has finished
 * gives all directions unknown and nothing measured. */
const Sense_Result* Sense_GetResult(void);

/* The same result in the form the rule takes, ready for Decide_NextAction(). */
Decide_Scan Sense_GetScan(void);

///////////////////// HEAD AIMING /////////////////////////////////////////////

/* These are also used outside a scan: during a step the head keeps looking at the right wall, so
 * that the car can follow it and notice where it ends.
 *
 * The servos never report their position, so "the head has arrived" is purely a matter of time. This
 * is the only place in the project that knows how long that is. */

/* Aim the head, in whole degrees, positive to the left (see head.h). Starts the travel timer.
 * Asking for the angle the head is already commanded to does nothing and, in particular, does not
 * restart the timer, so a head that had already arrived stays arrived. */
void Sense_PointHead(int8_t pan_degrees);

/* True when enough time has passed for the servo to have reached the commanded angle. */
bool Sense_IsHeadSettled(void);

/* Filtered distance in the direction the head currently points, for following the right wall during
 * a step. Returns false while the head is still travelling, because a measurement taken then belongs
 * to whatever the head passed on the way, and also when there is no valid echo. */
bool Sense_GetDistanceMm(uint16_t* distance_mm);

///////////////////// DEBUG ///////////////////////////////////////////////////

/* Short lowercase name of a direction for logs, for example "right". Never NULL. */
const char* Sense_DirectionName(Sense_Direction direction);

/* Short lowercase name of a state for logs: "unknown", "wall" or "open". Never NULL. */
const char* Sense_StateName(Decide_SideState state);

#ifdef __cplusplus
}
#endif

#endif /* SENSE_H */
