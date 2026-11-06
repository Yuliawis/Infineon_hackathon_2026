#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* The state machine that runs the maze: it gives the orders to sense, decide and drive in the right
 * order and waits for each of them to finish.
 *
 * It does no sensing and no driving of its own. It never calls Motor_Move() or Head_*() directly,
 * which is what lets the way a turn or a scan is done change without touching the course of the run.
 *
 * MAIN LOOP, every pass and in this order:
 *     Motion_Update();      the gyro heading is integrated here
 *     Sense_Update();
 *     Drive_Update();
 *     Navigator_Update();
 *
 * Navigator_Update() last, so that it sees the finished results of the other two in the same pass
 * instead of one pass later. */

typedef enum {
    NAV_STATE_IDLE = 0,      /* no run: before Navigator_Start() or after it ended */
    NAV_STATE_CALIBRATE,     /* standing still, measuring the gyro bias */
    NAV_STATE_SCAN,          /* looking around from a standstill */
    NAV_STATE_DECIDE,        /* applying the right hand rule, takes no time */
    NAV_STATE_STEP,          /* driving forward along the right wall */
    NAV_STATE_CONFIRM,       /* the right wall seems to have ended: is it real */
    NAV_STATE_MEASURE_FRONT, /* measuring the front wall to drive up to the opening */
    NAV_STATE_APPROACH,      /* driving up to the middle of the opening */
    NAV_STATE_TURN,          /* turning in place */
    NAV_STATE_SETTLE,        /* the short step after a turn */
    NAV_STATE_ESCAPE,        /* the right wall got too close: short move aimed away from it */
    NAV_STATE_FINISHED,      /* the run is over */
    NAV_STATE_ERROR,         /* the hardware did not start up */
    NAV_STATE_COUNT
} Nav_State;

///////////////////// INIT ////////////////////////////////////////////////////

/* Sets up sense and drive. Needs cybsp_init(), I2C_Main_Start(), Motor_Init(), Head_Init(),
 * Timing_Init() and Motion_Init() before. Returns false when the ultrasonic sensor cannot be set up,
 * and the navigator then stays in NAV_STATE_ERROR and never moves the car. */
bool Navigator_Init(void);

///////////////////// RUNNING /////////////////////////////////////////////////

/* Start a run. The car must stand still and the right wall must be beside it, because the first
 * thing that happens is the gyro calibration. */
void Navigator_Start(void);

/* Same, but the car does not move before Timing_GetMilliseconds() reaches go_at_ms. Calibration and
 * the first scan still happen right away, at a standstill, so the first movement comes exactly at
 * go_at_ms instead of that plus however long they took. A time already past means no wait. */
void Navigator_StartAt(uint32_t go_at_ms);

/* Advance the run. Call from the main loop, last of the update calls. */
void Navigator_Update(void);

/* Stop at once: the motors go off and the run ends. */
void Navigator_Stop(void);

Nav_State Navigator_GetState(void);

/* True while a run is going on, which is anything but idle, finished and error. */
bool Navigator_IsRunning(void);

/* How many turns the car has made, left counted as +90, right as -90 and a half turn as +180,
 * added up over the run. Around +360 is normal when following an outer wall; a large negative sum
 * hints at circling an island. This is a hint for the loop detection, not proof of anything. */
int32_t Navigator_GetTurnSumDeg(void);

///////////////////// DEBUG ///////////////////////////////////////////////////

/* Short lowercase name of a state for logs, for example "approach". Never NULL. */
const char* Navigator_StateName(Nav_State state);

#ifdef __cplusplus
}
#endif

#endif /* NAVIGATOR_H */
