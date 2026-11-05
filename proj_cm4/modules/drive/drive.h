#ifndef DRIVE_H
#define DRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Moves the car: one step forward, one turn in place, or stop. Nothing here knows about mazes,
 * walls or openings. It is told "go forward up to 180 mm" or "turn by -90 degrees" and does that.
 *
 * Nothing blocks. Start a movement, call Drive_Update() from the main loop until Drive_IsDone(),
 * then ask why it ended. car.c already has Car_TurnBy(), which turns correctly but blocks in a while
 * loop; the same gyro logic lives here as a state machine, so the main loop keeps running.
 *
 * REQUIREMENTS of the main loop:
 *  - Motion_Update() on every pass. The heading comes from the gyro and is integrated there, so a
 *    loop that skips it steers blind.
 *  - Drive_Update() on every pass while a movement is running.
 *
 * WHO AIMS THE HEAD: the caller. This module reads the distance in whatever direction the head
 * currently points, through Sense_GetDistanceMm(). A step watches the right wall, so the head must
 * point right; an approach watches the front wall, so the head must point forward. Aiming the head
 * is the navigator's job, because only the navigator knows what the car is about to do. */

/* Why a movement ended. The navigator reacts differently to each one, which is the whole point of
 * reporting it instead of just saying "done". */
typedef enum {
    DRIVE_END_NONE = 0,        /* still running, or nothing has run yet */
    DRIVE_END_LENGTH,          /* the planned length was used up: the normal end of a step */
    DRIVE_END_TARGET_REACHED,  /* an approach reached its front distance, or a turn its heading */
    DRIVE_END_WALL_LOST,       /* the watched wall suddenly went far away: an opening */
    DRIVE_END_WALL_TOO_CLOSE,  /* the watched wall came dangerously close */
    DRIVE_END_TIMEOUT,         /* a turn did not reach its heading in time: something is stuck */
    DRIVE_END_STOPPED,         /* Drive_Stop() was called */
    DRIVE_END_COUNT
} Drive_EndReason;

///////////////////// INIT ////////////////////////////////////////////////////

/* Stops the motors and clears the state. Needs Motor_Init(), Motion_Init(), Timing_Init() and
 * Sense_Init() before. */
void Drive_Init(void);

///////////////////// MOVEMENTS ///////////////////////////////////////////////

/* Step forward while following the right wall. The head must point right.
 *
 * length_mm is an upper limit, not a promise. There are no encoders, so the distance travelled is
 * only known as speed multiplied by time, and the speed changes with the battery and the floor. The
 * step therefore really ends on whichever comes first:
 *   - the right wall went far away      -> DRIVE_END_WALL_LOST, an opening to take
 *   - the right wall came too close     -> DRIVE_END_WALL_TOO_CLOSE, straighten out first
 *   - the time for length_mm ran out    -> DRIVE_END_LENGTH, scan again
 *
 * The heading is held with the gyro, so the car drives straight even when the right wall is out of
 * sight. A reading without a valid echo never ends the step: silence is not an opening. */
void Drive_StartStep(uint16_t length_mm);

/* Step forward watching nothing at all, holding the heading, until the length is used up.
 *
 * Needed for the short step the car must take after a turn, before it looks right again. Right after
 * a turn into an opening the right side is open by definition, so a normal step would end instantly
 * with DRIVE_END_WALL_LOST and the car would turn in place forever. This movement has no such
 * opinion about what it sees.
 *
 * The head is free during this movement, which is a good moment to aim it for what comes next.
 * Always ends with DRIVE_END_LENGTH. */
void Drive_StartMove(uint16_t length_mm);

/* Step forward until the wall in front is stop_front_mm away. The head must point forward.
 *
 * This is how the car drives up to the middle of an opening before turning: the target comes from
 * the corridor width, and the measurement tells the car where it actually is, which nothing else
 * can do without encoders.
 *
 * max_length_mm caps how far this may go, in case the front wall is never seen.
 * Ends with DRIVE_END_TARGET_REACHED, or DRIVE_END_LENGTH when the cap is reached first. */
void Drive_StartApproach(uint16_t stop_front_mm, uint16_t max_length_mm);

/* Turn in place by delta_deg, positive to the left, matching Motion_GetYaw() and
 * Decide_ActionTurnDegrees().
 *
 * For a half turn, ask for two turns of 90 degrees instead of one of 180: around 180 the shortest
 * way in Motion_GetYawError() can flip sign and the car would hunt back and forth.
 *
 * Ends with DRIVE_END_TARGET_REACHED, or DRIVE_END_TIMEOUT when the heading is not reached in time. */
void Drive_StartTurn(int16_t delta_deg);

/* Stop the motors at once and end the current movement with DRIVE_END_STOPPED. */
void Drive_Stop(void);

///////////////////// RUNNING /////////////////////////////////////////////////

/* Advance the current movement. Call from the main loop as often as possible. Cheap and safe to call
 * when nothing is running. */
void Drive_Update(void);

/* True when no movement is running: either it finished or none was started. */
bool Drive_IsDone(void);

/* Why the last movement ended. DRIVE_END_NONE while one is still running. */
Drive_EndReason Drive_GetEndReason(void);

/* Rough estimate of how far the last step travelled, in millimeters, from speed and time.
 * Good enough to tell "we have barely moved" from "we crossed the corridor", which the stuck
 * detection needs. Never use it as a real distance. */
uint16_t Drive_GetLastStepMm(void);

///////////////////// SPEED CALIBRATION ///////////////////////////////////////

/* Without encoders, every length this module is given is carried out as a time, and the speed is
 * what converts one into the other. A wrong speed therefore bends every distance in the run.
 *
 * So the car measures it. During an approach the head looks forward, which means the front distance
 * is known both at the start and at the end, and the difference over the elapsed time is a speed. It
 * is the one movement that has a ruler along it.
 *
 * Measurements that are too short in distance or time are thrown away, the result is kept inside a
 * plausible range, and each new value is averaged with the old one, so a single odd reading cannot
 * knock the car off. The distance the median filter lags behind while the car moves is added back,
 * otherwise the speed would come out low and every later movement would overshoot. */

/* The speed currently used for the conversion, in millimeters per second. */
uint16_t Drive_GetSpeedMmps(void);

/* False while this is still the starting guess from config.h, true once the car has measured it
 * at least once. Worth logging: a run whose speed was never measured is running on a guess. */
bool Drive_IsSpeedMeasured(void);

///////////////////// DEBUG ///////////////////////////////////////////////////

/* Short lowercase name of an end reason for logs, for example "wall lost". Never NULL. */
const char* Drive_EndReasonName(Drive_EndReason reason);

#ifdef __cplusplus
}
#endif

#endif /* DRIVE_H */
