#ifndef DECIDE_H
#define DECIDE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* The right hand rule: given what the car sees to the left, to the front and to the right, say where
 * to go next. Priority: right, then front, then left, then turn around.
 *
 * This module is deliberately free of hardware: no sensors, no motors, no timers, no state. It is a
 * pure function of its input, which is what makes it testable on a PC (see tests/decide) and what
 * makes it safe to change the sensing or the driving without touching the navigation rule.
 *
 * It also holds no map and no history. Detecting that the car is going in circles is the job of the
 * safety module, which can use Decide_ActionTurnDegrees() to add up the turns. */

/* What the car knows about one direction.
 *
 * DECIDE_UNKNOWN means the sensor gave no valid echo. That is NOT the same as free space: an
 * ultrasonic pulse that hits a wall at a steep angle often never comes back, so a wall and an open
 * corridor can both answer with silence. Treating silence as free space is how the car drives into
 * a wall, so every rule below requires an explicit DECIDE_OPEN before it moves that way.
 *
 * Resolving an unknown direction (measure again, look slightly off axis, take a short step) belongs
 * to the sensing module. By the time the state reaches this module the decision is final. */
typedef enum {
    DECIDE_UNKNOWN = 0, /* no valid echo, must never be treated as free */
    DECIDE_WALL,        /* an obstacle close enough to block the way */
    DECIDE_OPEN         /* confirmed free space */
} Decide_SideState;

/* What the car should do next. The navigator turns this into drive commands. */
typedef enum {
    DECIDE_ACTION_FORWARD,     /* keep following the right wall */
    DECIDE_ACTION_TURN_RIGHT,  /* an opening on the right: take it */
    DECIDE_ACTION_TURN_LEFT,   /* right and front blocked: the corridor bends left */
    DECIDE_ACTION_TURN_AROUND, /* dead end: everything is blocked */
    DECIDE_ACTION_COUNT        /* number of actions, not an action itself */
} Decide_Action;

/* What the car sees at one standstill scan. Filled by the sensing module. */
typedef struct {
    Decide_SideState left;
    Decide_SideState front;
    Decide_SideState right;
} Decide_Scan;

/* The rule itself. Never fails and never returns "do not know": for any input, including three
 * unknowns, there is always one defined action.
 *
 * All eight combinations of wall and open from the task description:
 *
 *   right   front   left    situation              action
 *   ------- ------- ------- ---------------------- -----------
 *   open    open    open    crossing               turn right
 *   open    open    wall    branch to the right    turn right
 *   open    wall    open    T crossing             turn right
 *   open    wall    wall    corner to the right    turn right
 *   wall    open    open    branch to the left     forward
 *   wall    open    wall    corridor               forward
 *   wall    wall    open    corner to the left     turn left
 *   wall    wall    wall    dead end               turn around
 *
 * scan must not be NULL. */
Decide_Action Decide_NextAction(const Decide_Scan* scan);

/* How much the heading changes when the action is carried out, in whole degrees, positive to the
 * left to match Motion_GetYaw() in motion.h.
 *
 * Two users: the driving module, which needs the target heading for a turn in place, and the safety
 * module, which adds these up to notice that the car is circling. A turn around counts as +180 and
 * not -180 so that the sum stays meaningful; which way the car physically spins is decided by the
 * driving module from the free space it has. */
int16_t Decide_ActionTurnDegrees(Decide_Action action);

/* Short lowercase name of the action for the debug log and the BLE telemetry, for example "right".
 * Always returns a valid string, never NULL, so it is safe to pass straight into a print. */
const char* Decide_ActionName(Decide_Action action);

#ifdef __cplusplus
}
#endif

#endif /* DECIDE_H */
