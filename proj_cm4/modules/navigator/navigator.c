#include "navigator.h"
#include "config.h"
#include "car.h"
#include "debug_uart.h"
#include "decide.h"
#include "drive.h"
#include "motion.h"
#include "sense.h"
#include "utils.h"
#include <stddef.h>

/* Distances worked out from the tunables in config.h. They live here and not in config.h because
 * they are not knobs: they follow from the corridor width and the sensor mounting, and changing one
 * of them on its own would only make the geometry inconsistent. */

/* Where to stop in front of a wall so that the turning axis ends up in the middle of the corridor.
 * The sensor sits ahead of the axis, so the sensor has to stop that much closer. */
#define APPROACH_FRONT_TARGET_MM ((CONFIG_CORRIDOR_WIDTH_MM / 2) - CONFIG_SENSOR_TO_AXIS_MM)

/* How far the axis has to go from the point where the side beam passes the edge of a wall until it
 * is in the middle of the opening: half a corridor, plus the sensor being ahead of the axis. What the
 * car has already covered since then is taken off at run time, see past_edge_mm. */
#define OPENING_CENTER_FROM_EDGE_MM ((CONFIG_CORRIDOR_WIDTH_MM / 2) + CONFIG_SENSOR_TO_AXIS_MM)

/* How far to drive after a right turn before looking right again, see CONFIG_ENTER_MARGIN_MM. */
#define ENTER_TRAVEL_MM ((CONFIG_CORRIDOR_WIDTH_MM / 2) - CONFIG_SENSOR_TO_AXIS_MM + CONFIG_ENTER_MARGIN_MM)

_Static_assert(APPROACH_FRONT_TARGET_MM > 0, "the sensor cannot sit further ahead than half a corridor");
_Static_assert(ENTER_TRAVEL_MM > CONFIG_STEP_AFTER_TURN_MM, "entering after a right turn must go further");

static Nav_State state = NAV_STATE_IDLE;

/* False until the one-shot command of the current state has been issued. Every state is written as
 * "give the order once, then wait", and this is what keeps the order from being given again on every
 * pass of the main loop. */
static bool order_given = false;

static uint32_t run_start_ms = 0;
static uint32_t state_start_ms = 0;

/* The car does not move before this time, see Navigator_StartAt(). */
static uint32_t go_at_ms = 0;
static bool go_wait_logged = false;

/* The action the rule chose, kept while the car drives up to the place where it can carry it out. */
static Decide_Action pending_action = DECIDE_ACTION_FORWARD;

/* Degrees still to turn for the pending action, taken apart into pieces of at most 90 degrees. */
static int16_t turn_remaining_deg = 0;
static bool turn_piece_running = false;
static uint32_t turn_piece_end_ms = 0;

static int32_t turn_sum_deg = 0;

/* The heading the car is supposed to have: a multiple of 90 degrees from the heading it started the
 * run with. Every movement is aimed at this, never at wherever the car points right now.
 *
 * This is what keeps the errors of a slippery floor from adding up. A turn that ends 5 degrees short,
 * or a stop that twists the car a little, is corrected by the next movement, because that movement
 * still aims at the grid heading. Before, each movement adopted the current heading as "straight",
 * so every one of those small errors was kept for the rest of the run. */
static int16_t nav_heading_deg = 0;

/* Estimate of how far the car, at rest, already is past the end of the right wall, so a turn into an
 * opening is taken from the middle of the opening and not from wherever the opening was noticed.
 *
 * It used to be assumed that the car always stands right at the edge. That is only true when a step
 * catches the edge on the move. An opening found by a standstill scan can be up to a whole step
 * further on, plus the slide after the stop on this floor, and driving the full distance from there
 * put the car at the far side of the opening, turning into the corner. */
static uint16_t past_edge_mm = 0;

/* True only while past_edge_mm comes from a measurement: a step that ended because the right wall
 * went away, so odometry recorded the crossing of the edge itself.
 *
 * In the other two places the number is filled in from constants, which say nothing about where the
 * edge of this opening is. Driving the 130 mm constant added 180 mm of travel to a car that already
 * stood near the middle of its cell, which is more than the 150 mm of room it has before its side
 * reaches the far wall of the new corridor. That is the wall it kept scraping. */
static bool past_edge_valid = false;

/* Lining up with the grid heading before a scan, see alignToGrid(). Reset on every state change, so
 * each state gets exactly one attempt and a car that cannot line up still carries on. */
static bool align_running = false;
static bool align_done = false;

/* DIAGNOSTIC, temporary. Right after a turn to the right and before driving into the new corridor,
 * the head looks left and right and the two distances go into the log. They say where the car really
 * stands in the corridor it just turned into, which no amount of arithmetic about beam width, filter
 * lag and sensor offset can settle. Costs about 0.6 s per right turn and changes nothing about where
 * the car drives. See updateSettle(). */
typedef enum {
    PROBE_LEFT_START = 0,
    PROBE_LEFT_WAIT,
    PROBE_RIGHT_START,
    PROBE_RIGHT_WAIT,
    PROBE_DONE
} Settle_Probe;

static Settle_Probe settle_probe = PROBE_LEFT_START;

static const char* const state_names[NAV_STATE_COUNT] = {
        [NAV_STATE_IDLE] = "idle",
        [NAV_STATE_CALIBRATE] = "calibrate",
        [NAV_STATE_SCAN] = "scan",
        [NAV_STATE_DECIDE] = "decide",
        [NAV_STATE_STEP] = "step",
        [NAV_STATE_CONFIRM] = "confirm",
        [NAV_STATE_MEASURE_FRONT] = "measure front",
        [NAV_STATE_APPROACH] = "approach",
        [NAV_STATE_TURN] = "turn",
        [NAV_STATE_SETTLE] = "settle",
        [NAV_STATE_ESCAPE] = "escape",
        [NAV_STATE_FINISHED] = "finished",
        [NAV_STATE_ERROR] = "error",
};

_Static_assert(sizeof(state_names) / sizeof(state_names[0]) == NAV_STATE_COUNT,
               "state_names must have one entry per state");

///////////////////// STATE HANDLING //////////////////////////////////////////

/* Into the (-180, 180] range of Motion_GetYaw(). */
static int16_t wrapHeadingDeg(int32_t heading_deg) {
    while (heading_deg > 180) {
        heading_deg -= 360;
    }
    while (heading_deg <= -180) {
        heading_deg += 360;
    }
    return (int16_t) heading_deg;
}

static void enterState(Nav_State next) {
    state = next;
    order_given = false;
    align_running = false;
    align_done = false;
    settle_probe = PROBE_LEFT_START;
    state_start_ms = Timing_GetMilliseconds();
    DBG_PRINTF("nav: %s\n", Navigator_StateName(next));
}

/* Turn in place back to the grid heading if the car is off by more than CONFIG_ALIGN_TOLERANCE_DEG.
 * Returns true once the car is lined up, or once the one attempt of this state is over.
 *
 * The side readings are only good when the beam meets the wall at a right angle. A car that has
 * skidded a few degrees reads the right wall as much further away, or not at all, and would take
 * that for an opening and turn into the wall. So every scan is taken square to the corridor. */
static bool alignToGrid(void) {
    if (align_running) {
        if (!Drive_IsDone()) {
            return false;
        }
        align_running = false;
        align_done = true;
        return true;
    }

    if (align_done) {
        return true;
    }

    const int16_t error_deg = Motion_GetYawError(nav_heading_deg);
    if ((error_deg <= CONFIG_ALIGN_TOLERANCE_DEG) && (error_deg >= -CONFIG_ALIGN_TOLERANCE_DEG)) {
        align_done = true;
        return true;
    }

    DBG_PRINTF("nav: aligning, %d deg off\n", (int) error_deg);
    Drive_StartTurnTo(nav_heading_deg);
    align_running = true;
    return false;
}

///////////////////// INIT ////////////////////////////////////////////////////

bool Navigator_Init(void) {
    turn_sum_deg = 0;

    if (!Sense_Init()) {
        DBG_PRINTF("nav: ultrasonic sensor did not start\n");
        state = NAV_STATE_ERROR;
        return false;
    }

    Drive_Init();
    state = NAV_STATE_IDLE;
    return true;
}

void Navigator_Start(void) {
    if (state == NAV_STATE_ERROR) {
        return; /* the hardware never came up, so there is nothing to start */
    }

    run_start_ms = Timing_GetMilliseconds();
    turn_sum_deg = 0;
    nav_heading_deg = 0;
    past_edge_mm = 0;
    past_edge_valid = false;
    turn_remaining_deg = 0;
    turn_piece_running = false;
    pending_action = DECIDE_ACTION_FORWARD;
    go_at_ms = Timing_GetMilliseconds(); /* no wait unless Navigator_StartAt() sets one */
    go_wait_logged = false;
    enterState(NAV_STATE_CALIBRATE);
}

void Navigator_StartAt(uint32_t go_at) {
    Navigator_Start();
    if (Navigator_IsRunning()) {
        go_at_ms = go_at;
    }
}

/* True while the first movement still has to wait for go_at_ms. Wrap-safe: a time in the past,
 * even 49 days of uptime later, reads as "already reached". */
static bool waitingToGo(void) {
    return (int32_t) (go_at_ms - Timing_GetMilliseconds()) > 0;
}

void Navigator_Stop(void) {
    Drive_Stop();
    enterState(NAV_STATE_FINISHED);
}

///////////////////// STATES //////////////////////////////////////////////////

/* Standing still, measuring the gyro bias.
 *
 * This is the one place in the project that is allowed to block: Motion_CalibrateGyro() waits inside
 * Cy_SysLib_Delay() for about a second. That is fine here and only here, because the car has to stand
 * perfectly still for the measurement anyway, so there is nothing for the main loop to do meanwhile. */
static void updateCalibrate(void) {
    if (!Motion_CalibrateGyro()) {
        DBG_PRINTF("nav: gyro calibration failed\n");
        state = NAV_STATE_ERROR;
        return;
    }

    /* From here on, zero degrees means "the heading the car started the run with". */
    Motion_ResetYaw();
    nav_heading_deg = 0;
    enterState(NAV_STATE_SCAN);
}

static void updateScan(void) {
    if (!alignToGrid()) {
        return;
    }

    if (!order_given) {
        Sense_StartScan();
        order_given = true;
        return;
    }

    if (Sense_IsDone()) {
        enterState(NAV_STATE_DECIDE);
    }
}

/* How far the next step may go.
 *
 * In practice the limit is always CONFIG_STEP_MAX_MM: the rule only chooses to go forward when the
 * front is open, and open means further away than CONFIG_FRONT_OPEN_MM, which is well beyond one
 * step plus the margin. The clamp is insurance for the day somebody lowers that threshold. */
static uint16_t stepLengthMm(void) {
    const Sense_Result* const result = Sense_GetResult();
    int length_mm = CONFIG_STEP_MAX_MM;

    if (result->state[SENSE_DIR_FRONT] == DECIDE_WALL) {
        /* The slide after the stop is taken off as well: on this floor the car does not stop where
         * the motors are cut. */
        const int free_mm =
                (int) result->distance_mm[SENSE_DIR_FRONT] - CONFIG_FRONT_MARGIN_MM - CONFIG_STOP_SLIDE_MM;
        length_mm = (free_mm < length_mm) ? free_mm : length_mm;
    }

    return (length_mm > 0) ? (uint16_t) length_mm : 0u;
}

/* Apply the right hand rule and send the car where it says. Takes no time at all, but it is its own
 * state so that the decision shows up in the log between the scan and the movement. */
static void updateDecide(void) {
    /* The first decision is the first movement, so this is where the start time is held. Everything
     * before it, calibration and the first scan, happens at a standstill. */
    if (waitingToGo()) {
        if (!go_wait_logged) {
            DBG_PRINTF("nav: ready, driving off at %lu ms\n", (unsigned long) go_at_ms);
            go_wait_logged = true;
        }
        return;
    }

    const Decide_Scan scan = Sense_GetScan();
    pending_action = Decide_NextAction(&scan);

    /* The whole decision on one line: every side with its reading, and the action the rule picked.
     * A direction the lazy scan never looked at shows "--" instead of a number, so a missing
     * measurement is never mistaken for a measured zero. */
    const Sense_Result* const result = Sense_GetResult();
    DBG_PRINTF("nav: decided %s | right %s", Decide_ActionName(pending_action), Sense_StateName(scan.right));
    if (result->measured[SENSE_DIR_RIGHT]) {
        DBG_PRINTF(" %u mm", (unsigned) result->distance_mm[SENSE_DIR_RIGHT]);
    } else {
        DBG_PRINTF(" --");
    }
    DBG_PRINTF(" | front %s", Sense_StateName(scan.front));
    if (result->measured[SENSE_DIR_FRONT]) {
        DBG_PRINTF(" %u mm", (unsigned) result->distance_mm[SENSE_DIR_FRONT]);
    } else {
        DBG_PRINTF(" --");
    }
    DBG_PRINTF(" | left %s", Sense_StateName(scan.left));
    if (result->measured[SENSE_DIR_LEFT]) {
        DBG_PRINTF(" %u mm\n", (unsigned) result->distance_mm[SENSE_DIR_LEFT]);
    } else {
        DBG_PRINTF(" --\n");
    }

    if (pending_action == DECIDE_ACTION_FORWARD) {
        enterState(NAV_STATE_STEP);
        return;
    }

    /* Every turn is taken from the middle of the corridor, so the car first drives up to the right
     * place. The front has to be measured again there: the head is pointing somewhere else after the
     * scan, and for a turn to the right the lazy scan never looked forward at all. */
    enterState(NAV_STATE_MEASURE_FRONT);
}

/* Driving forward along the right wall. */
static void updateStep(void) {
    if (!order_given) {
        /* The step watches the right wall, so the head has to be looking at it before the car moves,
         * otherwise the car would be blind to it for the first stretch. When the previous scan ended
         * on the right side the head is already there and this costs nothing. */
        Sense_PointHead(CONFIG_HEAD_PAN_RIGHT_DEG);
        if (!Sense_IsHeadSettled()) {
            return;
        }

        Drive_StartStep(stepLengthMm(), nav_heading_deg);
        order_given = true;
        return;
    }

    if (!Drive_IsDone()) {
        return;
    }

    /* The filter keeps reporting the wall for a while after the real edge (CONFIG_OPENING_LAG_MM),
     * and the car slides on after the motors are cut. */
    past_edge_mm = (uint16_t) (Drive_GetMmSinceWallSeen() + CONFIG_OPENING_LAG_MM + CONFIG_STOP_SLIDE_MM);

    /* Only the wall going away locates an edge. A step that simply ran out of length ends with the
     * wall still beside the car and the sensor nowhere near an edge. */
    past_edge_valid = (Drive_GetEndReason() == DRIVE_END_WALL_LOST);

    switch (Drive_GetEndReason()) {
        case DRIVE_END_WALL_LOST: enterState(NAV_STATE_CONFIRM); break;
        case DRIVE_END_WALL_TOO_CLOSE: enterState(NAV_STATE_ESCAPE); break;
        default: enterState(NAV_STATE_SCAN); break;
    }
}

/* The right wall went away while driving. Check it once more from a standstill: a single bad reading
 * at a bad angle looks exactly the same as a real opening. */
static void updateConfirm(void) {
    if (!alignToGrid()) {
        return;
    }

    if (!order_given) {
        /* The head is still pointing right from the step, so this costs only the samples. */
        Sense_StartScanDirection(SENSE_DIR_RIGHT);
        order_given = true;
        return;
    }

    if (!Sense_IsDone()) {
        return;
    }

    if (Sense_GetResult()->state[SENSE_DIR_RIGHT] != DECIDE_OPEN) {
        DBG_PRINTF("nav: opening not confirmed, carrying on\n");
        enterState(NAV_STATE_STEP);
        return;
    }

    /* TODO IMPROVEMENT, from the task description: drive another 30 to 50 mm and look again, to tell
     * a real opening from a narrow slit the car does not fit through. */
    pending_action = DECIDE_ACTION_TURN_RIGHT;
    enterState(NAV_STATE_MEASURE_FRONT);
}

static void updateMeasureFront(void) {
    if (!order_given) {
        Sense_PointHead(CONFIG_HEAD_PAN_FRONT_DEG);
        Sense_StartScanDirection(SENSE_DIR_FRONT);
        order_given = true;
        return;
    }

    if (Sense_IsDone()) {
        enterState(NAV_STATE_APPROACH);
    }
}

/* Drive up to the place where the turn is made. Two different targets, depending on where the car is
 * going, and this is the only state that knows the difference. */
static void updateApproach(void) {
    if (!order_given) {
        const Sense_Result* const result = Sense_GetResult();
        const bool front_wall_seen = (result->state[SENSE_DIR_FRONT] == DECIDE_WALL);

        if (pending_action == DECIDE_ACTION_TURN_RIGHT) {
            /* Into an opening on the right. The distance is what matters here, because the opening
             * is beside the car and nothing can be measured about it while driving past. The front
             * wall only serves as a floor, so the car does not run into it on the way. */
            /* What is left to the middle of the opening, less the slide at the end of this approach.
             * Only when an edge was actually located: with no measured edge there is nothing to
             * drive towards, so the turn is taken where the car stands, which after entering a
             * corridor is about the middle of its cell anyway. */
            const int travel_mm =
                    past_edge_valid ? (OPENING_CENTER_FROM_EDGE_MM - (int) past_edge_mm - CONFIG_STOP_SLIDE_MM)
                                    : CONFIG_TURN_NUDGE_MM;

            if (past_edge_valid) {
                DBG_PRINTF("nav: %u mm past the edge, %d mm to the middle\n", (unsigned) past_edge_mm,
                           travel_mm);
            } else {
                DBG_PRINTF("nav: edge not measured, nudging %d mm before the turn\n", travel_mm);
            }
            Drive_StartApproach(CONFIG_FRONT_MARGIN_MM, (travel_mm > 0) ? (uint16_t) travel_mm : 0u,
                                nav_heading_deg);
        } else if (front_wall_seen) {
            /* A corner or a dead end: the wall ahead is the thing to line up against, and it is
             * measured, so this is the accurate way. The cap only stops the car if it never gets
             * there, for instance because it moves slower than assumed. */
            const int remaining_mm = (int) result->distance_mm[SENSE_DIR_FRONT] - APPROACH_FRONT_TARGET_MM;
            Drive_StartApproach(APPROACH_FRONT_TARGET_MM, (remaining_mm > 0) ? (uint16_t) remaining_mm : 0u,
                                nav_heading_deg);
        } else {
            /* The rule sent the car into a turn, but the front gave no usable reading. Nothing can be
             * lined up against, so it turns where it stands rather than driving on blind. */
            DBG_PRINTF("nav: no front wall to approach\n");
            enterState(NAV_STATE_TURN);
            return;
        }

        order_given = true;
        return;
    }

    if (Drive_IsDone()) {
        enterState(NAV_STATE_TURN);
    }
}

/* Turn in place, in pieces of at most 90 degrees.
 *
 * A half turn is deliberately two turns of 90 with a pause between them: around 180 degrees the
 * shortest way in Motion_GetYawError() can flip sign, and a single order of 180 would leave the car
 * hunting back and forth. */
static void updateTurn(void) {
    if (!order_given) {
        turn_remaining_deg = Decide_ActionTurnDegrees(pending_action);
        turn_piece_running = false;
        /* Backdated so that the first piece starts without waiting for the pause. */
        turn_piece_end_ms = Timing_GetMilliseconds() - CONFIG_TURN_PAUSE_MS;
        order_given = true;
        return;
    }

    if (!Drive_IsDone()) {
        return;
    }

    if (turn_piece_running) {
        turn_piece_running = false;
        turn_piece_end_ms = Timing_GetMilliseconds();
    }

    if (turn_remaining_deg == 0) {
        enterState(NAV_STATE_SETTLE);
        return;
    }

    if ((Timing_GetMilliseconds() - turn_piece_end_ms) < CONFIG_TURN_PAUSE_MS) {
        return;
    }

    /* Each piece aims at the next grid heading, not at "current heading plus 90". A first half of a
     * half turn that ended a few degrees short is therefore made up by the second half. */
    const int16_t piece_deg = (int16_t) constrain_int(turn_remaining_deg, -90, 90);
    nav_heading_deg = wrapHeadingDeg((int32_t) nav_heading_deg + piece_deg);
    Drive_StartTurnTo(nav_heading_deg);
    turn_remaining_deg = (int16_t) (turn_remaining_deg - piece_deg);
    turn_sum_deg += piece_deg;
    turn_piece_running = true;
}

/* The step that must follow every turn, before the right side is looked at again.
 *
 * After a RIGHT turn it is long: the car stands in the middle of the old corridor, and its right side
 * looks back down that corridor, which is open. Scanning there sent the car right again, back the way
 * it came, so with the old 80 mm step the right hand rule was broken at every opening on the right.
 * It now drives into the new corridor until the side beam is past the corner. That is a few hundred
 * millimeters without looking right, so the head looks forward and the car stops if a wall is near.
 *
 * After a left turn or a half turn the right wall is right beside the car already, so a short blind
 * step with the head aimed right is enough, and the next scan gets its first direction for free. */
static void updateSettle(void) {
    /* DIAGNOSTIC, temporary: measure both sides before moving, see settle_probe. The car stands
     * still throughout, and the entering move below starts only once this is finished. */
    if (!order_given && (pending_action == DECIDE_ACTION_TURN_RIGHT) && (settle_probe != PROBE_DONE)) {
        switch (settle_probe) {
            case PROBE_LEFT_START: {
                Sense_StartScanDirection(SENSE_DIR_LEFT);
                settle_probe = PROBE_LEFT_WAIT;
                return;
            }
            case PROBE_LEFT_WAIT: {
                if (Sense_IsDone()) {
                    settle_probe = PROBE_RIGHT_START;
                }
                return;
            }
            case PROBE_RIGHT_START: {
                Sense_StartScanDirection(SENSE_DIR_RIGHT);
                settle_probe = PROBE_RIGHT_WAIT;
                return;
            }
            case PROBE_RIGHT_WAIT: {
                if (!Sense_IsDone()) {
                    return;
                }

                const Sense_Result* const probe = Sense_GetResult();
                DBG_PRINTF("nav: after turn: left ");
                if (probe->measured[SENSE_DIR_LEFT] && (probe->distance_mm[SENSE_DIR_LEFT] > 0u)) {
                    DBG_PRINTF("%u mm", (unsigned) probe->distance_mm[SENSE_DIR_LEFT]);
                } else {
                    DBG_PRINTF("no echo");
                }
                DBG_PRINTF(", right ");
                if (probe->measured[SENSE_DIR_RIGHT] && (probe->distance_mm[SENSE_DIR_RIGHT] > 0u)) {
                    DBG_PRINTF("%u mm\n", (unsigned) probe->distance_mm[SENSE_DIR_RIGHT]);
                } else {
                    DBG_PRINTF("no echo\n");
                }

                settle_probe = PROBE_DONE;
                return;
            }
            case PROBE_DONE:
            default: break;
        }
    }

    if (!order_given) {
        if (pending_action == DECIDE_ACTION_TURN_RIGHT) {
            Sense_PointHead(CONFIG_HEAD_PAN_FRONT_DEG);
            Drive_StartApproach(CONFIG_FRONT_MARGIN_MM, ENTER_TRAVEL_MM, nav_heading_deg);
        } else {
            Sense_PointHead(CONFIG_HEAD_PAN_RIGHT_DEG);
            Drive_StartMove(CONFIG_STEP_AFTER_TURN_MM, nav_heading_deg);
        }
        order_given = true;
        return;
    }

    if (Drive_IsDone()) {
        /* Just past the start of the right wall of the new corridor, or right beside it. Constants,
         * not a located edge, so the next right turn must not drive them. */
        past_edge_mm = (pending_action == DECIDE_ACTION_TURN_RIGHT)
                               ? (uint16_t) (CONFIG_ENTER_MARGIN_MM + CONFIG_STOP_SLIDE_MM)
                               : (uint16_t) (Drive_GetLastStepMm() + CONFIG_STOP_SLIDE_MM);
        past_edge_valid = false;
        enterState(NAV_STATE_SCAN);
    }
}

/* The right wall came too close. Turn away from it in place, drive a short way straight, then look
 * around again. Without this the car would stop at the same place on every step and never get
 * anywhere.
 *
 * Three separate movements, never a curve: a tank turn on the spot to CONFIG_ESCAPE_TURN_DEG off the
 * grid heading, a straight move holding that heading, and then the scan, whose alignment turns the
 * car back to the grid heading, again on the spot. Driving forward while swinging round made the car
 * skid on this floor.
 *
 * nav_heading_deg itself is left alone, so the next step goes straight down the corridor again. */
static bool escape_turned = false;

static void updateEscape(void) {
    const int16_t escape_heading_deg = wrapHeadingDeg((int32_t) nav_heading_deg + CONFIG_ESCAPE_TURN_DEG);

    if (!order_given) {
        Drive_StartTurnTo(escape_heading_deg);
        escape_turned = false;
        order_given = true;
        return;
    }

    if (!Drive_IsDone()) {
        return;
    }

    if (!escape_turned) {
        escape_turned = true;
        Drive_StartMove(CONFIG_ESCAPE_MOVE_MM, escape_heading_deg);
        return;
    }

    {
        /* The wall was right there when the escape began, so the car is at most this far past it.
         * An upper bound, not a located edge. */
        past_edge_mm = (uint16_t) (Drive_GetLastStepMm() + CONFIG_STOP_SLIDE_MM);
        past_edge_valid = false;
        enterState(NAV_STATE_SCAN);
    }
}

///////////////////// RUNNING /////////////////////////////////////////////////

void Navigator_Update(void) {
    if (!Navigator_IsRunning()) {
        return;
    }

    /* A run that goes on too long is given up on, so a car stuck in a corner does not grind its
     * motors until the battery is flat. The fuller safety net, with stuck and bump detection, is a
     * separate module. */
    if ((Timing_GetMilliseconds() - run_start_ms) >= CONFIG_RUN_TIMEOUT_MS) {
        DBG_PRINTF("nav: run timed out\n");
        Navigator_Stop();
        return;
    }

    switch (state) {
        case NAV_STATE_CALIBRATE: updateCalibrate(); break;
        case NAV_STATE_SCAN: updateScan(); break;
        case NAV_STATE_DECIDE: updateDecide(); break;
        case NAV_STATE_STEP: updateStep(); break;
        case NAV_STATE_CONFIRM: updateConfirm(); break;
        case NAV_STATE_MEASURE_FRONT: updateMeasureFront(); break;
        case NAV_STATE_APPROACH: updateApproach(); break;
        case NAV_STATE_TURN: updateTurn(); break;
        case NAV_STATE_SETTLE: updateSettle(); break;
        case NAV_STATE_ESCAPE: updateEscape(); break;

        case NAV_STATE_IDLE:
        case NAV_STATE_FINISHED:
        case NAV_STATE_ERROR:
        default: break;
    }
}

Nav_State Navigator_GetState(void) {
    return state;
}

bool Navigator_IsRunning(void) {
    return (state != NAV_STATE_IDLE) && (state != NAV_STATE_FINISHED) && (state != NAV_STATE_ERROR);
}

int32_t Navigator_GetTurnSumDeg(void) {
    return turn_sum_deg;
}

///////////////////// DEBUG ///////////////////////////////////////////////////

const char* Navigator_StateName(Nav_State nav_state) {
    if (((int) nav_state < 0) || ((int) nav_state >= (int) NAV_STATE_COUNT)) {
        return "invalid";
    }
    return state_names[nav_state];
}
