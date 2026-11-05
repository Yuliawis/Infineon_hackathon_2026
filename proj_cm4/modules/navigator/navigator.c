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

/* How far to drive from the moment the right wall ends until the axis is in the middle of the
 * opening: half a corridor, plus the sensor being ahead of the axis, minus the distance already
 * driven while the filter was catching up. */
#define OPENING_CENTER_TRAVEL_MM \
    ((CONFIG_CORRIDOR_WIDTH_MM / 2) + CONFIG_SENSOR_TO_AXIS_MM - CONFIG_OPENING_LAG_MM)

_Static_assert(APPROACH_FRONT_TARGET_MM > 0, "the sensor cannot sit further ahead than half a corridor");
_Static_assert(OPENING_CENTER_TRAVEL_MM > 0, "the detection lag cannot exceed the whole approach");

static Nav_State state = NAV_STATE_IDLE;

/* False until the one-shot command of the current state has been issued. Every state is written as
 * "give the order once, then wait", and this is what keeps the order from being given again on every
 * pass of the main loop. */
static bool order_given = false;

static uint32_t run_start_ms = 0;
static uint32_t state_start_ms = 0;

/* The action the rule chose, kept while the car drives up to the place where it can carry it out. */
static Decide_Action pending_action = DECIDE_ACTION_FORWARD;

/* Degrees still to turn for the pending action, taken apart into pieces of at most 90 degrees. */
static int16_t turn_remaining_deg = 0;
static bool turn_piece_running = false;
static uint32_t turn_piece_end_ms = 0;

static int32_t turn_sum_deg = 0;

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

static void enterState(Nav_State next) {
    state = next;
    order_given = false;
    state_start_ms = Timing_GetMilliseconds();
    DBG_PRINTF("nav: %s\n", Navigator_StateName(next));
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
    turn_remaining_deg = 0;
    turn_piece_running = false;
    pending_action = DECIDE_ACTION_FORWARD;
    enterState(NAV_STATE_CALIBRATE);
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
    enterState(NAV_STATE_SCAN);
}

static void updateScan(void) {
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
        const int free_mm = (int) result->distance_mm[SENSE_DIR_FRONT] - CONFIG_FRONT_MARGIN_MM;
        length_mm = (free_mm < length_mm) ? free_mm : length_mm;
    }

    return (length_mm > 0) ? (uint16_t) length_mm : 0u;
}

/* Apply the right hand rule and send the car where it says. Takes no time at all, but it is its own
 * state so that the decision shows up in the log between the scan and the movement. */
static void updateDecide(void) {
    const Decide_Scan scan = Sense_GetScan();
    pending_action = Decide_NextAction(&scan);

    DBG_PRINTF("nav: L=%s F=%s R=%s -> %s\n", Sense_StateName(scan.left), Sense_StateName(scan.front),
               Sense_StateName(scan.right), Decide_ActionName(pending_action));

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

        Drive_StartStep(stepLengthMm());
        order_given = true;
        return;
    }

    if (!Drive_IsDone()) {
        return;
    }

    switch (Drive_GetEndReason()) {
        case DRIVE_END_WALL_LOST: enterState(NAV_STATE_CONFIRM); break;
        case DRIVE_END_WALL_TOO_CLOSE: enterState(NAV_STATE_ESCAPE); break;
        default: enterState(NAV_STATE_SCAN); break;
    }
}

/* The right wall went away while driving. Check it once more from a standstill: a single bad reading
 * at a bad angle looks exactly the same as a real opening. */
static void updateConfirm(void) {
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
            Drive_StartApproach(CONFIG_FRONT_MARGIN_MM, OPENING_CENTER_TRAVEL_MM);
        } else if (front_wall_seen) {
            /* A corner or a dead end: the wall ahead is the thing to line up against, and it is
             * measured, so this is the accurate way. The cap only stops the car if it never gets
             * there, for instance because it moves slower than assumed. */
            const int remaining_mm = (int) result->distance_mm[SENSE_DIR_FRONT] - APPROACH_FRONT_TARGET_MM;
            Drive_StartApproach(APPROACH_FRONT_TARGET_MM, (remaining_mm > 0) ? (uint16_t) remaining_mm : 0u);
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

    const int16_t piece_deg = (int16_t) constrain_int(turn_remaining_deg, -90, 90);
    Drive_StartTurn(piece_deg);
    turn_remaining_deg = (int16_t) (turn_remaining_deg - piece_deg);
    turn_sum_deg += piece_deg;
    turn_piece_running = true;
}

/* The short step that must follow every turn.
 *
 * Without it the car looks right again from the same spot, sees the opening it has just turned into,
 * and turns into it again, forever. The head is aimed right during the step, so the servo travels
 * while the car drives and the next scan gets its first direction for free. */
static void updateSettle(void) {
    if (!order_given) {
        Sense_PointHead(CONFIG_HEAD_PAN_RIGHT_DEG);
        Drive_StartMove(CONFIG_STEP_AFTER_TURN_MM);
        order_given = true;
        return;
    }

    if (Drive_IsDone()) {
        enterState(NAV_STATE_SCAN);
    }
}

/* The right wall came too close. Turn a little away from it, then look around again. Without this
 * the car would stop at the same place on every step and never get anywhere. */
static void updateEscape(void) {
    if (!order_given) {
        Drive_StartTurn(CONFIG_ESCAPE_TURN_DEG);
        turn_sum_deg += CONFIG_ESCAPE_TURN_DEG;
        order_given = true;
        return;
    }

    if (Drive_IsDone()) {
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
