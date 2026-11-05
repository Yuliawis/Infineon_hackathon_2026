#include "drive.h"
#include "config.h"
#include "car.h"
#include "debug_uart.h"
#include "motion.h"
#include "sense.h"
#include "utils.h"
#include <stddef.h>

/* Base outputs, never below the value the car needs to start moving at all. */
#define STEP_BASE_PWM ((CONFIG_PWM_STEP > CONFIG_PWM_MIN) ? CONFIG_PWM_STEP : CONFIG_PWM_MIN)
#define TURN_BASE_PWM ((CONFIG_PWM_TURN > CONFIG_PWM_MIN) ? CONFIG_PWM_TURN : CONFIG_PWM_MIN)

typedef enum {
    STATE_IDLE = 0, /* nothing is running */
    STATE_MOVE,     /* forward, watching nothing */
    STATE_STEP,     /* forward, watching the right wall */
    STATE_APPROACH, /* forward, watching the front wall */
    STATE_TURN      /* turning in place to a heading */
} State;

static State state = STATE_IDLE;
static Drive_EndReason end_reason = DRIVE_END_NONE;

/* Heading the car holds while stepping, or turns to while turning. */
static int16_t target_yaw_deg = 0;

static uint32_t move_start_ms = 0;
static uint32_t move_limit_ms = 0; /* how long the movement may last, not a deadline */
static uint16_t approach_stop_mm = 0;
static uint16_t last_step_mm = 0;

/* Speed used to turn a length into a time. Starts as the guess from config.h and is replaced by the
 * car's own measurements, see the speed calibration section. */
static uint16_t speed_mmps = CONFIG_SPEED_FORWARD_MMPS;
static bool speed_measured = false;

/* First and last usable front readings of the current approach, with the times they were taken. */
static uint16_t approach_first_mm = 0;
static uint32_t approach_first_ms = 0;
static uint16_t approach_last_mm = 0;
static uint32_t approach_last_ms = 0;
static bool approach_first_valid = false;
static bool approach_last_valid = false;

static const char* const end_reason_names[DRIVE_END_COUNT] = {
        [DRIVE_END_NONE] = "none",
        [DRIVE_END_LENGTH] = "length",
        [DRIVE_END_TARGET_REACHED] = "target reached",
        [DRIVE_END_WALL_LOST] = "wall lost",
        [DRIVE_END_WALL_TOO_CLOSE] = "wall too close",
        [DRIVE_END_TIMEOUT] = "timeout",
        [DRIVE_END_STOPPED] = "stopped",
};

_Static_assert(sizeof(end_reason_names) / sizeof(end_reason_names[0]) == DRIVE_END_COUNT,
               "end_reason_names must have one entry per end reason");

///////////////////// LOW LEVEL ///////////////////////////////////////////////

/* The only place in the whole algorithm that talks to the motors. Which Motor_Move() argument sits
 * on which side of the car comes from config.h, so a swapped pair of wheels is a one line fix there
 * and changes nothing here. */
static void setWheels(int left, int right) {
    Motor_Move(CONFIG_MOTOR_M1_IS_LEFT ? left : right, CONFIG_MOTOR_M2_IS_LEFT ? left : right,
               CONFIG_MOTOR_M3_IS_LEFT ? left : right, CONFIG_MOTOR_M4_IS_LEFT ? left : right);
}

static int absInt(int value) {
    return (value < 0) ? -value : value;
}

/* How long a length takes at the current speed. Deliberately without any safety margin: a step that
 * ends too early only costs an extra scan, while a step that runs too long drives into a wall. */
static uint32_t lengthToTimeMs(uint16_t length_mm) {
    return ((uint32_t) length_mm * 1000u) / (uint32_t) speed_mmps;
}

static uint16_t timeToLengthMm(uint32_t elapsed_ms) {
    const uint32_t length_mm = (elapsed_ms * (uint32_t) speed_mmps) / 1000u;
    return (length_mm > (uint32_t) UINT16_MAX) ? UINT16_MAX : (uint16_t) length_mm;
}

/* How far the car travels while the median filter catches up with a steadily changing distance.
 *
 * A median of three delays a steady ramp by exactly one sample, so the reading at the end of an
 * approach is that much behind the real distance. Leaving it out would make every measured speed too
 * low, and a speed that is too low turns into a time that is too long and a car that overshoots. */
static uint16_t filterLagMm(void) {
    return (uint16_t) (((uint32_t) speed_mmps * (uint32_t) CONFIG_DISTANCE_PERIOD_MS) / 1000u);
}

/* Work out the speed from the two ends of the approach that has just finished. */
static void measureSpeed(void) {
    if (!approach_first_valid || !approach_last_valid) {
        return; /* the front wall was not visible at both ends */
    }

    if (approach_last_mm >= approach_first_mm) {
        return; /* the distance did not shrink, so this was not a clean run at a wall */
    }

    const uint32_t elapsed_ms = approach_last_ms - approach_first_ms;
    if (elapsed_ms < CONFIG_SPEED_SAMPLE_MIN_MS) {
        return;
    }

    const uint32_t travel_mm = (uint32_t) (approach_first_mm - approach_last_mm) + filterLagMm();
    if (travel_mm < (uint32_t) CONFIG_SPEED_SAMPLE_MIN_MM) {
        return;
    }

    const int measured_mmps =
            constrain_int((int) ((travel_mm * 1000u) / elapsed_ms), CONFIG_SPEED_MIN_MMPS, CONFIG_SPEED_MAX_MMPS);

    /* Half old, half new. Slow enough that one odd measurement cannot throw the car off, quick
     * enough to follow the battery going flat over a run. */
    speed_mmps = (uint16_t) (((int) speed_mmps + measured_mmps) / 2);
    speed_measured = true;

    DBG_PRINTF("drive: speed %d mmps measured over %lu mm, now using %u\n", measured_mmps,
               (unsigned long) travel_mm, (unsigned) speed_mmps);
}

static void finish(Drive_EndReason reason) {
    setWheels(0, 0);

    /* The approach is the one movement with a ruler along it, so this is where the speed gets
     * checked against reality. Done before last_step_mm, so that estimate uses the fresh speed. */
    if (state == STATE_APPROACH) {
        measureSpeed();
    }

    if ((state == STATE_MOVE) || (state == STATE_STEP) || (state == STATE_APPROACH)) {
        last_step_mm = timeToLengthMm(Timing_GetMilliseconds() - move_start_ms);
    }

    state = STATE_IDLE;
    end_reason = reason;
}

/* Hold the heading with the gyro. The wall is not used for steering at all: the ultrasonic sensor is
 * only trustworthy near a right angle to the wall, and a car that keeps correcting on the wall keeps
 * turning away from that angle and spoils its own readings. The gyro has no such problem. */
static void driveStraight(void) {
    int error_deg = Motion_GetYawError(target_yaw_deg);
    error_deg = constrain_int(error_deg, -CONFIG_HEADING_CORRECTION_MAX_DEG, CONFIG_HEADING_CORRECTION_MAX_DEG);

    /* A positive error means the car has to turn left, so the left wheels slow down and the right
     * ones speed up. */
    const int correction = error_deg * CONFIG_HEADING_GAIN_PWM_PER_DEG;
    setWheels(STEP_BASE_PWM - correction, STEP_BASE_PWM + correction);
}

///////////////////// INIT ////////////////////////////////////////////////////

void Drive_Init(void) {
    setWheels(0, 0);
    state = STATE_IDLE;
    end_reason = DRIVE_END_NONE;
    target_yaw_deg = 0;
    move_start_ms = 0;
    move_limit_ms = 0;
    approach_stop_mm = 0;
    last_step_mm = 0;
    approach_first_valid = false;
    approach_last_valid = false;

    /* speed_mmps is left alone on purpose: a speed the car has measured is worth more than the guess
     * in config.h, and it stays valid across runs as long as the battery and the floor do not
     * change. */
}

///////////////////// MOVEMENTS ///////////////////////////////////////////////

void Drive_StartStep(uint16_t length_mm) {
    target_yaw_deg = Motion_GetYaw(); /* hold whatever heading the car has now */
    move_start_ms = Timing_GetMilliseconds();
    move_limit_ms = lengthToTimeMs(length_mm);
    last_step_mm = 0;
    end_reason = DRIVE_END_NONE;
    state = STATE_STEP;
}

void Drive_StartMove(uint16_t length_mm) {
    target_yaw_deg = Motion_GetYaw();
    move_start_ms = Timing_GetMilliseconds();
    move_limit_ms = lengthToTimeMs(length_mm);
    last_step_mm = 0;
    end_reason = DRIVE_END_NONE;
    state = STATE_MOVE;
}

void Drive_StartApproach(uint16_t stop_front_mm, uint16_t max_length_mm) {
    target_yaw_deg = Motion_GetYaw();
    move_start_ms = Timing_GetMilliseconds();
    move_limit_ms = lengthToTimeMs(max_length_mm);
    approach_stop_mm = stop_front_mm;
    last_step_mm = 0;
    approach_first_valid = false;
    approach_last_valid = false;
    end_reason = DRIVE_END_NONE;
    state = STATE_APPROACH;
}

void Drive_StartTurn(int16_t delta_deg) {
    /* Motion_GetYawError() wraps the angle itself, so a sum outside the (-180, 180] range is fine. */
    target_yaw_deg = (int16_t) (Motion_GetYaw() + delta_deg);
    move_start_ms = Timing_GetMilliseconds();
    move_limit_ms = CONFIG_TURN_TIMEOUT_MS;
    end_reason = DRIVE_END_NONE;
    state = STATE_TURN;
}

void Drive_Stop(void) {
    finish(DRIVE_END_STOPPED);
}

///////////////////// RUNNING /////////////////////////////////////////////////

/* Forward with no opinion about the surroundings: only the heading and the clock matter. */
static void updateMove(void) {
    if ((Timing_GetMilliseconds() - move_start_ms) >= move_limit_ms) {
        finish(DRIVE_END_LENGTH);
        return;
    }

    driveStraight();
}

/* Forward while following the right wall. */
static void updateStep(void) {
    uint16_t wall_mm = 0;

    /* A reading without a valid echo gives nothing here. An ultrasonic pulse that hits a wall at a
     * steep angle often never comes back, so silence may mean a wall just as well as open space, and
     * ending the step on it would invent openings that are not there. */
    if (Sense_GetDistanceMm(&wall_mm)) {
        if (wall_mm < CONFIG_RIGHT_TOO_CLOSE_MM) {
            finish(DRIVE_END_WALL_TOO_CLOSE);
            return;
        }

        /* The same threshold a scan uses to call a direction open, on purpose: "the wall ended while
         * driving" and "that direction is open" have to mean the same thing, or the car would find
         * openings on the move that it then fails to confirm at a standstill. */
        if (wall_mm > CONFIG_RIGHT_OPEN_MM) {
            finish(DRIVE_END_WALL_LOST);
            return;
        }
    }

    if ((Timing_GetMilliseconds() - move_start_ms) >= move_limit_ms) {
        finish(DRIVE_END_LENGTH);
        return;
    }

    driveStraight();
}

/* Forward until the front wall is at the wanted distance. */
static void updateApproach(void) {
    uint16_t front_mm = 0;

    if (Sense_GetDistanceMm(&front_mm)) {
        /* Both ends of the approach are remembered, so the speed can be worked out from them when
         * the movement finishes, whichever way it ends. */
        const uint32_t now_ms = Timing_GetMilliseconds();
        if (!approach_first_valid) {
            approach_first_mm = front_mm;
            approach_first_ms = now_ms;
            approach_first_valid = true;
        }
        approach_last_mm = front_mm;
        approach_last_ms = now_ms;
        approach_last_valid = true;

        if (front_mm <= approach_stop_mm) {
            finish(DRIVE_END_TARGET_REACHED);
            return;
        }
    }

    if ((Timing_GetMilliseconds() - move_start_ms) >= move_limit_ms) {
        finish(DRIVE_END_LENGTH);
        return;
    }

    driveStraight();
}

/* Turn in place to the target heading. Same idea as Car_TurnBy() in car.c, without the while loop. */
static void updateTurn(void) {
    const int error_deg = Motion_GetYawError(target_yaw_deg);

    if (absInt(error_deg) <= CONFIG_TURN_TOLERANCE_DEG) {
        finish(DRIVE_END_TARGET_REACHED);
        return;
    }

    if ((Timing_GetMilliseconds() - move_start_ms) >= move_limit_ms) {
        finish(DRIVE_END_TIMEOUT);
        return;
    }

    /* A positive error means turn left: left wheels backward, right wheels forward. */
    const int speed = (error_deg > 0) ? TURN_BASE_PWM : -TURN_BASE_PWM;
    setWheels(-speed, speed);
}

void Drive_Update(void) {
    switch (state) {
        case STATE_MOVE: updateMove(); break;
        case STATE_STEP: updateStep(); break;
        case STATE_APPROACH: updateApproach(); break;
        case STATE_TURN: updateTurn(); break;
        case STATE_IDLE:
        default: break;
    }
}

bool Drive_IsDone(void) {
    return state == STATE_IDLE;
}

Drive_EndReason Drive_GetEndReason(void) {
    return end_reason;
}

uint16_t Drive_GetLastStepMm(void) {
    return last_step_mm;
}

uint16_t Drive_GetSpeedMmps(void) {
    return speed_mmps;
}

bool Drive_IsSpeedMeasured(void) {
    return speed_measured;
}

///////////////////// DEBUG ///////////////////////////////////////////////////

const char* Drive_EndReasonName(Drive_EndReason reason) {
    if (((int) reason < 0) || ((int) reason >= (int) DRIVE_END_COUNT)) {
        return "invalid";
    }
    return end_reason_names[reason];
}
