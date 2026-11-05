#include "drive.h"
#include "config.h"
#include "car.h"
#include "debug_uart.h"
#include "motion.h"
#include "sense.h"
#include "utils.h"
#include <stddef.h>

/* Base output, never below the value the car needs to start moving at all. */
#define STEP_BASE_PWM ((CONFIG_PWM_STEP > CONFIG_PWM_MIN) ? CONFIG_PWM_STEP : CONFIG_PWM_MIN)

typedef enum {
    STATE_IDLE = 0, /* nothing is running */
    STATE_MOVE,     /* forward, watching nothing */
    STATE_STEP,     /* forward, watching the right wall */
    STATE_APPROACH, /* forward, watching the front wall */
    STATE_TURN      /* turning in place to a heading */
} State;

/* A turn is not one movement but a sequence: spin, then stand still while the car slides, then look
 * at where it actually ended up and correct if needed. */
typedef enum {
    TURN_PHASE_SPIN = 0, /* motors on, output ramping down towards the target */
    TURN_PHASE_SETTLE    /* motors off, waiting for the slide to finish */
} TurnPhase;

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

static TurnPhase turn_phase = TURN_PHASE_SPIN;
static uint32_t turn_phase_start_ms = 0;
static uint8_t turn_pass = 0;   /* 0 is the first pass, the rest are slow corrections */
static int16_t turn_direction = 1; /* +1 turning left, -1 turning right, for the overshoot sign */
static int32_t turn_stop_rate_dps = 0; /* how fast the car was rotating when the motors were cut */

/* How long the car keeps rotating after the motors are cut. Learned from turn to turn, so a floor
 * more slippery than expected ends up handled without anybody editing a constant. Deliberately kept
 * across turns and across runs: it describes the floor, not the movement. */
static int32_t turn_coast_ms = CONFIG_TURN_COAST_MS_INIT;

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

static int32_t absInt32(int32_t value) {
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
    /* The commanded angle is scaled to cancel a scale error in the gyro, see
     * CONFIG_TURN_ANGLE_PERCENT. At 100 percent this changes nothing. */
    const int32_t scaled_deg = ((int32_t) delta_deg * CONFIG_TURN_ANGLE_PERCENT) / 100;

    /* Motion_GetYawError() wraps the angle itself, so a sum outside the (-180, 180] range is fine. */
    target_yaw_deg = (int16_t) (Motion_GetYaw() + scaled_deg);
    turn_direction = (Motion_GetYawError(target_yaw_deg) > 0) ? 1 : -1;
    turn_pass = 0;
    turn_phase = TURN_PHASE_SPIN;
    turn_stop_rate_dps = 0;
    move_start_ms = Timing_GetMilliseconds();
    turn_phase_start_ms = move_start_ms;
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

/* Cut the motors and start waiting for the car to stop sliding. */
static void beginTurnSettle(void) {
    setWheels(0, 0);
    turn_phase = TURN_PHASE_SETTLE;
    turn_phase_start_ms = Timing_GetMilliseconds();
}

/* Output for the current distance to the target: full speed far away, ramping down to the minimum at
 * the target, so momentum does not carry the car past it. Correction passes always crawl. */
static int turnSpeedFor(int error_deg) {
    int speed = CONFIG_TURN_SPEED_MIN;

    if (turn_pass == 0u) {
        const int distance_deg = absInt(error_deg);
        if (distance_deg >= CONFIG_TURN_SLOWDOWN_DEG) {
            speed = CONFIG_TURN_SPEED_MAX;
        } else {
            speed = CONFIG_TURN_SPEED_MIN +
                    ((CONFIG_TURN_SPEED_MAX - CONFIG_TURN_SPEED_MIN) * distance_deg) / CONFIG_TURN_SLOWDOWN_DEG;
        }
    }

    return (speed > CONFIG_PWM_MIN) ? speed : CONFIG_PWM_MIN;
}

/* Learn how far the car slides after the motors are cut.
 *
 * The overshoot is how far past the target it ended up, measured in the direction it was turning.
 * Dividing by the rotation speed at the moment of the cut turns that angle back into a time, which is
 * the quantity the prediction needs. Only half of the correction is applied, so the value settles
 * instead of bouncing. */
static void learnCoastTime(int error_deg) {
    if ((turn_pass != 0u) || (turn_stop_rate_dps <= 0)) {
        return;
    }

    const int32_t overshoot_deg = -(int32_t) error_deg * (int32_t) turn_direction;
    turn_coast_ms += (overshoot_deg * 1000) / turn_stop_rate_dps / 2;
    turn_coast_ms = constrain_int((int) turn_coast_ms, 0, CONFIG_TURN_COAST_MS_MAX);

    DBG_PRINTF("drive: turn overshoot %ld deg, coast now %ld ms\n", (long) overshoot_deg,
               (long) turn_coast_ms);
}

/* Turn in place to the target heading.
 *
 * The same approach as the reworked Car_TurnBy() in car.c, which was tuned on slippery tiles, but as
 * a state machine instead of a while loop, so the main loop keeps running. On a slippery floor the
 * car carries on rotating after the motors stop, which is why the motors are cut early by the angle
 * it is expected to slide, and why the result is only judged once the slide has finished. */
static void updateTurn(void) {
    const int error_deg = Motion_GetYawError(target_yaw_deg);
    const uint32_t now_ms = Timing_GetMilliseconds();

    /* One timeout for the whole turn, corrections included. */
    if ((now_ms - move_start_ms) >= move_limit_ms) {
        setWheels(0, 0);
        finish((absInt(error_deg) <= CONFIG_TURN_TOLERANCE_DEG) ? DRIVE_END_TARGET_REACHED
                                                                : DRIVE_END_TIMEOUT);
        return;
    }

    switch (turn_phase) {
        case TURN_PHASE_SPIN: {
            if (absInt(error_deg) <= CONFIG_TURN_TOLERANCE_DEG) {
                beginTurnSettle();
                return;
            }

            /* Stop early by what the car is going to slide: angle = rotation speed * coast time.
             * Only on the first pass, because the corrections crawl and hardly slide at all. */
            if (turn_pass == 0u) {
                const int32_t rate_dps = Motion_GetYawRate();
                const bool turning_towards_target = ((rate_dps > 0) == (error_deg > 0));
                const int32_t coast_deg = (absInt32(rate_dps) * turn_coast_ms) / 1000;

                if (turning_towards_target && (absInt(error_deg) <= CONFIG_TURN_TOLERANCE_DEG + coast_deg)) {
                    turn_stop_rate_dps = absInt32(rate_dps);
                    beginTurnSettle();
                    return;
                }
            }

            /* A positive error means turn left: left wheels backward, right wheels forward. */
            const int speed = turnSpeedFor(error_deg);
            const int signed_speed = (error_deg > 0) ? speed : -speed;
            setWheels(-signed_speed, signed_speed);
            break;
        }

        case TURN_PHASE_SETTLE: {
            /* The car is still sliding. Motion_Update() in the main loop keeps integrating the
             * heading meanwhile, so the rotation that happens here is not lost. */
            if ((now_ms - turn_phase_start_ms) < CONFIG_TURN_SETTLE_MS) {
                return;
            }

            learnCoastTime(error_deg);

            if (absInt(error_deg) <= CONFIG_TURN_TOLERANCE_DEG) {
                finish(DRIVE_END_TARGET_REACHED);
                return;
            }

            if (turn_pass >= CONFIG_TURN_MAX_CORRECTIONS) {
                /* Out of corrections and still off. Driving on slightly crooked beats standing here
                 * and nudging the car for the rest of the run. */
                DBG_PRINTF("drive: turn left %d deg off after %u passes\n", error_deg, (unsigned) turn_pass);
                finish(DRIVE_END_TIMEOUT);
                return;
            }

            ++turn_pass;
            turn_phase = TURN_PHASE_SPIN;
            turn_phase_start_ms = now_ms;
            break;
        }

        default: break;
    }
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
