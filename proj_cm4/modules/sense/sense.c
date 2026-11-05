#include "sense.h"
#include "config.h"
#include "car.h"
#include "distance.h"
#include "head.h"
#include <stddef.h>

/* Where the head looks for each direction. Pan is positive to the left, so the right side is
 * negative (see head.h). */
static const int8_t direction_pan_deg[SENSE_DIR_COUNT] = {
        [SENSE_DIR_RIGHT] = CONFIG_HEAD_PAN_RIGHT_DEG,
        [SENSE_DIR_FRONT] = CONFIG_HEAD_PAN_FRONT_DEG,
        [SENSE_DIR_LEFT] = CONFIG_HEAD_PAN_LEFT_DEG,
};

/* Above this distance a direction counts as open. One threshold per direction, because the sensor
 * looks straight at the wall in front but at a much worse angle to the sides. */
static const uint16_t direction_open_mm[SENSE_DIR_COUNT] = {
        [SENSE_DIR_RIGHT] = CONFIG_RIGHT_OPEN_MM,
        [SENSE_DIR_FRONT] = CONFIG_FRONT_OPEN_MM,
        [SENSE_DIR_LEFT] = CONFIG_LEFT_OPEN_MM,
};

static const char* const direction_names[SENSE_DIR_COUNT] = {
        [SENSE_DIR_RIGHT] = "right",
        [SENSE_DIR_FRONT] = "front",
        [SENSE_DIR_LEFT] = "left",
};

_Static_assert(sizeof(direction_pan_deg) / sizeof(direction_pan_deg[0]) == SENSE_DIR_COUNT,
               "direction_pan_deg must have one entry per direction");
_Static_assert(sizeof(direction_open_mm) / sizeof(direction_open_mm[0]) == SENSE_DIR_COUNT,
               "direction_open_mm must have one entry per direction");
_Static_assert(sizeof(direction_names) / sizeof(direction_names[0]) == SENSE_DIR_COUNT,
               "direction_names must have one entry per direction");

/* Phases of a scan. One direction goes AIM -> SAMPLE, then either the next direction or DONE. */
typedef enum {
    PHASE_IDLE = 0, /* no scan has been started yet */
    PHASE_AIM,      /* the head is travelling to the direction */
    PHASE_SAMPLE,   /* collecting fresh measurements at that angle */
    PHASE_DONE      /* the result is ready to read */
} Phase;

static Phase phase = PHASE_IDLE;
static Sense_Direction current_direction = SENSE_DIR_RIGHT;
static uint8_t retries_left = 0;
static Sense_Result result;

/* True when only the one direction that was asked for is measured, see Sense_StartScanDirection(). */
static bool single_direction = false;

/* Value of Distance_GetCount() when the current set of samples began, and when that was. The
 * difference of the counter, not a time, is what tells a fresh measurement from an old one. */
static uint32_t sample_baseline = 0;
static uint32_t sample_start_ms = 0;

/* When the head was last commanded to a new angle. Shared by the scan and by Sense_PointHead(). */
static uint32_t head_move_start_ms = 0;

///////////////////// HEAD AIMING /////////////////////////////////////////////

void Sense_PointHead(int8_t pan_degrees) {
    /* Head_GetPan() returns the last commanded angle, already cut to the head limits, so asking for
     * an angle outside the limits twice in a row is correctly recognized as "no change". Not
     * restarting the timer matters: during a step this is called on every pass of the main loop. */
    if (pan_degrees == Head_GetPan()) {
        return;
    }

    Head_SetPan(pan_degrees);
    head_move_start_ms = Timing_GetMilliseconds();
}

bool Sense_IsHeadSettled(void) {
    return (Timing_GetMilliseconds() - head_move_start_ms) >= (uint32_t) CONFIG_HEAD_SETTLE_MS;
}

bool Sense_GetDistanceMm(uint16_t* distance_mm) {
    if (!Sense_IsHeadSettled()) {
        return false;
    }
    return Distance_GetMm(distance_mm);
}

///////////////////// INIT ////////////////////////////////////////////////////

bool Sense_Init(void) {
    /* Automatic mode: the driver measures every 50 ms on its own and the median filter runs in its
     * interrupt, so no measuring call is needed in the main loop. */
    if (!Distance_Init(DISTANCE_MODE_AUTO)) {
        return false;
    }
    Distance_Start();

    phase = PHASE_IDLE;
    current_direction = SENSE_DIR_RIGHT;
    retries_left = 0;

    for (int i = 0; i < (int) SENSE_DIR_COUNT; ++i) {
        result.state[i] = DECIDE_UNKNOWN;
        result.distance_mm[i] = 0;
        result.measured[i] = false;
    }

    /* The head spends most of the run looking at the right wall, so that is where it starts. */
    Sense_PointHead(CONFIG_HEAD_PAN_RIGHT_DEG);
    return true;
}

///////////////////// SCANNING ////////////////////////////////////////////////

/* Point the head at a direction and start waiting for the servo. */
static void beginDirection(Sense_Direction direction) {
    current_direction = direction;
    retries_left = (uint8_t) CONFIG_SCAN_RETRIES;
    phase = PHASE_AIM;
    Sense_PointHead(direction_pan_deg[direction]);
}

/* Start collecting a fresh set of measurements at the current head angle.
 * The baseline is taken here, after the head has arrived, so that the samples taken while the head
 * was still moving are not counted. */
static void beginSampling(void) {
    sample_baseline = Distance_GetCount();
    sample_start_ms = Timing_GetMilliseconds();
    phase = PHASE_SAMPLE;
}

/* Store the state of the current direction and either go on to the next one or finish. */
static void finishDirection(Decide_SideState state, uint16_t distance_mm) {
    result.state[current_direction] = state;
    result.distance_mm[current_direction] = distance_mm;
    result.measured[current_direction] = true;

    if (single_direction) {
        phase = PHASE_DONE;
        return;
    }

    /* Lazy scanning: an open direction already settles the action, because the scanning order is the
     * priority order of the rule. The directions left unmeasured stay unknown, and unknown counts as
     * a wall, which is the lowest priority, so they cannot change the outcome. */
    if (CONFIG_SCAN_LAZY && (state == DECIDE_OPEN)) {
        phase = PHASE_DONE;
        return;
    }

    if ((int) current_direction + 1 >= (int) SENSE_DIR_COUNT) {
        phase = PHASE_DONE;
        return;
    }

    beginDirection((Sense_Direction) ((int) current_direction + 1));
}

/* Turn the filtered reading into a state, or ask for another set of samples. */
static void classifyCurrentDirection(void) {
    uint16_t distance_mm = 0;

    if (!Distance_GetMm(&distance_mm)) {
        /* No valid echo. An ultrasonic pulse that hits a wall at a steep angle often never comes
         * back, so this is not free space and the only honest thing to do is to try again. */
        if (retries_left > 0u) {
            --retries_left;
            beginSampling();
            return;
        }
        finishDirection(DECIDE_UNKNOWN, 0);
        return;
    }

    /* Exactly at the threshold counts as a wall: the conservative side. */
    const Decide_SideState state =
            (distance_mm > direction_open_mm[current_direction]) ? DECIDE_OPEN : DECIDE_WALL;
    finishDirection(state, distance_mm);
}

void Sense_StartScan(void) {
    for (int i = 0; i < (int) SENSE_DIR_COUNT; ++i) {
        result.state[i] = DECIDE_UNKNOWN;
        result.distance_mm[i] = 0;
        result.measured[i] = false;
    }

    single_direction = false;
    beginDirection(SENSE_DIR_RIGHT);
}

void Sense_StartScanDirection(Sense_Direction direction) {
    if (((int) direction < 0) || ((int) direction >= (int) SENSE_DIR_COUNT)) {
        return;
    }

    /* The other directions keep their previous values on purpose: this is a fresh look at one
     * direction, not a new picture of the surroundings. */
    result.state[direction] = DECIDE_UNKNOWN;
    result.distance_mm[direction] = 0;
    result.measured[direction] = false;

    single_direction = true;
    beginDirection(direction);
}

void Sense_Update(void) {
    switch (phase) {
        case PHASE_AIM: {
            if (Sense_IsHeadSettled()) {
                beginSampling();
            }
            break;
        }

        case PHASE_SAMPLE: {
            /* The filter window holds the readings from the previous head angle, so it takes a full
             * window of fresh measurements to flush them out. See CONFIG_SCAN_SAMPLES. */
            if ((Distance_GetCount() - sample_baseline) >= (uint32_t) CONFIG_SCAN_SAMPLES) {
                classifyCurrentDirection();
                break;
            }

            /* The counter stopped moving, so the sensor is not measuring at all. Retrying would only
             * time out again, so this direction is reported unknown straight away and the run goes
             * on instead of stopping here. */
            if ((Timing_GetMilliseconds() - sample_start_ms) >= CONFIG_SCAN_TIMEOUT_MS) {
                finishDirection(DECIDE_UNKNOWN, 0);
            }
            break;
        }

        case PHASE_IDLE:
        case PHASE_DONE:
        default: break;
    }
}

bool Sense_IsDone(void) {
    return phase == PHASE_DONE;
}

const Sense_Result* Sense_GetResult(void) {
    return &result;
}

Decide_Scan Sense_GetScan(void) {
    const Decide_Scan scan = {
            .left = result.state[SENSE_DIR_LEFT],
            .front = result.state[SENSE_DIR_FRONT],
            .right = result.state[SENSE_DIR_RIGHT],
    };
    return scan;
}

///////////////////// DEBUG ///////////////////////////////////////////////////

const char* Sense_DirectionName(Sense_Direction direction) {
    if (((int) direction < 0) || ((int) direction >= (int) SENSE_DIR_COUNT)) {
        return "invalid";
    }
    return direction_names[direction];
}

const char* Sense_StateName(Decide_SideState state) {
    switch (state) {
        case DECIDE_UNKNOWN: return "unknown";
        case DECIDE_WALL: return "wall";
        case DECIDE_OPEN: return "open";
        default: return "invalid";
    }
}
