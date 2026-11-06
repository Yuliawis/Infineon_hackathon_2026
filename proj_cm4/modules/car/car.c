#include "car.h"
#include "utils.h"
#include <math.h>
#include "cy_systick.h"
#include "cybsp.h"
#include "PCA9685.h"
#include "motion.h"
#include "distance.h"
#include "cybsp.h"
#include <stdlib.h>

/////////////////////PCA9685 drive area///////////////////////////////////
#define SERVO_FREQUENCY \
    50  //Define the operating frequency of servo (shared with the motors, same as in Freenove sketches)
#define MOTOR_SPEED_MIN  -4095  //Define a minimum speed limit for wheels
#define MOTOR_SPEED_MAX  4095   //Define a maximum speed limit for wheels
#define PIN_MOTOR_M1_IN1 15     //Define the positive pole of M1
#define PIN_MOTOR_M1_IN2 14     //Define the negative pole of M1
#define PIN_MOTOR_M2_IN1 9      //Define the positive pole of M2
#define PIN_MOTOR_M2_IN2 8      //Define the negative pole of M2
#define PIN_MOTOR_M3_IN1 12     //Define the positive pole of M3
#define PIN_MOTOR_M3_IN2 13     //Define the negative pole of M3
#define PIN_MOTOR_M4_IN1 10     //Define the positive pole of M4
#define PIN_MOTOR_M4_IN2 11     //Define the negative pole of M4
// ---- Turn tuning (edit these if the turn angle is wrong) ----
#define TURN_MAX_CORRECTIONS   (2)     // extra slow passes to fix an overshoot (backup only)
#define TURN_COAST_MS_INIT     (150)   // first guess: how long the car keeps rotating after stop
#define TURN_COAST_MS_MAX      (600)   // limit for the learned value
#define TURN_SETTLE_MS         (300)   // wait after stopping, while the car slides
#define TURN_SPEED             (1000)  // max turning speed, far from the target
#define TURN_SPEED_MIN         (750)   // min speed near the target (too low = wheels stall)
#define TURN_SLOWDOWN_DEG      (30)    // start slowing down this many degrees before the target
#define TURN_TOLERANCE_DEG     (4)     // stop when this close to the target
#define TURN_TIMEOUT_MS        (5000u)
#define DRIVE_SPEED            (1000)  // wheel speed while driving straight
#define DRIVE_HEADING_GAIN     (30)    // speed correction per degree of heading error
#define DRIVE_TIMEOUT_MS       (6000u)
#define DRIVE_MM_PER_SECOND    (250u)   // car speed at DRIVE_SPEED, used without a wall ahead. CALIBRATE on the car.
#define DRIVE_SENSOR_MAX_MM    (2000u)  // farther than this the wall ahead is not trusted as a reference
#define DRIVE_SENSOR_MARGIN_MM (100u)   // keep at least this distance to the wall ahead
#define DRIVE_SENSOR_OFFSET_MM (125)     // from the car center forward to the distance sensor. CALIBRATE on the car.
#define DRIVE_STOP_LEAD_MM     (50u)    // stop this much early: sensor lag + coasting after stop. CALIBRATE on the car.
#define DRIVE_SENSOR_TIME_PCT  (150u)   // sensor drive also ends after this % of its expected time (echo lost)
// ---- Position correction after a sensor drive (measure while standing, then creep to the target) ----
#define DRIVE_MAX_CORRECTIONS    (2)      // slow passes to fix the stop position (0 = off)
#define DRIVE_TOLERANCE_MM       (50)     // good enough when this close to the target (sensor noise ~5-10 mm)
#define DRIVE_CORRECT_MAX_MM     (200)    // larger error = reading not trusted, no correction
#define DRIVE_SPEED_MIN          (500)    // creeping speed (too low = wheels stall). CALIBRATE on the car.
#define DRIVE_SETTLE_MS          (100u)   // wait after stopping, while the car slides
#define DRIVE_MEASURE_MS         (170u)   // fill the distance filter while standing (3 measurements, 50 ms each)
#define DRIVE_CORRECT_TIMEOUT_MS (1500u)  // max time of one correction pass
#define DRIVE_CORRECT_NEAR_MM    (400)  // correct only when stopping this close to the wall ahead (last cell before it)


/* 8MHz IMO clock with 8000000 reload value to generate 1s interrupt */
#define SYSTICK_RELOAD_VAL (8000UL)

static volatile uint32_t milliseconds = 0;
// Sound PWM clock frequency. Implies that TCPWM divider is 1 (direct frequency from PERI)
static uint32_t sound_pwm_clock_hz;

///////////////////// MOTORS API //////////////////////////////////////////////

// Motor & servo subsystem initialization
void Motor_Init(void) {
    PCA9685_Init();
    PCA9685_setToFrequency(SERVO_FREQUENCY);
}

// Function to control the car motors
void Motor_Move(int m1_speed, int m2_speed, int m3_speed, int m4_speed) {
    m1_speed = MOTOR_1_DIRECTION * constrain_int(m1_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
    m2_speed = MOTOR_2_DIRECTION * constrain_int(m2_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
    m3_speed = MOTOR_3_DIRECTION * constrain_int(m3_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
    m4_speed = MOTOR_4_DIRECTION * constrain_int(m4_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);

    if (m1_speed >= 0) {
        PCA9685_setChannelPulseWidth(PIN_MOTOR_M1_IN1, m1_speed);
        PCA9685_setChannelPulseWidth(PIN_MOTOR_M1_IN2, 0);
    } else {
        m1_speed = -m1_speed;
        PCA9685_setChannelPulseWidth(PIN_MOTOR_M1_IN1, 0);
        PCA9685_setChannelPulseWidth(PIN_MOTOR_M1_IN2, m1_speed);
    }
    if (m2_speed >= 0) {
        PCA9685_setChannelPulseWidth(PIN_MOTOR_M2_IN1, m2_speed);
        PCA9685_setChannelPulseWidth(PIN_MOTOR_M2_IN2, 0);
    } else {
        m2_speed = -m2_speed;
        PCA9685_setChannelPulseWidth(PIN_MOTOR_M2_IN1, 0);
        PCA9685_setChannelPulseWidth(PIN_MOTOR_M2_IN2, m2_speed);
    }
    if (m3_speed >= 0) {
        PCA9685_setChannelPulseWidth(PIN_MOTOR_M3_IN1, m3_speed);
        PCA9685_setChannelPulseWidth(PIN_MOTOR_M3_IN2, 0);
    } else {
        m3_speed = -m3_speed;
        PCA9685_setChannelPulseWidth(PIN_MOTOR_M3_IN1, 0);
        PCA9685_setChannelPulseWidth(PIN_MOTOR_M3_IN2, m3_speed);
    }
    if (m4_speed >= 0) {
        PCA9685_setChannelPulseWidth(PIN_MOTOR_M4_IN1, m4_speed);
        PCA9685_setChannelPulseWidth(PIN_MOTOR_M4_IN2, 0);
    } else {
        m4_speed = -m4_speed;
        PCA9685_setChannelPulseWidth(PIN_MOTOR_M4_IN1, 0);
        PCA9685_setChannelPulseWidth(PIN_MOTOR_M4_IN2, m4_speed);
    }
}


///////////////////// SOUND API ///////////////////////////////////////////////

//Sound subsystem initialization
void Sound_Init(void) {
    sound_pwm_clock_hz = Cy_SysClk_PeriphGetFrequency(peri_0_div_8_1_HW, peri_0_div_8_1_NUM);
    (void) Cy_TCPWM_PWM_Init(PWM_Buz_HW, PWM_Buz_NUM, &PWM_Buz_config);
    Cy_TCPWM_Enable_Multiple(PWM_Buz_HW, PWM_Buz_MASK);
    Cy_TCPWM_PWM_SetCompare0(PWM_Buz_HW, PWM_Buz_NUM, 0);
    Cy_TCPWM_TriggerStart(PWM_Buz_HW, PWM_Buz_MASK);
}

// Set permanent output (non-blocking) of tone with desired frequency
void Sound_WriteTone(uint32_t freq) {
    uint32_t period;
    if (freq != 0u) {
        period = sound_pwm_clock_hz / freq;
        Cy_TCPWM_PWM_SetPeriod0(PWM_Buz_HW, PWM_Buz_NUM, period);
        Cy_TCPWM_PWM_SetCompare0(PWM_Buz_HW, PWM_Buz_NUM, period / 2);
        Cy_TCPWM_PWM_SetCounter(PWM_Buz_HW, PWM_Buz_NUM, 0);
    } else {
        Cy_TCPWM_PWM_SetCompare0(PWM_Buz_HW, PWM_Buz_NUM, 0);
    }
}

//Play a tone with defined frequency and duraction
void Sound_Play(uint32_t freq, uint32_t duration) {
    Sound_WriteTone(freq);
    Cy_SysLib_Delay(duration);
    Sound_WriteTone(0);
}


///////////////////// TIMING API //////////////////////////////////////////////
static void systick_handler(void) {
    milliseconds++;
}

void Timing_Init(void) {
    /* Enable Systick and the Systick interrupt */
    Cy_SysTick_Enable();

    /* Initialize the systick, set the 8MHz IMO as clock source */
    Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_IMO, SYSTICK_RELOAD_VAL);

    /* Set Systick interrupt callback */
    Cy_SysTick_SetCallback(0, systick_handler);
}

/* Get the number of milliseconds since the timing system was initialized by Timing_Init() */
uint32_t Timing_GetMilliseconds(void) {
    return milliseconds;
}


///////////////////// HEADING ////////////////////////////////////////////////
// Absolute heading (gyro yaw) the car should face. Turns and straight drives aim at it,
// so the leftover error of one turn is fixed by the next move instead of adding up.
static int16_t commanded_yaw = 0;

static int16_t wrap_deg(int32_t a) {
    a %= 360;
    if (a > 180) {
        a -= 360;
    }
    if (a <= -180) {
        a += 360;
    }
    return (int16_t) a;
}

void Car_ResetHeading(void) {
    (void) Motion_Update();
    commanded_yaw = Motion_GetYaw();
}

int16_t Car_GetHeading(void) {
    return commanded_yaw;
}

///////////////////// IDLE HOOK ///////////////////////////////////////////////
// Called on every pass of the blocking loops below (turn, drive, wait), e.g. for periodic debug prints.
static void (*idle_hook)(void) = NULL;

void Car_SetIdleHook(void (*hook)(void)) {
    idle_hook = hook;
}

static inline void runIdleHook(void) {
    if (idle_hook != NULL) {
        idle_hook();
    }
}

/* Turn the car in place by delta_deg degrees (positive = left, negative = right).
 * The turn is relative to the commanded heading, not to the current yaw, so errors do not accumulate.
 * Blocking: returns when the target heading is reached or after TURN_TIMEOUT_MS.
 * Needs Motion_Init(), Timing_Init() and Car_ResetHeading() to be called before. */
void Car_TurnBy(int16_t delta_deg) {
    (void) Motion_Update();  // fresh yaw before taking the start heading
    commanded_yaw = wrap_deg((int32_t) commanded_yaw + delta_deg);
    int16_t target = commanded_yaw;  // Motion_GetYawError() wraps the angle itself

    // Coast time is learned from turn to turn (slippery floor -> it grows by itself)
    static int32_t coast_ms = TURN_COAST_MS_INIT;
    int32_t stop_rate_dps = 0;                                     // rotation speed at the moment the motors stopped
    int16_t turn_dir = (Motion_GetYawError(target) > 0) ? 1 : -1;  // +1 left, -1 right

    // First pass stops early, by the angle the car is expected to slide.
    // The next passes only fix what is left (normally they are not needed).
    for (int pass = 0; pass <= TURN_MAX_CORRECTIONS; pass++) {
        if (abs(Motion_GetYawError(target)) <= TURN_TOLERANCE_DEG) {
            break;
        }
        uint32_t start = Timing_GetMilliseconds();
        while (abs(Motion_GetYawError(target)) > TURN_TOLERANCE_DEG) {
            runIdleHook();
            if (Timing_GetMilliseconds() - start > TURN_TIMEOUT_MS) {
                break;  // safety: the IMU is stuck or the car does not turn
            }
            if (!Motion_Update()) {
                continue;  // no new sensor sample yet
            }
            int16_t err = Motion_GetYawError(target);

            // Predict the slide: angle = rotation speed * coast time. Stop that much earlier.
            if (pass == 0) {
                int32_t rate = Motion_GetYawRate();  // deg/s, positive = left
                bool towardTarget = (rate > 0) == (err > 0);
                int32_t coast_deg = abs(rate) * coast_ms / 1000;
                if (towardTarget && abs(err) <= TURN_TOLERANCE_DEG + coast_deg) {
                    stop_rate_dps = abs(rate);
                    break;
                }
            }

            // Slow down near the target so momentum does not carry the car past it.
            // Correction passes always go slowly.
            int speed = TURN_SPEED_MIN;
            if (pass == 0 && abs(err) >= TURN_SLOWDOWN_DEG) {
                speed = TURN_SPEED;
            } else if (pass == 0) {
                speed = TURN_SPEED_MIN + (TURN_SPEED - TURN_SPEED_MIN) * abs(err) / TURN_SLOWDOWN_DEG;
            }

            // error > 0 means turn left: left wheels backward, right wheels forward
            int s = (err > 0) ? speed : -speed;
            Motor_Move(-s, -s, s, s);  // check on the real car which motors are left/right
        }
        Motor_Move(0, 0, 0, 0);
        Car_Wait(TURN_SETTLE_MS);  // keeps reading the gyro while the car slides to a stop

        // Learn from the first pass: overshoot > 0 means it slid past the target
        if (pass == 0 && stop_rate_dps > 0) {
            int32_t overshoot = -Motion_GetYawError(target) * turn_dir;
            coast_ms += overshoot * 1000 / stop_rate_dps / 2;  // half step, so it does not jump around
            if (coast_ms < 0) {
                coast_ms = 0;
            }
            if (coast_ms > TURN_COAST_MS_MAX) {
                coast_ms = TURN_COAST_MS_MAX;
            }
        }
    }
}

/* Wait ms milliseconds while still reading the gyro, so rotation that happens
 * meanwhile (the car sliding after the motors stop) is not lost. */
void Car_Wait(uint32_t ms) {
    uint32_t start = Timing_GetMilliseconds();
    while (Timing_GetMilliseconds() - start < ms) {
        (void) Motion_Update();
        runIdleHook();
    }
}

/* Front distance at which to stop, or -1 if the wall ahead can't be used as the reference.
 * grid_mm == 0: stop when mm closer than now.
 * grid_mm > 0: walls stand on a grid of grid_mm cells and the car should stop in a cell center, where the
 * wall ahead is k * grid_mm + grid_mm / 2 from the car center. Take the center nearest to "mm farther",
 * so the error of the previous drive is corrected instead of carried on. A correction of more than a quarter
 * cell means the wall is not where the grid puts it (bad reading, other layout): then just mm. */
static int32_t driveStopDistance(uint16_t front_mm, uint16_t mm, uint16_t grid_mm) {
    if (front_mm > DRIVE_SENSOR_MAX_MM) {
        return -1;
    }
    int32_t stop_mm = (int32_t) front_mm - (int32_t) mm;
    if (grid_mm > 0u && stop_mm >= 0) {
        int32_t grid = (int32_t) grid_mm;
        int32_t left_mm = stop_mm + DRIVE_SENSOR_OFFSET_MM;      // car center to the wall after mm
        int32_t center_mm = (left_mm / grid) * grid + grid / 2;  // nearest k * grid + grid / 2
        int32_t correction_mm = center_mm - left_mm;
        if (correction_mm <= grid / 4 && correction_mm >= -grid / 4) {
            stop_mm = center_mm - DRIVE_SENSOR_OFFSET_MM;
        }
    }
    return (stop_mm >= (int32_t) DRIVE_SENSOR_MARGIN_MM) ? stop_mm : -1;
}

/* Motors for a straight drive at speed (negative = backward), heading held by the gyro.
 * error > 0 means the car points too far right, so the left wheels slow down and the right ones speed up.
 * Same turning effect when driving backward, so the correction sign does not change. */
static void driveStraight(int speed, int16_t heading) {
    int correction = Motion_GetYawError(heading) * DRIVE_HEADING_GAIN;
    Motor_Move(speed - correction, speed + correction, speed - correction,
               speed + correction);  // M1, M3 left; M2, M4 right
}

/* After a sensor drive: measure the front distance while standing and creep forward or backward
 * until it is within DRIVE_TOLERANCE_MM of target_mm. Up to DRIVE_MAX_CORRECTIONS passes. */
static void driveCorrect(int32_t target_mm, int16_t heading) {
    for (int pass = 0;; pass++) {
        Car_Wait(DRIVE_SETTLE_MS);  // let the car stop sliding
        Distance_Start();           // forget the readings taken while moving
        Car_Wait(DRIVE_MEASURE_MS);

        uint16_t now_mm;
        if (!Distance_GetMm(&now_mm)) {
            return;  // no reading: nothing to correct against
        }
        int32_t error = (int32_t) now_mm - target_mm;  // > 0: too far from the wall, go forward
        if (abs(error) <= DRIVE_TOLERANCE_MM || abs(error) > DRIVE_CORRECT_MAX_MM || pass >= DRIVE_MAX_CORRECTIONS) {
            return;
        }

        bool forward = error > 0;
        uint32_t start = Timing_GetMilliseconds();
        while (Timing_GetMilliseconds() - start < DRIVE_CORRECT_TIMEOUT_MS) {
            runIdleHook();
            if (!Motion_Update()) {
                continue;  // no new gyro sample yet
            }
            if (Distance_GetMm(&now_mm) && (forward ? (int32_t) now_mm <= target_mm : (int32_t) now_mm >= target_mm)) {
                break;  // reached or crossed the target
            }
            driveStraight(forward ? DRIVE_SPEED_MIN : -DRIVE_SPEED_MIN, heading);
        }
        Motor_Move(0, 0, 0, 0);
    }
}

/* Drive straight forward by about mm millimeters, holding the heading with the gyro.
 * If a wall is seen ahead (head must look forward, distance filter must be fresh), the distance to it is the
 * reference: see driveStopDistance(), the car stops DRIVE_STOP_LEAD_MM early to cover sensor lag and coasting.
 * Otherwise the time is used (DRIVE_MM_PER_SECOND).
 * With the sensor the drive still ends after DRIVE_SENSOR_TIME_PCT of the expected time, so a lost echo or a
 * wrong (too far) reading can't make the car drive on until DRIVE_TIMEOUT_MS.
 * After a sensor drive that ends near the wall (DRIVE_CORRECT_NEAR_MM) the stop position is measured while
 * standing and fixed by slow moves (driveCorrect()).
 * Restarts the distance filter then (Distance_Start()).
 * Blocking. Needs Motion_Init(), Distance_Start() and Timing_Init() before. */
void Car_DriveForward(uint16_t mm, uint16_t grid_mm) {
    uint16_t front_mm = 0;
    int32_t stop_at = Distance_GetMm(&front_mm) ? driveStopDistance(front_mm, mm, grid_mm) : -1;
    bool use_sensor = stop_at >= 0;
    uint16_t stop_mm = use_sensor ? (uint16_t) (stop_at + (int32_t) DRIVE_STOP_LEAD_MM) : 0;
    // with the sensor the planned distance can differ from mm (snapped to a cell center)
    int32_t planned_mm = use_sensor ? (int32_t) front_mm - stop_at : (int32_t) mm;
    if (planned_mm < (int32_t) mm) {
        planned_mm = (int32_t) mm;  // never a shorter time limit than for mm itself
    }
    uint32_t duration_ms = (uint32_t) planned_mm * 1000u / DRIVE_MM_PER_SECOND;
    uint32_t limit_ms = use_sensor ? duration_ms * DRIVE_SENSOR_TIME_PCT / 100u : duration_ms;
    int16_t heading = commanded_yaw;  // absolute heading, so leftover turn error is corrected too
    uint32_t start = Timing_GetMilliseconds();

    while (1) {
        runIdleHook();
        uint32_t elapsed = Timing_GetMilliseconds() - start;
        if (elapsed > DRIVE_TIMEOUT_MS || elapsed >= limit_ms) {
            break;
        }
        if (!Motion_Update()) {
            continue;  // no new gyro sample yet
        }
        uint16_t now_mm;
        if (use_sensor && Distance_GetMm(&now_mm) && now_mm <= stop_mm) {
            break;
        }
        driveStraight(DRIVE_SPEED, heading);
    }
    Motor_Move(0, 0, 0, 0);

    // Fix the position only in front of a wall: there it matters (front wall check, the turn after) and the
    // reading is the most reliable. On the way there the snapping to cell centers keeps errors from adding up.
    if (use_sensor && stop_at <= DRIVE_CORRECT_NEAR_MM) {
        driveCorrect(stop_at, heading);
    }
}
