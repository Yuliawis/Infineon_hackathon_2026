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
#define SERVO_FREQUENCY 50      //Define the operating frequency of servo (shared with the motors, same as in Freenove sketches)
#define MOTOR_SPEED_MIN -4095    //Define a minimum speed limit for wheels
#define MOTOR_SPEED_MAX 4095     //Define a maximum speed limit for wheels
#define PIN_MOTOR_M1_IN1 15      //Define the positive pole of M1
#define PIN_MOTOR_M1_IN2 14      //Define the negative pole of M1
#define PIN_MOTOR_M2_IN1 9       //Define the positive pole of M2
#define PIN_MOTOR_M2_IN2 8       //Define the negative pole of M2
#define PIN_MOTOR_M3_IN1 12      //Define the positive pole of M3
#define PIN_MOTOR_M3_IN2 13      //Define the negative pole of M3
#define PIN_MOTOR_M4_IN1 10      //Define the positive pole of M4
#define PIN_MOTOR_M4_IN2 11      //Define the negative pole of M4
// ---- Turn tuning (edit these if the turn angle is wrong) ----
#define TURN_MAX_CORRECTIONS (2)    // extra slow passes to fix an overshoot (backup only)
#define TURN_COAST_MS_INIT  (150)   // first guess: how long the car keeps rotating after stop
#define TURN_COAST_MS_MAX   (600)   // limit for the learned value
#define TURN_SETTLE_MS      (300)   // wait after stopping, while the car slides
#define TURN_SPEED          (1000)  // max turning speed, far from the target
#define TURN_SPEED_MIN      (700)   // min speed near the target (too low = wheels stall)
#define TURN_SLOWDOWN_DEG   (30)    // start slowing down this many degrees before the target
#define TURN_ANGLE_PERCENT  (100)   // turns too far -> lower (e.g. 80), too short -> raise
#define TURN_TOLERANCE_DEG  (5)     // stop when this close to the target
#define TURN_TIMEOUT_MS     (5000u)
#define DRIVE_SPEED         (1500)    // wheel speed while driving straight
#define DRIVE_HEADING_GAIN  (30)      // speed correction per degree of heading error
#define DRIVE_TIMEOUT_MS    (6000u)
#define DRIVE_MM_PER_SECOND (250u)    // car speed at DRIVE_SPEED, used without a wall ahead. CALIBRATE on the car.
#define DRIVE_SENSOR_MAX_MM (2000u)   // farther than this the wall ahead is not trusted as a reference
#define DRIVE_SENSOR_MARGIN_MM (100u) // keep at least this distance to the wall ahead


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
uint32_t Timing_GetMilliseconds(void)
{
   return milliseconds;
}


///////////////////// IDLE HOOK ///////////////////////////////////////////////
// Called on every pass of the blocking loops below (turn, drive, wait), e.g. for periodic debug prints.
static void (*idle_hook)(void) = NULL;

void Car_SetIdleHook(void (*hook)(void))
{
    idle_hook = hook;
}

static inline void runIdleHook(void)
{
    if (idle_hook != NULL)
    {
        idle_hook();
    }
}

/* Turn the car in place by delta_deg degrees (positive = left, negative = right).
 * Blocking: returns when the target heading is reached or after TURN_TIMEOUT_MS.
 * Needs Motion_Init() and Timing_Init() to be called before. */
void Car_TurnBy(int16_t delta_deg)
{
    (void)Motion_Update();   // fresh yaw before taking the start heading
    int16_t scaled = (int16_t)((int32_t)delta_deg * TURN_ANGLE_PERCENT / 100);
    int16_t target = Motion_GetYaw() + scaled;   // Motion_GetYawError() wraps the angle itself

    // Coast time is learned from turn to turn (slippery floor -> it grows by itself)
    static int32_t coast_ms = TURN_COAST_MS_INIT;
    int32_t stop_rate_dps = 0;   // rotation speed at the moment the motors stopped
    int16_t turn_dir = (Motion_GetYawError(target) > 0) ? 1 : -1;   // +1 left, -1 right

    // First pass stops early, by the angle the car is expected to slide.
    // The next passes only fix what is left (normally they are not needed).
    for (int pass = 0; pass <= TURN_MAX_CORRECTIONS; pass++)
    {
        if (abs(Motion_GetYawError(target)) <= TURN_TOLERANCE_DEG)
        {
            break;
        }
        uint32_t start = Timing_GetMilliseconds();
        while (abs(Motion_GetYawError(target)) > TURN_TOLERANCE_DEG)
        {
            runIdleHook();
            if (Timing_GetMilliseconds() - start > TURN_TIMEOUT_MS)
            {
                break;   // safety: the IMU is stuck or the car does not turn
            }
            if (!Motion_Update())
            {
                continue;   // no new sensor sample yet
            }
            int16_t err = Motion_GetYawError(target);

            // Predict the slide: angle = rotation speed * coast time. Stop that much earlier.
            if (pass == 0)
            {
                int32_t rate = Motion_GetYawRate();   // deg/s, positive = left
                bool towardTarget = (rate > 0) == (err > 0);
                int32_t coast_deg = abs(rate) * coast_ms / 1000;
                if (towardTarget && abs(err) <= TURN_TOLERANCE_DEG + coast_deg)
                {
                    stop_rate_dps = abs(rate);
                    break;
                }
            }

            // Slow down near the target so momentum does not carry the car past it.
            // Correction passes always go slowly.
            int speed = TURN_SPEED_MIN;
            if (pass == 0 && abs(err) >= TURN_SLOWDOWN_DEG)
            {
                speed = TURN_SPEED;
            }
            else if (pass == 0)
            {
                speed = TURN_SPEED_MIN + (TURN_SPEED - TURN_SPEED_MIN) * abs(err) / TURN_SLOWDOWN_DEG;
            }

            // error > 0 means turn left: left wheels backward, right wheels forward
            int s = (err > 0) ? speed : -speed;
            Motor_Move(-s, -s, s, s);   // check on the real car which motors are left/right
        }
        Motor_Move(0, 0, 0, 0);
        Car_Wait(TURN_SETTLE_MS);   // keeps reading the gyro while the car slides to a stop

        // Learn from the first pass: overshoot > 0 means it slid past the target
        if (pass == 0 && stop_rate_dps > 0)
        {
            int32_t overshoot = -Motion_GetYawError(target) * turn_dir;
            coast_ms += overshoot * 1000 / stop_rate_dps / 2;   // half step, so it does not jump around
            if (coast_ms < 0) { coast_ms = 0; }
            if (coast_ms > TURN_COAST_MS_MAX) { coast_ms = TURN_COAST_MS_MAX; }
        }
    }
}

/* Wait ms milliseconds while still reading the gyro, so rotation that happens
 * meanwhile (the car sliding after the motors stop) is not lost. */
void Car_Wait(uint32_t ms)
{
    uint32_t start = Timing_GetMilliseconds();
    while (Timing_GetMilliseconds() - start < ms)
    {
        (void)Motion_Update();
        runIdleHook();
    }
}

/* Drive straight forward by about mm millimeters, holding the heading with the gyro.
 * If a wall is seen ahead (head must look forward, distance filter must be fresh), the distance to it is the
 * reference: the car stops when it got mm closer. Otherwise the time is used (DRIVE_MM_PER_SECOND).
 * Blocking. Needs Motion_Init(), Distance_Start() and Timing_Init() before. */
void Car_DriveForward(uint16_t mm)
{
    uint16_t front_mm = 0;
    bool use_sensor = Distance_GetMm(&front_mm) && front_mm <= DRIVE_SENSOR_MAX_MM
                      && front_mm >= mm + DRIVE_SENSOR_MARGIN_MM;
    uint16_t stop_mm = use_sensor ? (uint16_t)(front_mm - mm) : 0;
    uint32_t duration_ms = (uint32_t)mm * 1000u / DRIVE_MM_PER_SECOND;
    int16_t heading = Motion_GetYaw();
    uint32_t start = Timing_GetMilliseconds();

    while (1)
    {
        runIdleHook();
        uint32_t elapsed = Timing_GetMilliseconds() - start;
        if (elapsed > DRIVE_TIMEOUT_MS || (!use_sensor && elapsed >= duration_ms))
        {
            break;
        }
        if (!Motion_Update())
        {
            continue;   // no new gyro sample yet
        }
        uint16_t now_mm;
        if (use_sensor && Distance_GetMm(&now_mm) && now_mm <= stop_mm)
        {
            break;
        }
        // error > 0 means the car points too far right, so the left wheels slow down and the right ones speed up
        int correction = Motion_GetYawError(heading) * DRIVE_HEADING_GAIN;
        Motor_Move(DRIVE_SPEED - correction, DRIVE_SPEED + correction,
                   DRIVE_SPEED - correction, DRIVE_SPEED + correction);   // M1, M3 left; M2, M4 right
    }
    Motor_Move(0, 0, 0, 0);
}
