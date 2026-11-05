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
#define TURN_SPEED          (1200)
#define TURN_TOLERANCE_DEG  (1)
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


/* Turn the car in place by delta_deg degrees (positive = left, negative = right).
 * Blocking: returns when the target heading is reached or after TURN_TIMEOUT_MS.
 * Needs Motion_Init() and Timing_Init() to be called before. */
void Car_TurnBy(int16_t delta_deg)
{
    int16_t target = Motion_GetYaw() + delta_deg;   // Motion_GetYawError() wraps the angle itself
    uint32_t start = Timing_GetMilliseconds();

    while (abs(Motion_GetYawError(target)) > TURN_TOLERANCE_DEG)
    {
        if (Timing_GetMilliseconds() - start > TURN_TIMEOUT_MS)
        {
            break;   // safety: the IMU is stuck or the car does not turn
        }
        if (!Motion_Update())
        {
            continue;   // no new sensor sample yet
        }
        // error > 0 means turn left: left wheels backward, right wheels forward
        int s = (Motion_GetYawError(target) > 0) ? TURN_SPEED : -TURN_SPEED;
        Motor_Move(-s, -s, s, s);   // check on the real car which motors are left/right
    }
    Motor_Move(0, 0, 0, 0);
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
