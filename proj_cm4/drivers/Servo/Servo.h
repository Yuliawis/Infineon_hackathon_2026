#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <stdbool.h>

/* Typical hobby servo: 500..2500 us for 180 degrees */
#define SERVO_DEFAULT_MIN_PULSE_US  (500u)
#define SERVO_DEFAULT_MAX_PULSE_US  (2500u)
#define SERVO_DEFAULT_RANGE_DEGREES (180)

typedef struct {
    uint8_t channel;       /* PCA9685 channel */
    uint16_t min_pulse_us; /* pulse at 0 degrees */
    uint16_t max_pulse_us; /* pulse at range_degrees */
    uint8_t range_degrees;
    uint8_t angle; /* last commanded angle */
    bool released; /* no pulses are sent */
} Servo;

/* The PCA9685 frequency must be set before (Motor_Init()). Does not move the servo.
   * Returns false on invalid arguments. */
bool Servo_Init(Servo* servo, uint8_t channel, uint16_t min_pulse_us, uint16_t max_pulse_us, uint8_t range_degrees);

/* Values outside of the range are cut to it */
void Servo_SetAngle(Servo* servo, uint8_t angle);
void Servo_SetPulseMicroseconds(Servo* servo, uint16_t pulse_us);
/* Stops the pulses, the servo goes limp */
void Servo_Release(Servo* servo);

uint8_t Servo_getAngle(const Servo* servo);
uint16_t Servo_angleToPulseMicroseconds(const Servo* servo, uint8_t angle);

#ifdef __cplusplus
}
#endif

#endif
