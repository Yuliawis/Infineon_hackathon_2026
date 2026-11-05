#include "head.h"
#include "Servo.h"
#include "utils.h"

#define SERVO_CENTER_DEG (SERVO_DEFAULT_RANGE_DEGREES / 2)

static Servo pan_servo;
static Servo tilt_servo;
static int8_t pan = 0;
static int8_t tilt = 0;

/* Computed in int and cut to the servo range, so that a large trim or angle cannot wrap around */
static uint8_t toServoAngle(int servo_angle) {
    return (uint8_t) constrain_int(servo_angle, 0, SERVO_DEFAULT_RANGE_DEGREES);
}

static uint8_t panToServoAngle(int8_t pan_degrees) {
    return toServoAngle(SERVO_CENTER_DEG + HEAD_PAN_TRIM_DEG + HEAD_PAN_DIRECTION * pan_degrees);
}

static uint8_t tiltToServoAngle(int8_t tilt_degrees) {
    return toServoAngle(SERVO_CENTER_DEG + HEAD_TILT_TRIM_DEG + HEAD_TILT_DIRECTION * tilt_degrees);
}

bool Head_Init(void) {
    if (!Servo_Init(&pan_servo, HEAD_PAN_CHANNEL, SERVO_DEFAULT_MIN_PULSE_US, SERVO_DEFAULT_MAX_PULSE_US,
                    SERVO_DEFAULT_RANGE_DEGREES) ||
        !Servo_Init(&tilt_servo, HEAD_TILT_CHANNEL, SERVO_DEFAULT_MIN_PULSE_US, SERVO_DEFAULT_MAX_PULSE_US,
                    SERVO_DEFAULT_RANGE_DEGREES) ||
        HEAD_PAN_CHANNEL == HEAD_TILT_CHANNEL) {
        return false;
    }

    Head_Center();
    return true;
}

void Head_SetPan(int8_t pan_degrees) {
    pan = (int8_t) constrain_int(pan_degrees, HEAD_PAN_MIN_DEG, HEAD_PAN_MAX_DEG);
    Servo_SetAngle(&pan_servo, panToServoAngle(pan));
}

void Head_SetTilt(int8_t tilt_degrees) {
    tilt = (int8_t) constrain_int(tilt_degrees, HEAD_TILT_MIN_DEG, HEAD_TILT_MAX_DEG);
    Servo_SetAngle(&tilt_servo, tiltToServoAngle(tilt));
}

void Head_Look(int8_t pan_degrees, int8_t tilt_degrees) {
    Head_SetPan(pan_degrees);
    Head_SetTilt(tilt_degrees);
}

/* The sum is made in int and cut to int8_t range first, so it cannot wrap before the head limits apply */
void Head_PanBy(int8_t delta_degrees) {
    Head_SetPan((int8_t) constrain_int(pan + delta_degrees, INT8_MIN, INT8_MAX));
}

void Head_TiltBy(int8_t delta_degrees) {
    Head_SetTilt((int8_t) constrain_int(tilt + delta_degrees, INT8_MIN, INT8_MAX));
}

void Head_Center(void) {
    Head_Look(0, 0);
}

void Head_Release(void) {
    Servo_Release(&pan_servo);
    Servo_Release(&tilt_servo);
}

int8_t Head_GetPan(void) {
    return pan;
}

int8_t Head_GetTilt(void) {
    return tilt;
}
