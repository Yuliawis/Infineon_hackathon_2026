#ifndef HEAD_H
#define HEAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Head with the ultrasonic sensor: one servo turns it left/right (pan), the other up/down (tilt).
// Both servos are on the PCA9685. Channels 8..15 are used by the motors.
#define HEAD_PAN_CHANNEL  (0u)
#define HEAD_TILT_CHANNEL (1u)

#define HEAD_PAN_DIRECTION  1  //If the head turns the wrong way, change 1 to -1
#define HEAD_TILT_DIRECTION 1  //If the head tilts the wrong way, change 1 to -1

// Zero position correction for a crooked mount (whole degrees)
#define HEAD_PAN_TRIM_DEG  (0)
#define HEAD_TILT_TRIM_DEG (0)

// Limits in degrees. Freenove limits the tilt servo to 90..150 of its own scale, which is 0..60 here.
#define HEAD_PAN_MIN_DEG  (-90)
#define HEAD_PAN_MAX_DEG  (90)
#define HEAD_TILT_MIN_DEG (-20)
#define HEAD_TILT_MAX_DEG (10)

// Needs I2C_Main_Start() and Motor_Init() before. Moves the head to the center.
bool Head_Init(void);

// Whole degrees, cut to the limits above. 0 is straight ahead / level.
// Pan is positive to the left (like the yaw in motion.h), tilt is positive up.
void Head_SetPan(int8_t pan_degrees);
void Head_SetTilt(int8_t tilt_degrees);
void Head_Look(int8_t pan_degrees, int8_t tilt_degrees);
void Head_PanBy(int8_t delta_degrees);
void Head_TiltBy(int8_t delta_degrees);
void Head_Center(void);

// The servos go limp. Any Head_Set*/Head_Look call holds them again.
void Head_Release(void);

// Last commanded angles, the servos do not report their position
int8_t Head_GetPan(void);
int8_t Head_GetTilt(void);

#ifdef __cplusplus
}
#endif

#endif /* HEAD_H */
