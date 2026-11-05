#ifndef MOTION_H
#define MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Assumes the MPU6050 is mounted flat on the car: Z axis up, X axis forward.
// If it is mounted differently, change the signs here (or swap axes in motion.c).
#define MOTION_YAW_DIRECTION 1  //If the yaw sign is reversed, change 1 to -1

// Minimal period between two sensor readings. Motion_Update() does nothing if called more often.
#define MOTION_UPDATE_PERIOD_MS (10u)

// Yaw rate (millidegrees per second) above which the car is considered turning.
#define MOTION_TURN_RATE_THRESHOLD_MDPS (5000)

typedef enum {
    MOTION_TURN_NONE = 0,
    MOTION_TURN_LEFT,  // counterclockwise seen from above
    MOTION_TURN_RIGHT  // clockwise seen from above
} Motion_TurnState;

///////////////////// INIT ////////////////////////////////////////////////////
// Init MPU6050 (+-500 dps, +-4 g) and calibrate gyro bias.
// Needs I2C_Main_Start() and Timing_Init() to be called before.
// Keep the car still during the call. Returns false if the sensor does not answer.
bool Motion_Init(void);

// Measure gyro bias again. Keep the car still during the call.
bool Motion_CalibrateGyro(void);

///////////////////// UPDATE //////////////////////////////////////////////////
// Read the sensor, integrate yaw and filter pitch/roll. Call it as often as possible from the main loop.
// Returns true if a new sample was processed.
bool Motion_Update(void);

///////////////////// GYROSCOPE API ///////////////////////////////////////////
// Everything is integer. The main API is in whole degrees and degrees per second.
// The Mdeg/Mdps variants give the same values in thousandths for finer control.

// Angular rates in degrees per second, bias already subtracted.
// Yaw rate is positive when the car turns left (counterclockwise).
int16_t Motion_GetYawRate(void);
int16_t Motion_GetPitchRate(void);
int16_t Motion_GetRollRate(void);
int32_t Motion_GetYawRateMdps(void);
int32_t Motion_GetPitchRateMdps(void);
int32_t Motion_GetRollRateMdps(void);

// Heading in degrees in range (-180, 180]. Positive is left. Drifts slowly with time.
int16_t Motion_GetYaw(void);
int32_t Motion_GetYawMdeg(void);
void Motion_ResetYaw(void);

// Shortest signed angle in degrees (-180, 180] to turn left from the current yaw to the target yaw.
int16_t Motion_GetYawError(int16_t target_yaw);
int32_t Motion_GetYawErrorMdeg(int32_t target_yaw_mdeg);

Motion_TurnState Motion_GetTurnState(void);
bool Motion_IsTurning(void);

///////////////////// ACCELEROMETER API ///////////////////////////////////////
// Acceleration in milli-g (1000 = 1 g), because whole g would be too coarse.
int16_t Motion_GetAccelXMg(void);
int16_t Motion_GetAccelYMg(void);
int16_t Motion_GetAccelZMg(void);

// Tilt in degrees, accelerometer and gyroscope fused (complementary filter).
// Pitch is positive when the nose goes up, roll is positive when the left side goes up.
int16_t Motion_GetPitch(void);
int16_t Motion_GetRoll(void);
int32_t Motion_GetPitchMdeg(void);
int32_t Motion_GetRollMdeg(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_H */
