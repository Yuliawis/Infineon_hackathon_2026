#ifndef CAR_H
#define CAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
// #include <PCA9685.h>

#define MOTOR_1_DIRECTION -1  //If the direction is reversed, change 1 to -1
#define MOTOR_2_DIRECTION -1  //If the direction is reversed, change 1 to -1
#define MOTOR_3_DIRECTION -1  //If the direction is reversed, change 1 to -1
#define MOTOR_4_DIRECTION -1  //If the direction is reversed, change 1 to -1

void PCA9685_Close_Com_Address(void);  //Close the PCA9685 public address

///////////////////// MOTORS & SERVO API //////////////////////////////////////
void Motor_Init(void);                                                    //servo initialization
void Motor_Move(int m1_speed, int m2_speed, int m3_speed, int m4_speed);  //A function to control the car motor

///////////////////// MOTION HELPERS //////////////////////////////////////////
// Take the current yaw as the commanded heading. Call with the car still and aligned to the walls.
void Car_ResetHeading(void);

// Absolute heading (gyro yaw, degrees) that turns and drives aim at.
int16_t Car_GetHeading(void);

// Blocking in-place turn using the gyro: positive = left, negative = right.
// Relative to the commanded heading (not the current yaw), so turn errors do not add up.
void Car_TurnBy(int16_t delta_deg);

// Blocking straight drive by about mm millimeters, heading held by the gyro, front distance used if available.
// grid_mm > 0: walls are on a grid of grid_mm cells, with a wall ahead the car stops in the nearest cell
// center, so drive errors do not add up. 0 = just mm.
// No wall seen ahead (no echo or too far to trust): the car drives blind for blind_ms, the time that mm takes.
// Returns false if a wall ahead was too close for mm: then the car stopped short of it (or did not move).
bool Car_DriveForward(uint16_t mm, uint16_t grid_mm, uint32_t blind_ms);

// Wait while still reading the gyro (use instead of Cy_SysLib_Delay while moving).
void Car_Wait(uint32_t ms);

// Function called on every pass of the blocking loops (Car_TurnBy, Car_DriveForward, Car_Wait).
// Keep it short and non-blocking, it runs inside the motor control loops. NULL = none.
void Car_SetIdleHook(void (*hook)(void));

// How Car_DriveForward() is about to drive, reported once at its start (e.g. for the status LEDs)
typedef enum {
    CAR_DRIVE_SENSOR,  // to the cell center, measured from the wall ahead
    CAR_DRIVE_TIMER,   // no wall ahead within range: blind, on time
    CAR_DRIVE_SHORT    // wall too close: only up to the margin, or not at all
} Car_DriveMode;
void Car_SetDriveModeHook(void (*hook)(Car_DriveMode mode));

///////////////////// SOUND API ///////////////////////////////////////////////
void Sound_Init(void);
void Sound_WriteTone(uint32_t freq);
void Sound_Play(uint32_t freq, uint32_t duration);


///////////////////// TIMING API //////////////////////////////////////////////
void Timing_Init(void);
uint32_t Timing_GetMilliseconds(void);

#ifdef __cplusplus
}
#endif

#endif /* CAR_H */
