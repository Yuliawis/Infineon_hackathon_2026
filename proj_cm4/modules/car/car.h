#ifndef CAR_H
#define CAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
// #include <PCA9685.h>

#define MOTOR_1_DIRECTION     -1 //If the direction is reversed, change 1 to -1
#define MOTOR_2_DIRECTION     -1 //If the direction is reversed, change 1 to -1
#define MOTOR_3_DIRECTION     -1 //If the direction is reversed, change 1 to -1
#define MOTOR_4_DIRECTION     -1 //If the direction is reversed, change 1 to -1

void PCA9685_Close_Com_Address(void);  //Close the PCA9685 public address

///////////////////// MOTORS & SERVO API //////////////////////////////////////
void Motor_Init(void);                //servo initialization
void Motor_Move(int m1_speed, int m2_speed, int m3_speed, int m4_speed);//A function to control the car motor

///////////////////// MOTION HELPERS //////////////////////////////////////////
// Blocking in-place turn using the gyro: positive = left, negative = right.
void Car_TurnBy(int16_t delta_deg);
// Blocking straight drive by about mm millimeters, heading held by the gyro, front distance used if available.
void Car_DriveForward(uint16_t mm);

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
