#include "Servo.h"
#include "PCA9685.h"
#include "utils.h"

bool Servo_Init(Servo *servo, uint8_t channel, uint16_t min_pulse_us, uint16_t max_pulse_us, uint8_t range_degrees)
{
  if (servo == NULL
    || channel >= CHANNELS_PER_DEVICE
    || min_pulse_us == 0
    || min_pulse_us >= max_pulse_us
    || range_degrees == 0)
  {
    return false;
  }

  servo->channel = channel;
  servo->min_pulse_us = min_pulse_us;
  servo->max_pulse_us = max_pulse_us;
  servo->range_degrees = range_degrees;
  servo->angle = range_degrees / 2;
  servo->released = true;
  return true;
}

uint16_t Servo_angleToPulseMicroseconds(const Servo *servo, uint8_t angle)
{
  int32_t angle_limited = constrain_int(angle, 0, servo->range_degrees);
  int32_t pulse_span_us = (int32_t)servo->max_pulse_us - servo->min_pulse_us;
  int32_t offset_us = (angle_limited * pulse_span_us + servo->range_degrees / 2) / servo->range_degrees;
  return (uint16_t)(servo->min_pulse_us + offset_us);
}

void Servo_SetPulseMicroseconds(Servo *servo, uint16_t pulse_us)
{
  pulse_us = (uint16_t)constrain_int(pulse_us, servo->min_pulse_us, servo->max_pulse_us);
  PCA9685_setChannelPulseWidth(servo->channel, PCA9685_microsecondsToPulseWidth(pulse_us));
  servo->released = false;
}

void Servo_SetAngle(Servo *servo, uint8_t angle)
{
  servo->angle = (uint8_t)constrain_int(angle, 0, servo->range_degrees);
  Servo_SetPulseMicroseconds(servo, Servo_angleToPulseMicroseconds(servo, servo->angle));
}

void Servo_Release(Servo *servo)
{
  PCA9685_setChannelPulseWidth(servo->channel, TIME_MIN);
  servo->released = true;
}

uint8_t Servo_getAngle(const Servo *servo)
{
  return servo->angle;
}
