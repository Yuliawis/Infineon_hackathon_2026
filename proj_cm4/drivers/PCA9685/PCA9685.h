#ifndef PCA9685_H
#define PCA9685_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <stddef.h>

#define TIME_MIN                (0u)
#define TIME_MAX                (4096u)
#define PWM_PERIOD_MAX_US       (39525u)
#define MICROSECONDS_PER_SECOND (1000000u)
#define PCA9685_FREQUENCY_MIN   (MICROSECONDS_PER_SECOND / PWM_PERIOD_MAX_US)
#define PCA9685_FREQUENCY_MAX   (MICROSECONDS_PER_SECOND / PWM_PERIOD_MIN_US)
#define PCA9685_PULSE_WIDTH_MIN (TIME_MIN)
#define PCA9685_PULSE_WIDTH_MAX (TIME_MAX)
#define CHANNELS_PER_DEVICE     (16u)

typedef uint16_t Channel;
typedef uint16_t Frequency;
typedef double Percent;
typedef uint16_t Time;
typedef uint16_t Duration;
typedef uint16_t DurationMicroseconds;

union Mode1Register {
    struct {
        uint8_t allcall : 1;
        uint8_t sub3 : 1;
        uint8_t sub2 : 1;
        uint8_t sub1 : 1;
        uint8_t sleep : 1;
        uint8_t ai : 1;
        uint8_t extclk : 1;
        uint8_t restart : 1;
    } fields;
    uint8_t data;
};


void PCA9685_Init(void);
void PCA9685_setToFrequency(Frequency frequency);
void PCA9685_setToServoFrequency();

Duration PCA9685_getPulseWidthMin();
Duration PCA9685_getPulseWidthMax();

void PCA9685_setChannelPulseWidth(Channel channel, Duration pulse_width);
void PCA9685_setChannelOnAndOffTime(Channel channel, Time on_time, Time off_time);

Frequency PCA9685_getDeviceServoFrequency();

union Mode1Register PCA9685_readMode1Register();

void PCA9685_sleep();
void PCA9685_wake();

void PCA9685_setPrescale(uint8_t prescale);
uint8_t PCA9685_frequencyToPrescale(Frequency frequency);

/* Converts a pulse length to ticks for the prescale that was set last. Needs PCA9685_setToFrequency() first. */
Duration PCA9685_microsecondsToPulseWidth(DurationMicroseconds microseconds);

void PCA9685_pulseWidthAndPhaseShiftToOnTimeAndOffTime(Duration pulse_width, Time* on_time, Time* off_time);

#ifdef __cplusplus
}
#endif

#endif
