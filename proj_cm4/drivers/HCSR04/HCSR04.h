#ifndef HCSR04_H
#define HCSR04_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <stdbool.h>

/* HC-SR04 ultrasonic sensor, low level driver: measures the length of the Echo pulse.
 * Millimeters are made by modules/distance.
 *
 * Trig P12.6 is the output of TCPWM0 counter 7 (PWM), so the 10 us pulse is made by hardware.
 * Echo P12.7 has no TCPWM input on this chip, so it is a GPIO interrupt on both edges, and the pulse width
 * is the difference of the counter value between the edges. The driver sets the measurement period to 50 ms
 * during initialization; configure a base period of at least 40 ms.
 *
 * DEVICE CONFIGURATOR (design.modus), the names must match:
 * 1. Peripherals > PWM (TCPWM), block tcpwm[0].cnt[7]:
 *      Name (alias) PWM_Trig, PWM Mode: PWM, Clock Prescaler: Divide by 1, PWM Alignment: Left Aligned,
 *      Run Mode: Continuous, Period: 59999 (60 ms), Compare: 10 (10 us),
 *      Interrupt Source: Overflow & Underflow, Invert PWM Output and PWM_n Output: off,
 *      Clock Signal: an 8 bit divider with a whole number of MHz (peri_0_div_8_1 = 1 MHz, then Period and
 *      Compare are in microseconds; with 2 MHz double them), PWM (line) output: pin P12[6].
 * 2. Pin P12[6]: alias ULTRA_TRIG, strong drive, connected to the PWM line above.
 * 3. Pin P12[7]: alias ULTRA_ECHO, Digital High-Z with input buffer, Interrupt trigger: Both edges. */

/* Needs cybsp_init() before. Returns false when the configuration is wrong: the clock is not a whole number
   * of MHz, the Trig pulse is shorter than 10 us or the period is shorter than 40 ms. */
bool HCSR04_Init(void);

void HCSR04_Start(void);
void HCSR04_Stop(void);

/* Two ways to get the results. Both can be used, the measurements are stored in any case.
   * 1. HCSR04_getEcho(): length of the last Echo pulse in microseconds, false when there was no echo in the
   *    last period. measurement_number counts finished measurements since HCSR04_Start(), with or without an
   *    echo, to tell a new one from an old one. Both are read in one step, either pointer may be NULL.
   * 2. A callback, called from the interrupt after every finished measurement (has_echo is false when the
   *    sound did not come back in the period). Keep it short. Set it before HCSR04_Start(), NULL removes it.
   * When nothing reflects the sound the sensor holds Echo for about 38 ms, this is not rejected here. */
bool HCSR04_getEcho(uint32_t* echo_us, uint32_t* measurement_number);

typedef void (*HCSR04_MeasurementCallback)(bool has_echo, uint32_t echo_us);
void HCSR04_setMeasurementCallback(HCSR04_MeasurementCallback callback);

#ifdef __cplusplus
}
#endif

#endif
