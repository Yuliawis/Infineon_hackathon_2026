#include "PCA9685.h"
#include "utils.h"
#include "i2c_common.h"
#include "cybsp.h"

#define PCA9685_ADDRESS        (0x5Fu)
#define MODE1_REGISTER_ADDRESS (0x00)

const static uint8_t LED0_ON_L_REGISTER_ADDRESS = 0x06;
const static uint8_t LED_REGISTERS_SIZE = 4;

const static uint8_t PRE_SCALE_REGISTER_ADDRESS = 0xFE;
const static uint8_t PRE_SCALE_MIN = 0x03;
const static uint8_t PRE_SCALE_MAX = 0xFF;

#define PWM_PERIOD_MIN_US (617u)

#define OSCILLATOR_MHZ             (25u)
#define PRE_SCALE_POWER_ON_DEFAULT (0x1Eu)

/* Remembered to convert microseconds to ticks. The chip is not read back. */
static uint8_t current_prescale = PRE_SCALE_POWER_ON_DEFAULT;

#define SLEEP                  (1u)
#define WAKE                   (0u)
#define AUTO_INCREMENT_ENABLED (1u)
#define RESTART_ENABLED        (1u)
#define RESTART_CLEAR          (1u)

const static Frequency SERVO_FREQUENCY = 50;

static void write8(uint8_t register_address, uint8_t data) {
    cy_stc_scb_i2c_master_xfer_config_t transaction;
    static uint8_t dataPacket[2];

    /* Wait for previous transaction completion */
    while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(I2C_Main_HW, &I2C_Main_context))) {
    }

    dataPacket[0] = register_address;
    dataPacket[1] = data;

    transaction.slaveAddress = PCA9685_ADDRESS;
    transaction.buffer = dataPacket;
    transaction.bufferSize = 2;
    transaction.xferPending = false;
    Cy_SCB_I2C_MasterWrite(I2C_Main_HW, &transaction, &I2C_Main_context);
}

static void write32(uint8_t register_address, uint32_t data) {
    cy_stc_scb_i2c_master_xfer_config_t transaction;
    static uint8_t dataPacket[5];

    /* Wait for previous transaction completion */
    while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(I2C_Main_HW, &I2C_Main_context))) {
    }

    dataPacket[0] = register_address;
    for (int byte_n = 0; byte_n < 4; ++byte_n) {
        dataPacket[byte_n + 1] = (data >> (8u * byte_n)) & 0xFFu;
    }

    transaction.slaveAddress = PCA9685_ADDRESS;
    transaction.buffer = dataPacket;
    transaction.bufferSize = 5;
    transaction.xferPending = false;
    Cy_SCB_I2C_MasterWrite(I2C_Main_HW, &transaction, &I2C_Main_context);
}

static uint8_t read8(uint8_t register_address) {
    cy_stc_scb_i2c_master_xfer_config_t transaction;
    static uint8_t dataPacket[1];

    /* Wait for previous transaction completion */
    while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(I2C_Main_HW, &I2C_Main_context))) {
    }

    dataPacket[0] = register_address;

    transaction.slaveAddress = PCA9685_ADDRESS;
    transaction.buffer = dataPacket;
    transaction.bufferSize = 1;
    transaction.xferPending = true;

    Cy_SCB_I2C_MasterWrite(I2C_Main_HW, &transaction, &I2C_Main_context);

    /* Wait for previous transaction completion */
    while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(I2C_Main_HW, &I2C_Main_context))) {
    }

    transaction.bufferSize = 1;
    transaction.xferPending = false;
    Cy_SCB_I2C_MasterRead(I2C_Main_HW, &transaction, &I2C_Main_context);

    return dataPacket[0];
}

static void sleep() {
    union Mode1Register mode1_register;
    mode1_register.data = read8(MODE1_REGISTER_ADDRESS);
    mode1_register.fields.sleep = SLEEP;
    write8(MODE1_REGISTER_ADDRESS, mode1_register.data);
}

static void wake() {
    union Mode1Register mode1_register;
    mode1_register.data = read8(MODE1_REGISTER_ADDRESS);
    mode1_register.fields.sleep = WAKE;
    mode1_register.fields.ai = AUTO_INCREMENT_ENABLED;
    write8(MODE1_REGISTER_ADDRESS, mode1_register.data);
    if (mode1_register.fields.restart == RESTART_ENABLED) {
        Cy_SysLib_Delay(1);
        mode1_register.fields.restart = RESTART_CLEAR;
        write8(MODE1_REGISTER_ADDRESS, mode1_register.data);
    }
}

void PCA9685_Init(void) {
    /* write 0x00, 0x00 to PCA9685 */
    uint8_t dataToSend[2u] = {0x00, 0x00};
    cy_stc_scb_i2c_master_xfer_config_t transaction;

    transaction.slaveAddress = PCA9685_ADDRESS;
    transaction.buffer = dataToSend;
    transaction.bufferSize = 2u;
    transaction.xferPending = false;
    Cy_SCB_I2C_MasterWrite(I2C_Main_HW, &transaction, &I2C_Main_context);
}

void PCA9685_setChannelPulseWidth(Channel channel, Duration pulse_width) {
    Time on_time;
    Time off_time;
    PCA9685_pulseWidthAndPhaseShiftToOnTimeAndOffTime(pulse_width, &on_time, &off_time);
    PCA9685_setChannelOnAndOffTime(channel, on_time, off_time);
}

void PCA9685_setChannelOnAndOffTime(Channel channel, Time on_time, Time off_time) {
    if (channel >= CHANNELS_PER_DEVICE) {
        return;
    }
    uint8_t register_address = LED0_ON_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * channel;
    uint32_t data = (off_time << 16u) | on_time;
    write32(register_address, data);
}

void PCA9685_setToFrequency(Frequency frequency) {
    uint8_t prescale = PCA9685_frequencyToPrescale(frequency);
    PCA9685_setPrescale(prescale);
}

void PCA9685_setToServoFrequency() {
    PCA9685_setToFrequency(SERVO_FREQUENCY);
}

void PCA9685_setPrescale(uint8_t prescale) {
    sleep();
    write8(PRE_SCALE_REGISTER_ADDRESS, prescale);
    wake();
    current_prescale = prescale;
}

Duration PCA9685_microsecondsToPulseWidth(DurationMicroseconds microseconds) {
    /* One tick lasts (prescale + 1) / 25 MHz, so it does not depend on the requested frequency being exact */
    uint32_t ticks_divider = (uint32_t) current_prescale + 1u;
    uint32_t ticks = ((uint32_t) microseconds * OSCILLATOR_MHZ + ticks_divider / 2u) / ticks_divider;
    return (Duration) (ticks > TIME_MAX ? TIME_MAX : ticks);
}

uint8_t PCA9685_frequencyToPrescale(Frequency frequency) {
    DurationMicroseconds period_us = MICROSECONDS_PER_SECOND / frequency;
    period_us = constrain_int(period_us, PWM_PERIOD_MIN_US, PWM_PERIOD_MAX_US);
    uint8_t prescale = map_int(period_us, PWM_PERIOD_MIN_US, PWM_PERIOD_MAX_US, PRE_SCALE_MIN, PRE_SCALE_MAX);
    return prescale;
}

void PCA9685_pulseWidthAndPhaseShiftToOnTimeAndOffTime(Duration pulse_width, Time* on_time, Time* off_time) {
    if (pulse_width == TIME_MIN) {
        *on_time = TIME_MIN;
        *off_time = TIME_MAX;
        return;
    }
    if (pulse_width >= TIME_MAX) {
        *on_time = TIME_MAX;
        *off_time = TIME_MIN;
        return;
    }
    *on_time = 0;
    *off_time = (*on_time + pulse_width) % TIME_MAX;
}
