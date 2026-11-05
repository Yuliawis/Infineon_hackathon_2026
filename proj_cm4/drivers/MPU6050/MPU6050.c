#include "MPU6050.h"
#include "i2c_common.h"
#include "cybsp.h"

/* 0x68 when AD0 is low, 0x69 when AD0 is high */
#define MPU6050_ADDRESS (0x68u)

#define SMPLRT_DIV_REGISTER_ADDRESS   (0x19u)
#define CONFIG_REGISTER_ADDRESS       (0x1Au)
#define GYRO_CONFIG_REGISTER_ADDRESS  (0x1Bu)
#define ACCEL_CONFIG_REGISTER_ADDRESS (0x1Cu)
#define ACCEL_XOUT_H_REGISTER_ADDRESS (0x3Bu)
#define TEMP_OUT_H_REGISTER_ADDRESS   (0x41u)
#define GYRO_XOUT_H_REGISTER_ADDRESS  (0x43u)
#define PWR_MGMT_1_REGISTER_ADDRESS   (0x6Bu)
#define WHO_AM_I_REGISTER_ADDRESS     (0x75u)

#define RANGE_SELECT_SHIFT (3u)
#define RANGE_SELECT_MASK  (0x18u)

#define SLEEP                   (1u)
#define WAKE                    (0u)
#define DEVICE_RESET            (1u)
#define CLOCK_SOURCE_PLL_GYRO_X (1u)

#define ALL_REGISTERS_SIZE         (14u)
#define VECTOR_REGISTERS_SIZE      (6u)
#define TEMPERATURE_REGISTERS_SIZE (2u)

#define TRANSFER_TIMEOUT_US   (100000u)
#define TRANSFER_POLL_STEP_US (10u)
#define RESET_DELAY_MS        (100u)
#define WAKE_DELAY_MS         (10u)

#define DEFAULT_SAMPLE_RATE_DIVIDER (9u)

/* Temperature in degrees Celsius = raw / 340 + 36.53 */
#define TEMPERATURE_SENSITIVITY          (340)
#define TEMPERATURE_OFFSET_CENTI_CELSIUS (3653)

/* Full scale of the 16 bit result: 2 g / 250 dps for range 0, doubling with each range step */
#define ACCEL_FULL_SCALE_MG_RANGE_0  (2000)
#define GYRO_FULL_SCALE_MDPS_RANGE_0 (250000)
#define RAW_FULL_SCALE_SHIFT         (15u)
#define ROUNDING_HALF                (1 << (RAW_FULL_SCALE_SHIFT - 1))
static MPU6050_AccelRange accel_range = MPU6050_ACCEL_RANGE_2G;
static MPU6050_GyroRange gyro_range = MPU6050_GYRO_RANGE_250_DPS;

/* Wait until the bus master is idle. Returns false on timeout. Errors of the previous transfer are not checked. */
static bool waitIdle(void) {
    uint32_t waited_us = 0;

    while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(I2C_Main_HW, &I2C_Main_context))) {
        if (waited_us >= TRANSFER_TIMEOUT_US) {
            Cy_SCB_I2C_MasterAbortWrite(I2C_Main_HW, &I2C_Main_context);
            return false;
        }
        Cy_SysLib_DelayUs(TRANSFER_POLL_STEP_US);
        waited_us += TRANSFER_POLL_STEP_US;
    }

    return true;
}

/* Wait for the transaction started by this driver. Returns false on timeout or bus error. */
static bool waitTransferDone(void) {
    return waitIdle() && 0UL == (CY_SCB_I2C_MASTER_ERR & Cy_SCB_I2C_MasterGetStatus(I2C_Main_HW, &I2C_Main_context));
}

static bool write8(uint8_t register_address, uint8_t data) {
    cy_stc_scb_i2c_master_xfer_config_t transaction;
    static uint8_t dataPacket[2];

    if (!waitIdle()) {
        return false;
    }

    dataPacket[0] = register_address;
    dataPacket[1] = data;

    transaction.slaveAddress = MPU6050_ADDRESS;
    transaction.buffer = dataPacket;
    transaction.bufferSize = 2;
    transaction.xferPending = false;
    if (CY_SCB_I2C_SUCCESS != Cy_SCB_I2C_MasterWrite(I2C_Main_HW, &transaction, &I2C_Main_context)) {
        return false;
    }

    return waitTransferDone();
}

static bool readRegisters(uint8_t register_address, uint8_t* data, uint8_t size) {
    cy_stc_scb_i2c_master_xfer_config_t transaction;
    static uint8_t addressPacket[1];

    if (!waitIdle()) {
        return false;
    }

    addressPacket[0] = register_address;

    /* Repeated start between the register address write and the data read */
    transaction.slaveAddress = MPU6050_ADDRESS;
    transaction.buffer = addressPacket;
    transaction.bufferSize = 1;
    transaction.xferPending = true;
    if (CY_SCB_I2C_SUCCESS != Cy_SCB_I2C_MasterWrite(I2C_Main_HW, &transaction, &I2C_Main_context)) {
        return false;
    }

    if (!waitTransferDone()) {
        return false;
    }

    transaction.buffer = data;
    transaction.bufferSize = size;
    transaction.xferPending = false;
    if (CY_SCB_I2C_SUCCESS != Cy_SCB_I2C_MasterRead(I2C_Main_HW, &transaction, &I2C_Main_context)) {
        return false;
    }

    return waitTransferDone();
}

static bool read8(uint8_t register_address, uint8_t* data) {
    return readRegisters(register_address, data, 1);
}

static int16_t toInt16(const uint8_t* buffer) {
    return (int16_t) (((uint16_t) buffer[0] << 8u) | buffer[1]);
}

static void bufferToVector(const uint8_t* buffer, MPU6050_Vector3Raw* vector) {
    vector->x = toInt16(&buffer[0]);
    vector->y = toInt16(&buffer[2]);
    vector->z = toInt16(&buffer[4]);
}

static bool updateRange(uint8_t register_address, uint8_t range) {
    uint8_t config;
    if (!read8(register_address, &config)) {
        return false;
    }
    config = (config & ~RANGE_SELECT_MASK) | (range << RANGE_SELECT_SHIFT);
    return write8(register_address, config);
}

bool MPU6050_Init(void) {
    if (!MPU6050_isConnected()) {
        return false;
    }

    union PwrMgmt1Register pwr_mgmt_1_register;
    pwr_mgmt_1_register.data = 0;
    pwr_mgmt_1_register.fields.device_reset = DEVICE_RESET;
    if (!write8(PWR_MGMT_1_REGISTER_ADDRESS, pwr_mgmt_1_register.data)) {
        return false;
    }
    Cy_SysLib_Delay(RESET_DELAY_MS);

    /* Leave sleep mode and use the gyro PLL as a more stable clock than the internal oscillator */
    pwr_mgmt_1_register.data = 0;
    pwr_mgmt_1_register.fields.sleep = WAKE;
    pwr_mgmt_1_register.fields.clksel = CLOCK_SOURCE_PLL_GYRO_X;
    if (!write8(PWR_MGMT_1_REGISTER_ADDRESS, pwr_mgmt_1_register.data)) {
        return false;
    }
    Cy_SysLib_Delay(WAKE_DELAY_MS);

    return MPU6050_setDlpf(MPU6050_DLPF_44_HZ) && MPU6050_setSampleRateDivider(DEFAULT_SAMPLE_RATE_DIVIDER) &&
           MPU6050_setAccelRange(MPU6050_ACCEL_RANGE_2G) && MPU6050_setGyroRange(MPU6050_GYRO_RANGE_250_DPS);
}

bool MPU6050_isConnected(void) {
    return MPU6050_readWhoAmI() == MPU6050_WHO_AM_I_VALUE;
}

uint8_t MPU6050_readWhoAmI(void) {
    uint8_t who_am_i = 0;
    if (!read8(WHO_AM_I_REGISTER_ADDRESS, &who_am_i)) {
        return 0;
    }
    return who_am_i;
}

void MPU6050_sleep(void) {
    union PwrMgmt1Register pwr_mgmt_1_register;
    if (read8(PWR_MGMT_1_REGISTER_ADDRESS, &pwr_mgmt_1_register.data)) {
        pwr_mgmt_1_register.fields.sleep = SLEEP;
        write8(PWR_MGMT_1_REGISTER_ADDRESS, pwr_mgmt_1_register.data);
    }
}

void MPU6050_wake(void) {
    union PwrMgmt1Register pwr_mgmt_1_register;
    if (read8(PWR_MGMT_1_REGISTER_ADDRESS, &pwr_mgmt_1_register.data)) {
        pwr_mgmt_1_register.fields.sleep = WAKE;
        write8(PWR_MGMT_1_REGISTER_ADDRESS, pwr_mgmt_1_register.data);
    }
}

void MPU6050_reset(void) {
    union PwrMgmt1Register pwr_mgmt_1_register;
    pwr_mgmt_1_register.data = 0;
    pwr_mgmt_1_register.fields.device_reset = DEVICE_RESET;
    write8(PWR_MGMT_1_REGISTER_ADDRESS, pwr_mgmt_1_register.data);
    Cy_SysLib_Delay(RESET_DELAY_MS);
}

bool MPU6050_setAccelRange(MPU6050_AccelRange range) {
    if (range > MPU6050_ACCEL_RANGE_16G || !updateRange(ACCEL_CONFIG_REGISTER_ADDRESS, range)) {
        return false;
    }
    accel_range = range;
    return true;
}

bool MPU6050_setGyroRange(MPU6050_GyroRange range) {
    if (range > MPU6050_GYRO_RANGE_2000_DPS || !updateRange(GYRO_CONFIG_REGISTER_ADDRESS, range)) {
        return false;
    }
    gyro_range = range;
    return true;
}

bool MPU6050_setDlpf(MPU6050_Dlpf dlpf) {
    if (dlpf > MPU6050_DLPF_5_HZ) {
        return false;
    }
    return write8(CONFIG_REGISTER_ADDRESS, dlpf);
}

bool MPU6050_setSampleRateDivider(uint8_t divider) {
    return write8(SMPLRT_DIV_REGISTER_ADDRESS, divider);
}

bool MPU6050_readAccelRaw(MPU6050_Vector3Raw* accel) {
    uint8_t buffer[VECTOR_REGISTERS_SIZE];
    if (!readRegisters(ACCEL_XOUT_H_REGISTER_ADDRESS, buffer, VECTOR_REGISTERS_SIZE)) {
        return false;
    }
    bufferToVector(buffer, accel);
    return true;
}

bool MPU6050_readGyroRaw(MPU6050_Vector3Raw* gyro) {
    uint8_t buffer[VECTOR_REGISTERS_SIZE];
    if (!readRegisters(GYRO_XOUT_H_REGISTER_ADDRESS, buffer, VECTOR_REGISTERS_SIZE)) {
        return false;
    }
    bufferToVector(buffer, gyro);
    return true;
}

bool MPU6050_readTemperatureRaw(int16_t* temperature) {
    uint8_t buffer[TEMPERATURE_REGISTERS_SIZE];
    if (!readRegisters(TEMP_OUT_H_REGISTER_ADDRESS, buffer, TEMPERATURE_REGISTERS_SIZE)) {
        return false;
    }
    *temperature = toInt16(buffer);
    return true;
}

/* Registers 0x3B..0x48 are read in a single burst so all values belong to the same sample */
bool MPU6050_readAllRaw(MPU6050_RawData* data) {
    uint8_t buffer[ALL_REGISTERS_SIZE];
    if (!readRegisters(ACCEL_XOUT_H_REGISTER_ADDRESS, buffer, ALL_REGISTERS_SIZE)) {
        return false;
    }
    bufferToVector(&buffer[0], &data->accel);
    data->temperature = toInt16(&buffer[6]);
    bufferToVector(&buffer[8], &data->gyro);
    return true;
}

bool MPU6050_readAccel(MPU6050_AccelMg* accel) {
    MPU6050_Vector3Raw raw;
    if (!MPU6050_readAccelRaw(&raw)) {
        return false;
    }
    accel->x = MPU6050_accelRawToMg(raw.x);
    accel->y = MPU6050_accelRawToMg(raw.y);
    accel->z = MPU6050_accelRawToMg(raw.z);
    return true;
}

bool MPU6050_readGyro(MPU6050_GyroMdps* gyro) {
    MPU6050_Vector3Raw raw;
    if (!MPU6050_readGyroRaw(&raw)) {
        return false;
    }
    gyro->x = MPU6050_gyroRawToMdps(raw.x);
    gyro->y = MPU6050_gyroRawToMdps(raw.y);
    gyro->z = MPU6050_gyroRawToMdps(raw.z);
    return true;
}

bool MPU6050_readTemperature(int16_t* temperature_centi_celsius) {
    int16_t raw;
    if (!MPU6050_readTemperatureRaw(&raw)) {
        return false;
    }
    *temperature_centi_celsius = MPU6050_temperatureRawToCentiCelsius(raw);
    return true;
}

bool MPU6050_readAll(MPU6050_Data* data) {
    MPU6050_RawData raw;
    if (!MPU6050_readAllRaw(&raw)) {
        return false;
    }
    data->accel.x = MPU6050_accelRawToMg(raw.accel.x);
    data->accel.y = MPU6050_accelRawToMg(raw.accel.y);
    data->accel.z = MPU6050_accelRawToMg(raw.accel.z);
    data->gyro.x = MPU6050_gyroRawToMdps(raw.gyro.x);
    data->gyro.y = MPU6050_gyroRawToMdps(raw.gyro.y);
    data->gyro.z = MPU6050_gyroRawToMdps(raw.gyro.z);
    data->temperature = MPU6050_temperatureRawToCentiCelsius(raw.temperature);
    return true;
}

int16_t MPU6050_accelRawToMg(int16_t raw) {
    int32_t full_scale_mg = ACCEL_FULL_SCALE_MG_RANGE_0 << accel_range;
    return (int16_t) (((int32_t) raw * full_scale_mg + ROUNDING_HALF) >> RAW_FULL_SCALE_SHIFT);
}

/* The product does not fit 32 bits for the 2000 dps range, so 64 bit arithmetic is used */
int32_t MPU6050_gyroRawToMdps(int16_t raw) {
    int64_t full_scale_mdps = (int64_t) GYRO_FULL_SCALE_MDPS_RANGE_0 << gyro_range;
    return (int32_t) (((int64_t) raw * full_scale_mdps + ROUNDING_HALF) >> RAW_FULL_SCALE_SHIFT);
}

int16_t MPU6050_temperatureRawToCentiCelsius(int16_t raw) {
    return (int16_t) ((int32_t) raw * 100 / TEMPERATURE_SENSITIVITY + TEMPERATURE_OFFSET_CENTI_CELSIUS);
}
