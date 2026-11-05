#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <stdbool.h>

#define MPU6050_WHO_AM_I_VALUE (0x68u)

typedef enum {
    MPU6050_ACCEL_RANGE_2G = 0,
    MPU6050_ACCEL_RANGE_4G = 1,
    MPU6050_ACCEL_RANGE_8G = 2,
    MPU6050_ACCEL_RANGE_16G = 3
} MPU6050_AccelRange;

typedef enum {
    MPU6050_GYRO_RANGE_250_DPS = 0,
    MPU6050_GYRO_RANGE_500_DPS = 1,
    MPU6050_GYRO_RANGE_1000_DPS = 2,
    MPU6050_GYRO_RANGE_2000_DPS = 3
} MPU6050_GyroRange;

/* Digital low pass filter setting (CONFIG register DLPF_CFG). */
typedef enum {
    MPU6050_DLPF_260_HZ = 0,
    MPU6050_DLPF_184_HZ = 1,
    MPU6050_DLPF_94_HZ = 2,
    MPU6050_DLPF_44_HZ = 3,
    MPU6050_DLPF_21_HZ = 4,
    MPU6050_DLPF_10_HZ = 5,
    MPU6050_DLPF_5_HZ = 6
} MPU6050_Dlpf;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} MPU6050_Vector3Raw;

/* Acceleration in milli-g (1000 = 1 g). Fits int16 for all ranges (16 g = 16000). */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} MPU6050_AccelMg;

/* Angular rate in millidegrees per second (1000 = 1 degree per second). Up to 2000000 for the 2000 dps range. */
typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} MPU6050_GyroMdps;

typedef struct {
    MPU6050_Vector3Raw accel;
    MPU6050_Vector3Raw gyro;
    int16_t temperature;
} MPU6050_RawData;

typedef struct {
    MPU6050_AccelMg accel;
    MPU6050_GyroMdps gyro;
    int16_t temperature; /* hundredths of a degree Celsius (2500 = 25.00 C) */
} MPU6050_Data;

union PwrMgmt1Register {
    struct {
        uint8_t clksel : 3;
        uint8_t temp_dis : 1;
        uint8_t reserved : 1;
        uint8_t cycle : 1;
        uint8_t sleep : 1;
        uint8_t device_reset : 1;
    } fields;
    uint8_t data;
};

/* All functions returning bool return false on I2C failure. */
bool MPU6050_Init(void);
bool MPU6050_isConnected(void);
uint8_t MPU6050_readWhoAmI(void);

void MPU6050_sleep(void);
void MPU6050_wake(void);
void MPU6050_reset(void);

bool MPU6050_setAccelRange(MPU6050_AccelRange range);
bool MPU6050_setGyroRange(MPU6050_GyroRange range);
bool MPU6050_setDlpf(MPU6050_Dlpf dlpf);
bool MPU6050_setSampleRateDivider(uint8_t divider);

bool MPU6050_readAccelRaw(MPU6050_Vector3Raw* accel);
bool MPU6050_readGyroRaw(MPU6050_Vector3Raw* gyro);
bool MPU6050_readTemperatureRaw(int16_t* temperature);
bool MPU6050_readAllRaw(MPU6050_RawData* data);

bool MPU6050_readAccel(MPU6050_AccelMg* accel);
bool MPU6050_readGyro(MPU6050_GyroMdps* gyro);
bool MPU6050_readTemperature(int16_t* temperature_centi_celsius);
bool MPU6050_readAll(MPU6050_Data* data);

/* Conversions use the range that was set last */
int16_t MPU6050_accelRawToMg(int16_t raw);
int32_t MPU6050_gyroRawToMdps(int16_t raw);
int16_t MPU6050_temperatureRawToCentiCelsius(int16_t raw);

#ifdef __cplusplus
}
#endif

#endif
