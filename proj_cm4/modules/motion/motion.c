#include "motion.h"
#include "MPU6050.h"
#include "car.h"
#include "debug_uart.h"
#include "i2c_common.h"
#include "cybsp.h"
#include "utils.h"

#define GYRO_RANGE  (MPU6050_GYRO_RANGE_500_DPS)
#define ACCEL_RANGE (MPU6050_ACCEL_RANGE_4G)

#define CALIBRATION_SAMPLES          (60)
#define CALIBRATION_SETTLE_SAMPLES   (20)
#define CALIBRATION_SAMPLE_PERIOD_MS (10u)
#define CALIBRATION_ATTEMPTS         (2u)
#define CALIBRATION_RETRY_DELAY_MS   (100u)
// If the gyro readings spread more than this during calibration, the car was moving
#define CALIBRATION_MAX_SPREAD_MDPS (3000)

// Gyro noise below this is not integrated into yaw to limit drift while standing still
#define YAW_RATE_DEADBAND_MDPS (200)

// Gap longer than this means stale data, so the gyro is not integrated over it
#define MAX_INTEGRATION_STEP_MS (500u)

// Yaw is accumulated in microdegrees (rate in mdps times time in ms), so slow rotation is not lost to rounding.
// 180 degrees = 180000000, so it fits int32 with room for one integration step.
#define MICRODEGREES_PER_MILLIDEGREE (1000)
#define YAW_HALF_TURN_UDEG           (180000000)
#define YAW_FULL_TURN_UDEG           (360000000)
#define HALF_TURN_MDEG               (180000)
#define FULL_TURN_MDEG               (360000)
#define MILLISECONDS_PER_SECOND      (1000)
#define MILLI_PER_UNIT               (1000)
#define MILLI_HALF                   (500)

// Complementary filter: gyro is trusted short term, accelerometer long term
#define TILT_GYRO_WEIGHT_PERCENT (98)
// Accelerometer tilt is ignored while total acceleration differs from 1 g by more than 0.3 g (car is accelerating)
#define TILT_ACCEL_MIN_MG (700)
#define TILT_ACCEL_MAX_MG (1300)

static bool initialized = false;
static uint32_t last_update_ms = 0;

static MPU6050_GyroMdps gyro_bias;
static MPU6050_AccelMg accel;
static int32_t yaw_rate = 0;
static int32_t pitch_rate = 0;
static int32_t roll_rate = 0;
static int32_t yaw_udeg = 0;
static int32_t pitch = 0;
static int32_t roll = 0;

static int32_t wrapAngleMdeg(int32_t angle) {
    while (angle > HALF_TURN_MDEG) {
        angle -= FULL_TURN_MDEG;
    }
    while (angle <= -HALF_TURN_MDEG) {
        angle += FULL_TURN_MDEG;
    }
    return angle;
}

static int32_t wrapYawUdeg(int32_t angle) {
    while (angle > YAW_HALF_TURN_UDEG) {
        angle -= YAW_FULL_TURN_UDEG;
    }
    while (angle <= -YAW_HALF_TURN_UDEG) {
        angle += YAW_FULL_TURN_UDEG;
    }
    return angle;
}

static int32_t absolute(int32_t value) {
    return value < 0 ? -value : value;
}

/* Thousandths to whole units, rounded to the nearest */
static int16_t milliToUnits(int32_t value) {
    return (int16_t) ((value + (value < 0 ? -MILLI_HALF : MILLI_HALF)) / MILLI_PER_UNIT);
}

static bool accelTiltIsReliable(void) {
    int32_t magnitude_squared = (int32_t) accel.x * accel.x + (int32_t) accel.y * accel.y + (int32_t) accel.z * accel.z;
    return magnitude_squared > TILT_ACCEL_MIN_MG * TILT_ACCEL_MIN_MG &&
           magnitude_squared < TILT_ACCEL_MAX_MG * TILT_ACCEL_MAX_MG;
}

static int32_t accelPitch(void) {
    int32_t yz_magnitude = (int32_t) sqrt_int((uint32_t) ((int32_t) accel.y * accel.y + (int32_t) accel.z * accel.z));
    return atan2_mdeg(accel.x, yz_magnitude);
}

static int32_t accelRoll(void) {
    return atan2_mdeg(accel.y, accel.z);
}

bool Motion_CalibrateGyro(void) {
    MPU6050_GyroMdps sum = {0, 0, 0};
    MPU6050_GyroMdps min = {0, 0, 0};
    MPU6050_GyroMdps max = {0, 0, 0};

    for (int32_t sample_n = 0; sample_n < CALIBRATION_SETTLE_SAMPLES + CALIBRATION_SAMPLES; ++sample_n) {
        MPU6050_GyroMdps gyro;
        Cy_SysLib_Delay(CALIBRATION_SAMPLE_PERIOD_MS);
        if (!MPU6050_readGyro(&gyro)) {
            DBG_PRINTF("Motion: gyro read failed at sample %ld, I2C status 0x%08lX\n", (long) sample_n,
                       (unsigned long) Cy_SCB_I2C_MasterGetStatus(I2C_Main_HW, &I2C_Main_context));
            return false;
        }
        // The first samples after wake-up are not stable yet
        if (sample_n < CALIBRATION_SETTLE_SAMPLES) {
            continue;
        }
        if (sample_n == CALIBRATION_SETTLE_SAMPLES) {
            min = gyro;
            max = gyro;
        }
        sum.x += gyro.x;
        sum.y += gyro.y;
        sum.z += gyro.z;
        min.x = (gyro.x < min.x) ? gyro.x : min.x;
        min.y = (gyro.y < min.y) ? gyro.y : min.y;
        min.z = (gyro.z < min.z) ? gyro.z : min.z;
        max.x = (gyro.x > max.x) ? gyro.x : max.x;
        max.y = (gyro.y > max.y) ? gyro.y : max.y;
        max.z = (gyro.z > max.z) ? gyro.z : max.z;
    }

    if (max.x - min.x > CALIBRATION_MAX_SPREAD_MDPS || max.y - min.y > CALIBRATION_MAX_SPREAD_MDPS ||
        max.z - min.z > CALIBRATION_MAX_SPREAD_MDPS) {
        DBG_PRINTF("Motion: gyro spread too large (mdps): x %ld, y %ld, z %ld; mean x %ld, y %ld, z %ld\n",
                   (long) (max.x - min.x), (long) (max.y - min.y), (long) (max.z - min.z),
                   (long) (sum.x / CALIBRATION_SAMPLES), (long) (sum.y / CALIBRATION_SAMPLES),
                   (long) (sum.z / CALIBRATION_SAMPLES));
        return false;
    }

    gyro_bias.x = sum.x / CALIBRATION_SAMPLES;
    gyro_bias.y = sum.y / CALIBRATION_SAMPLES;
    gyro_bias.z = sum.z / CALIBRATION_SAMPLES;
    return true;
}

bool Motion_Init(void) {
    initialized = false;

    if (!MPU6050_Init()) {
        DBG_PRINTF("Motion: MPU6050 init failed, WHO_AM_I 0x%02X (expected 0x%02X), I2C status 0x%08lX\n",
                   MPU6050_readWhoAmI(), MPU6050_WHO_AM_I_VALUE,
                   (unsigned long) Cy_SCB_I2C_MasterGetStatus(I2C_Main_HW, &I2C_Main_context));
        return false;
    }
    if (!MPU6050_setGyroRange(GYRO_RANGE) || !MPU6050_setAccelRange(ACCEL_RANGE)) {
        DBG_PRINTF("Motion: range setup failed\n");
        return false;
    }
    bool calibrated = false;
    for (uint8_t attempt = 0; attempt < CALIBRATION_ATTEMPTS; ++attempt) {
        DBG_PRINTF("Motion: gyro calibration attempt %u/%u\n", (unsigned) (attempt + 1u),
                   (unsigned) CALIBRATION_ATTEMPTS);
        if (Motion_CalibrateGyro()) {
            calibrated = true;
            break;
        }

        if (attempt + 1u < CALIBRATION_ATTEMPTS) {
            Cy_SysLib_Delay(CALIBRATION_RETRY_DELAY_MS);
        }
    }
    if (!calibrated) {
        DBG_PRINTF("Motion: gyro calibration failed after %u attempts\n", (unsigned) CALIBRATION_ATTEMPTS);
        return false;
    }
    if (!MPU6050_readAccel(&accel)) {
        DBG_PRINTF("Motion: accel read failed\n");
        return false;
    }

    yaw_udeg = 0;
    pitch = accelPitch();
    roll = accelRoll();
    yaw_rate = 0;
    pitch_rate = 0;
    roll_rate = 0;
    last_update_ms = Timing_GetMilliseconds();
    initialized = true;
    return true;
}

bool Motion_Update(void) {
    uint32_t now_ms = Timing_GetMilliseconds();
    MPU6050_Data data;

    if (!initialized || (now_ms - last_update_ms) < MOTION_UPDATE_PERIOD_MS) {
        return false;
    }

    if (!MPU6050_readAll(&data)) {
        /* last_update_ms is left alone, so the next good sample integrates over the whole gap. Moving
         * it forward here threw away the rotation of every interval whose read failed. */
        return false;
    }

    uint32_t dt_ms = now_ms - last_update_ms;
    last_update_ms = now_ms;

    accel = data.accel;
    yaw_rate = MOTION_YAW_DIRECTION * (data.gyro.z - gyro_bias.z);
    roll_rate = data.gyro.x - gyro_bias.x;
    pitch_rate = -(data.gyro.y - gyro_bias.y);

    if (dt_ms > MAX_INTEGRATION_STEP_MS) {
        return true;
    }

    if (absolute(yaw_rate) > YAW_RATE_DEADBAND_MDPS) {
        yaw_udeg = wrapYawUdeg(yaw_udeg + yaw_rate * (int32_t) dt_ms);
    }

    pitch += pitch_rate * (int32_t) dt_ms / MILLISECONDS_PER_SECOND;
    roll += roll_rate * (int32_t) dt_ms / MILLISECONDS_PER_SECOND;
    if (accelTiltIsReliable()) {
        pitch = (TILT_GYRO_WEIGHT_PERCENT * pitch + (100 - TILT_GYRO_WEIGHT_PERCENT) * accelPitch()) / 100;
        roll = (TILT_GYRO_WEIGHT_PERCENT * roll + (100 - TILT_GYRO_WEIGHT_PERCENT) * accelRoll()) / 100;
    }
    return true;
}

int16_t Motion_GetYawRate(void) {
    return milliToUnits(yaw_rate);
}

int16_t Motion_GetPitchRate(void) {
    return milliToUnits(pitch_rate);
}

int16_t Motion_GetRollRate(void) {
    return milliToUnits(roll_rate);
}

int32_t Motion_GetYawRateMdps(void) {
    return yaw_rate;
}

int32_t Motion_GetPitchRateMdps(void) {
    return pitch_rate;
}

int32_t Motion_GetRollRateMdps(void) {
    return roll_rate;
}

int16_t Motion_GetYaw(void) {
    return milliToUnits(Motion_GetYawMdeg());
}

int32_t Motion_GetYawMdeg(void) {
    return yaw_udeg / MICRODEGREES_PER_MILLIDEGREE;
}

void Motion_ResetYaw(void) {
    yaw_udeg = 0;
}

int16_t Motion_GetYawError(int16_t target_yaw) {
    return milliToUnits(Motion_GetYawErrorMdeg((int32_t) target_yaw * MILLI_PER_UNIT));
}

int32_t Motion_GetYawErrorMdeg(int32_t target_yaw_mdeg) {
    return wrapAngleMdeg(target_yaw_mdeg - Motion_GetYawMdeg());
}

Motion_TurnState Motion_GetTurnState(void) {
    if (yaw_rate > MOTION_TURN_RATE_THRESHOLD_MDPS) {
        return MOTION_TURN_LEFT;
    }
    if (yaw_rate < -MOTION_TURN_RATE_THRESHOLD_MDPS) {
        return MOTION_TURN_RIGHT;
    }
    return MOTION_TURN_NONE;
}

bool Motion_IsTurning(void) {
    return Motion_GetTurnState() != MOTION_TURN_NONE;
}

int16_t Motion_GetAccelXMg(void) {
    return accel.x;
}

int16_t Motion_GetAccelYMg(void) {
    return accel.y;
}

int16_t Motion_GetAccelZMg(void) {
    return accel.z;
}

int16_t Motion_GetPitch(void) {
    return milliToUnits(pitch);
}

int16_t Motion_GetRoll(void) {
    return milliToUnits(roll);
}

int32_t Motion_GetPitchMdeg(void) {
    return pitch;
}

int32_t Motion_GetRollMdeg(void) {
    return roll;
}
