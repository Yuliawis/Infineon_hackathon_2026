#ifndef I2C_COMMON_H
#define I2C_COMMON_H

#include "cy_scb_i2c.h"

// Shared PDL context for I2C_Main (SCB2)
extern cy_stc_scb_i2c_context_t I2C_Main_context;

// Init I2C_Main as master, hook its interrupt and enable it. Call once after cybsp_init().
cy_en_scb_i2c_status_t I2C_Main_Start(void);

#endif /* I2C_COMMON_H */
