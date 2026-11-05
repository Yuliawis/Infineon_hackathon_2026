#include "i2c_common.h"
#include "cy_sysint.h"
#include "cybsp.h"

#define I2C_MAIN_IRQ_PRIORITY (7u)

cy_stc_scb_i2c_context_t I2C_Main_context;

static void I2C_Main_Isr(void)
{
    Cy_SCB_I2C_Interrupt(I2C_Main_HW, &I2C_Main_context);
}

static const cy_stc_sysint_t I2C_Main_irq_cfg =
{
    .intrSrc      = I2C_Main_IRQ,
    .intrPriority = I2C_MAIN_IRQ_PRIORITY,
};

cy_en_scb_i2c_status_t I2C_Main_Start(void)
{
    cy_en_scb_i2c_status_t status = Cy_SCB_I2C_Init(I2C_Main_HW, &I2C_Main_config, &I2C_Main_context);
    if (status != CY_SCB_I2C_SUCCESS)
    {
        return status;
    }

    if (Cy_SysInt_Init(&I2C_Main_irq_cfg, I2C_Main_Isr) != CY_SYSINT_SUCCESS)
    {
        return CY_SCB_I2C_BAD_PARAM;
    }

    NVIC_EnableIRQ(I2C_Main_irq_cfg.intrSrc);

    Cy_SCB_I2C_Enable(I2C_Main_HW);
    return CY_SCB_I2C_SUCCESS;
}
