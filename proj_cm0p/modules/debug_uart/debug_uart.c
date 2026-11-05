#include "debug_uart.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

cy_rslt_t Debug_Init(void) {
    cy_rslt_t result = cyhal_uart_init_cfg(&cy_retarget_io_uart_obj, &scb_5_hal_config);
    if (result == CY_RSLT_SUCCESS) {
        result = cy_retarget_io_init_hal();
    }
    return result;
}
