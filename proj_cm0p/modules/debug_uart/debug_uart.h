#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include "cy_result.h"

// Route printf/scanf to the debug UART (scb_5, P5[0]/P5[1]) via retarget-io.
// Only the core selected by RETARGET_CORE may call this; printf on the other core will fault.
cy_rslt_t Debug_Init(void);

// Use instead of printf: compiles out unless this core owns the debug UART.
#if defined(RETARGET_CORE_CM0P)
#include <stdio.h>
#define DBG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DBG_PRINTF(...) ((void)0)
#endif

#endif /* DEBUG_UART_H */
