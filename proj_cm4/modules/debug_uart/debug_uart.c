#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include "debug_uart.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "log_fifo.h"

#if defined(BLE_LOG_ENABLED)
static uint8_t bleLogStorage[2048];
static log_fifo_t bleLogFifo = LOG_FIFO_INIT(bleLogStorage);
#endif

cy_rslt_t Debug_Init(void) {
    cy_rslt_t result = cyhal_uart_init_cfg(&cy_retarget_io_uart_obj, &scb_5_hal_config);
    if (result == CY_RSLT_SUCCESS) {
        result = cy_retarget_io_init_hal();
    }
    return result;
}

static void debugVprintf(bool toBle, const char* fmt, va_list args) {
    char buf[DEBUG_LINE_MAX];

    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    if (n <= 0) {
        return;
    }
    // vsnprintf returns the untruncated length; clamp to what actually fit
    size_t len = ((size_t) n < sizeof(buf)) ? (size_t) n : sizeof(buf) - 1u;
    (void) len;  // unused when neither output is enabled
    (void) toBle;

#if DEBUG_UART_OWNER
    fwrite(buf, 1u, len, stdout);
#endif
#if defined(BLE_LOG_ENABLED)
    if (toBle) {
        (void) LogFifo_Push(&bleLogFifo, (const uint8_t*) buf, len);
    }
#endif
}

void Debug_Printf(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    debugVprintf(true, fmt, args);
    va_end(args);
}

void Debug_PrintfLocal(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    debugVprintf(false, fmt, args);
    va_end(args);
}

#if defined(BLE_LOG_ENABLED)
void Debug_BleLogPush(const uint8_t* data, size_t len) {
    (void) LogFifo_Push(&bleLogFifo, data, len);
}

size_t Debug_BleLogPeek(uint8_t* out, size_t maxLen) {
    return LogFifo_Peek(&bleLogFifo, out, maxLen);
}

void Debug_BleLogDrop(size_t len) {
    LogFifo_Drop(&bleLogFifo, len);
}

size_t Debug_BleLogUsed(void) {
    return LogFifo_Used(&bleLogFifo);
}
#endif
