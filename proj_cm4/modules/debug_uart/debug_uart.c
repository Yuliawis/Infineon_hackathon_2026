#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include "debug_uart.h"
#include "car.h"  // Timing_GetMilliseconds()
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

void Debug_Printf(const char* fmt, ...) {
    char buf[DEBUG_LINE_MAX];
    va_list args;

    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (n <= 0) {
        return;
    }
    // vsnprintf returns the untruncated length; clamp to what actually fit
    size_t len = ((size_t) n < sizeof(buf)) ? (size_t) n : sizeof(buf) - 1u;
    (void) len;  // unused when neither output is enabled

#if DEBUG_TIMESTAMP
    // Put "[s.mmm] " (time since Timing_Init()) at the start of every non-empty line.
    // A line may be built from several calls, so remember whether the last call ended one.
    static bool atLineStart = true;
    char out[DEBUG_LINE_MAX + 3u * DEBUG_STAMP_MAX];  // room for 3 stamps, the rest is cut
    size_t outLen = 0;
    uint32_t ms = Timing_GetMilliseconds();

    for (size_t i = 0; i < len; i++) {
        if (atLineStart && (buf[i] != '\n')) {
            if (outLen + DEBUG_STAMP_MAX > sizeof(out)) {
                break;
            }
            int s = snprintf(&out[outLen], sizeof(out) - outLen, "[%lu.%03lu] ", (unsigned long) (ms / 1000u),
                             (unsigned long) (ms % 1000u));
            if (s > 0) {
                outLen += (size_t) s;
            }
        }
        if (outLen >= sizeof(out)) {
            break;
        }
        out[outLen++] = buf[i];
        atLineStart = (buf[i] == '\n');
    }
    atLineStart = (buf[len - 1u] == '\n');  // also right when the text above was cut

    const char* text = out;
    len = outLen;
#else
    const char* text = buf;
#endif
    (void) text;

#if DEBUG_UART_OWNER
    fwrite(text, 1u, len, stdout);
#endif
#if defined(BLE_LOG_ENABLED)
    (void) LogFifo_Push(&bleLogFifo, (const uint8_t*) text, len);
#endif
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
