#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stddef.h>
#include <stdint.h>
#include "cy_result.h"

// This core owns the debug UART when RETARGET_CORE=CM4.
#if defined(RETARGET_CORE_CM4)
#define DEBUG_UART_OWNER 1
#else
#define DEBUG_UART_OWNER 0
#endif

// Route printf/scanf to the debug UART (scb_5, P5[0]/P5[1]) via retarget-io.
// Only the core selected by RETARGET_CORE may call this; printf on the other core will fault.
cy_rslt_t Debug_Init(void);

// printf-style log. Goes to the debug UART if this core owns it, and to the BLE log
// (NUS TX notifications) when BLE_LOG_ENABLED. One call is limited to DEBUG_LINE_MAX - 1 characters.
#define DEBUG_LINE_MAX 160
void Debug_Printf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// Use instead of printf: compiles out unless the output goes somewhere.
#if DEBUG_UART_OWNER || defined(BLE_LOG_ENABLED)
#define DBG_PRINTF(...) Debug_Printf(__VA_ARGS__)
#else
#define DBG_PRINTF(...) ((void) 0)
#endif

// Same, but only to the debug UART, never over BLE. For bulky output that would flood the BLE log.
void Debug_PrintfLocal(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
#if DEBUG_UART_OWNER
#define DBG_PRINTF_LOCAL(...) Debug_PrintfLocal(__VA_ARGS__)
#else
#define DBG_PRINTF_LOCAL(...) ((void) 0)
#endif

#if defined(BLE_LOG_ENABLED)
// Log text waiting to be forwarded to CM0+ (which sends it over BLE). ISR-safe.
size_t Debug_BleLogPeek(uint8_t* out, size_t maxLen);
void Debug_BleLogDrop(size_t len);
size_t Debug_BleLogUsed(void);
#endif

#endif /* DEBUG_UART_H */
