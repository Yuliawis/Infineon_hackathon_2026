#ifndef LOG_FIFO_H
#define LOG_FIFO_H

// Small byte FIFO for buffering log text until it can be sent over BLE.
// Header-only so both cores can use it. All functions are safe to call from ISRs
// (they briefly disable interrupts on the calling core). Not safe across cores.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "cy_syslib.h"

typedef struct {
    uint8_t* buf;
    size_t size;               // capacity in bytes
    size_t head;               // next write index
    size_t tail;               // next read index
    size_t count;              // bytes currently stored
    volatile uint32_t dropped; // bytes rejected because the FIFO was full
} log_fifo_t;

#define LOG_FIFO_INIT(storage) {.buf = (storage), .size = sizeof(storage), .head = 0, .tail = 0, .count = 0, .dropped = 0}

// Append the whole chunk, or nothing if it does not fit (keeps lines intact).
static inline bool LogFifo_Push(log_fifo_t* f, const uint8_t* data, size_t len) {
    bool ok = false;
    uint32_t intState = Cy_SysLib_EnterCriticalSection();
    if (len <= (f->size - f->count)) {
        for (size_t i = 0; i < len; i++) {
            f->buf[f->head] = data[i];
            f->head = (f->head + 1u == f->size) ? 0u : f->head + 1u;
        }
        f->count += len;
        ok = true;
    } else {
        f->dropped += len;
    }
    Cy_SysLib_ExitCriticalSection(intState);
    return ok;
}

// Copy up to maxLen bytes without removing them. Returns the number of bytes copied.
static inline size_t LogFifo_Peek(log_fifo_t* f, uint8_t* out, size_t maxLen) {
    uint32_t intState = Cy_SysLib_EnterCriticalSection();
    size_t n = (f->count < maxLen) ? f->count : maxLen;
    size_t idx = f->tail;
    for (size_t i = 0; i < n; i++) {
        out[i] = f->buf[idx];
        idx = (idx + 1u == f->size) ? 0u : idx + 1u;
    }
    Cy_SysLib_ExitCriticalSection(intState);
    return n;
}

// Remove len bytes from the front (after they were peeked and sent).
static inline void LogFifo_Drop(log_fifo_t* f, size_t len) {
    uint32_t intState = Cy_SysLib_EnterCriticalSection();
    if (len > f->count) {
        len = f->count;
    }
    f->tail = (f->tail + len) % f->size;
    f->count -= len;
    Cy_SysLib_ExitCriticalSection(intState);
}

static inline size_t LogFifo_Used(const log_fifo_t* f) {
    return f->count;
}

#endif /* LOG_FIFO_H */
