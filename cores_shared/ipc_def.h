#ifndef IPC_DEF_H
#define IPC_DEF_H

#include <stdint.h>

#define IPC_BUFFER_SIZE          256
#define IPC_CM0_TO_CM4_CLIENT_ID 0x00u
#define IPC_CM4_TO_CM0_CLIENT_ID 0x01u

// Custom user codes to distinguish between different kinds of payloads
// Currently only CMD payloads are used
#define IPC_USR_CODE_CMD 0x01u
#define IPC_USR_CODE_REQ 0x02u
#define IPC_USR_CODE_RSP 0x03u

typedef struct __attribute__((packed, aligned(4))) {
    uint8_t clientId;
    uint8_t userCode;
    uint16_t intrMask;
    uint8_t buffer[IPC_BUFFER_SIZE];
    uint8_t len;
} ipc_msg_t;

#endif /* IPC_DEF_H */

/* [] END OF FILE */
