#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "cy_pdl.h"
#include "cybsp.h"

#include "debug_uart.h"
#include "cm4_ipc.h"
#include "cm4_cmd.h"
#include "cm0_cm4_shared_cmd.h"
#include "cm4_command_list.h"

static ipc_msg_t ipcMsgForCM0 = {/* IPC structure to be sent to CM0 */
                                 .clientId = IPC_CM4_TO_CM0_CLIENT_ID,
                                 .userCode = 0,
                                 .intrMask = CY_SYS_CYPIPE_INTR_MASK,
                                 .buffer = {},
                                 .len = 0};

static void processCM4Command(ipc_msg_t* msg);

#if defined(BLE_LOG_ENABLED)
// Separate from ipcMsgForCM0 so a log transfer never clobbers a command reply being built.
static ipc_msg_t ipcLogForCM0 = {.clientId = IPC_CM4_TO_CM0_CLIENT_ID,
                                 .userCode = IPC_USR_CODE_CMD,
                                 .intrMask = CY_SYS_CYPIPE_INTR_MASK,
                                 .buffer = {},
                                 .len = 0};

// Hand buffered CM4 log text to CM0+, which sends it over BLE. Must not log anything itself.
void CM4_ForwardLogsToCM0(void) {
    // len is uint8_t and buffer[0] holds the command, so at most 254 bytes of text per message
    const size_t maxText = 254u;

    if (!CM4_IsCM0Ready()) {
        return;
    }
    size_t n = Debug_BleLogPeek(&ipcLogForCM0.buffer[1], maxText);
    if (n == 0u) {
        return;
    }
    // If more is waiting, cut at the last full line so CM0+ logs don't land mid-line
    if (n == maxText) {
        size_t cut = n;
        while ((cut > 0u) && (ipcLogForCM0.buffer[cut] != '\n')) {
            cut--;
        }
        if (cut > 0u) {
            n = cut;
        }
    }
    ipcLogForCM0.buffer[0] = (uint8_t) CM0_SHARED_LOG;
    ipcLogForCM0.len = (uint8_t) (n + 1u);
    if (CM4_SendCM0Message(&ipcLogForCM0)) {
        Debug_BleLogDrop(n);
    }
}
#endif

void CM4_ProcessIncomingIPCMessage(ipc_msg_t* msg) {
    // In general, impossible situation, but never trust anyone.
    if (msg != NULL) {
        // Process messages with at least 1 byte of payload
        if (msg->len > 0) {
            // Decode message code. You would mostly like to use Commands
            switch (msg->userCode) {
                case IPC_USR_CODE_CMD: {
                    // Only [0] element was used for simplicity.
                    // IDEA: You could expand it to accept strings or more complex data packets!
                    // You can define array of strings and perform memcmp. This operation is a bit more costful.
                    processCM4Command(msg);
                    break;
                }
                case IPC_USR_CODE_REQ: {
                    // Not implemented. You can be very creative here!
                    break;
                }
                default: break;
            }
        }
    }
}

// msg->buffer[0] is the command, msg->buffer[1..len-1] are its parameters.
static void processCM4Command(ipc_msg_t* msg) {
    switch ((enum cm4CommandList) msg->buffer[0]) {
        case CM4_COMMAND_LED_ENA: {
            Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, 0);
            Cy_GPIO_Write(CYBSP_USER_LED2_PORT, CYBSP_USER_LED2_NUM, 0);
            break;
        }
        case CM4_COMMAND_LED_DIS: {
            Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, 1);
            Cy_GPIO_Write(CYBSP_USER_LED2_PORT, CYBSP_USER_LED2_NUM, 1);
            break;
        }
        case CM4_COMMAND_CAR_SAY: {
            if (CM4_IsCM0Ready()) {
                ipcMsgForCM0.userCode = IPC_USR_CODE_CMD;
                ipcMsgForCM0.len = 0x01;
                ipcMsgForCM0.buffer[0] = (uint8_t) CM0_SHARED_CAR_SAY;
                CM4_SendCM0Message(&ipcMsgForCM0);
            }
            break;
        }
        case CM4_COMMAND_ECHO: {
            if (CM4_IsCM0Ready()) {
                ipcMsgForCM0.userCode = IPC_USR_CODE_CMD;
                ipcMsgForCM0.len = msg->len;
                ipcMsgForCM0.buffer[0] = (uint8_t) CM0_SHARED_BLE_NTF_RELAY;
                memcpy(&ipcMsgForCM0.buffer[1], &msg->buffer[1], ipcMsgForCM0.len - 1);
                CM4_SendCM0Message(&ipcMsgForCM0);
            }
            break;
        }
        default: break;
    }
}


/* [] END OF FILE */
