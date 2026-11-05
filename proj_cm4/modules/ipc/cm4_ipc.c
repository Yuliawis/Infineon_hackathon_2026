#include <string.h>
#include "ipc_def.h"
#include "cm4_ipc.h"
#include "cy_device.h"
#include "cy_ipc_pipe.h"

static volatile bool isCM0Ready = true;
static volatile bool isDataAvailableFromCM0 = false;

static ipc_msg_t ipcMsgFromCM0_Local;

// Do not use directly, only pass to IPC functions!
void CM4_MessageCallback(uint32_t* msg) {
    ipc_msg_t* ipcMsgFromCM0;

    if (msg != NULL) {
        /* Cast the message received to the IPC structure */
        ipcMsgFromCM0 = (ipc_msg_t*) msg;

        // Store the message locally
        memcpy((void*) &ipcMsgFromCM0_Local, (void*) ipcMsgFromCM0, sizeof(ipc_msg_t));

        // Mark new data is available
        isDataAvailableFromCM0 = true;
    }
}

// Do not use directly, only pass to IPC functions!
void CM4_ReleaseCallback(void) {
    /* Message processed by CM0. Ready to receive a new message */
    isCM0Ready = true;
}

bool CM4_IsCM0Ready(void) {
    return isCM0Ready;
}

bool CM4_isDataAvailableFromCM0(void) {
    return isDataAvailableFromCM0;
}

ipc_msg_t* CM4_GetCM0Message(void) {
    // Mark that no new data is available from CM0 (we read it just now).
    isDataAvailableFromCM0 = false;
    // This is a programmer responsibility to check for NULL pointer!
    return &ipcMsgFromCM0_Local;
}

bool CM4_SendCM0Message(ipc_msg_t* msg) {
    bool ret = true;
    // Mark busy BEFORE sending: the release callback may fire before SendMessage returns.
    isCM0Ready = false;
    /* Send the string message to CM0 */
    // For the sake of simplicity, we won't record exact error.
    // Inversion is needed so retuned 0 error will be actually converted to true
    ret = (bool) !Cy_IPC_Pipe_SendMessage(CY_IPC_EP_CYPIPE_CM0_ADDR, CY_IPC_EP_CYPIPE_CM4_ADDR, (void*) msg,
                                          CM4_ReleaseCallback);
    if (!ret) {
        // Message was not sent, so no release callback will come
        isCM0Ready = true;
    }
    return ret;
}


/* [] END OF FILE */
