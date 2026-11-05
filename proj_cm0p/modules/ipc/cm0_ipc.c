#include <string.h>
#include "cm0_ipc.h"
#include "cy_device.h"
#include "cy_ipc_pipe.h"
#include "debug_uart.h"

static bool isCM4Ready = true;
static bool isDataAvailableFromCM4 = false;

static ipc_msg_t ipcMsgFromCM4_Local;

// Do not use directly, only pass to IPC functions!
void CM0_MessageCallback(uint32_t *msg)
{
    ipc_msg_t *ipcMsgFromCM4;
    
    if (msg != NULL)
    {
        /* Cast the message received to the IPC structure */
        ipcMsgFromCM4 = (ipc_msg_t*)msg;

        // Store the message locally
        memcpy((void*)&ipcMsgFromCM4_Local,
               (void*)ipcMsgFromCM4,
                sizeof(ipc_msg_t));
        
        // Mark new data is available
        isDataAvailableFromCM4 = true;
    }
}

// Do not use directly, only pass to IPC functions!
void CM0_ReleaseCallback(void)
{
    /* Message processed by CM4. Ready to receive a new message */
    isCM4Ready = true;
    DBG_PRINTF("CM4 Accepted msg!\r\n");
}

bool CM0_IsCM4Ready(void)
{
    return isCM4Ready;
}

bool CM0_isDataAvailableFromCM4(void)
{
    return isDataAvailableFromCM4;
}

ipc_msg_t* CM0_GetCM4Message(void)
{
    // Mark that no new data is available from CM4 (we read it just now).
    isDataAvailableFromCM4 = false;
    // This is a programmer responsibility to check for NULL pointer!
    return &ipcMsgFromCM4_Local;
}

bool CM0_SendCM4Message(ipc_msg_t* msg)
{
    bool ret = true;
    /* Send the string message to CM4 */
    // For the sake of simplicity, we won't record exact error.
    // Inversion is needed so retuned 0 error will be actually converted to true
    ret = (bool)!Cy_IPC_Pipe_SendMessage(CY_IPC_EP_CYPIPE_CM4_ADDR,
                                         CY_IPC_EP_CYPIPE_CM0_ADDR,
                                         (void *)msg,
                                         CM0_ReleaseCallback);
    return ret;
}

/* [] END OF FILE */
