#ifndef CM4_CMD_H
#define CM4_CMD_H

#include "ipc_def.h"

/*******************************************************************************
* Function Name: CM4_ProcessIncomingIPCMessage()
********************************************************************************
* Summary:
*   Decode a message received from CM0+ and dispatch it (commands, requests).
*
* Parameters:
*   msg: message returned by CM4_GetCM0Message(). NULL is ignored.
*
*******************************************************************************/
void CM4_ProcessIncomingIPCMessage(ipc_msg_t* msg);

#if defined(BLE_LOG_ENABLED)
/*******************************************************************************
* Function Name: CM4_ForwardLogsToCM0()
********************************************************************************
* Summary:
*   Hand buffered CM4 log text to CM0+, which sends it over BLE. Call it
*   periodically from the main loop. Must not log anything itself.
*
*******************************************************************************/
void CM4_ForwardLogsToCM0(void);

/*******************************************************************************
* Function Name: CM4_FlushLogsToCM0()
********************************************************************************
* Summary:
*   Forward buffered CM4 log text until it is all sent, waiting for CM0+
*   between messages. Blocks for at most about 200 ms. Use it after a long
*   blocking job that logged a lot. Must not log anything itself.
*
*******************************************************************************/
void CM4_FlushLogsToCM0(void);
#endif

#endif /* CM4_CMD_H */

/* [] END OF FILE */
