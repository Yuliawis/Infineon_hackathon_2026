#ifndef CM0_IPC_H
#define CM0_IPC_H

#include <stdbool.h>
#include "ipc_def.h"

/*******************************************************************************
* Function Name: CM0_MessageCallback()
********************************************************************************
* Summary:
*   Callback function that is executed when a string message is received from
*   CM4.
*
* Parameters:
*   msg: IPC message received
*
*******************************************************************************/
void CM0_MessageCallback(uint32_t *msg);

/*******************************************************************************
* Function Name: CM0_ReleaseCallback()
********************************************************************************
* Summary:
*   Callback function that is executed when the CM4 receives a message, freeing
*   this core to send another message to CM4.
*
*******************************************************************************/
void CM0_ReleaseCallback(void);

/*******************************************************************************
* Function Name: CM0_IsCM4Ready()
********************************************************************************
* Summary:
*    This is a simple getter to check whether CM4 is ready for new IPC commands.
*
*******************************************************************************/
bool CM0_IsCM4Ready(void);

/*******************************************************************************
* Function Name: CM0_isDataAvailableFromCM4()
********************************************************************************
* Summary:
*    Check whether there is new unread data from CM4.
*
*******************************************************************************/
bool CM0_isDataAvailableFromCM4(void);

/*******************************************************************************
* Function Name: CM0_GetCM4Message()
********************************************************************************
* Summary:
*    Retrieve pointer to latest stored IPC message from CM4. It will mark
*    latest IPC data as read.
*    NOTE: This is a programmer responsibility to check for NULL pointer!
*
*******************************************************************************/
ipc_msg_t* CM0_GetCM4Message(void);

/*******************************************************************************
* Function Name: CM0_SendCM4Message()
********************************************************************************
* Summary:
*    Send message to CM4 core via IPC.
*
*******************************************************************************/
bool CM0_SendCM4Message(ipc_msg_t* msg);

#endif /* CM0_IPC_H */

/* [] END OF FILE */
