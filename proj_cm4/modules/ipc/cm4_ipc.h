
#ifndef CM4_IPC_H
#define CM4_IPC_H

#include <stdint.h>
#include <stdbool.h>
#include "ipc_def.h"

/*******************************************************************************
* Function Name: CM4_MessageCallback()
********************************************************************************
* Summary:
*   Callback function that is executed when a string message is received from
*   CM0.
*
* Parameters:
*   msg: IPC message received
*
*******************************************************************************/
void CM4_MessageCallback(uint32_t *msg);

/*******************************************************************************
* Function Name: CM4_ReleaseCallback()
********************************************************************************
* Summary:
*   Callback function that is executed when the CM0 receives a message, freeing
*   this core to send another message to CM0.
*
*******************************************************************************/
void CM4_ReleaseCallback(void);

/*******************************************************************************
* Function Name: CM4_IsCM0Ready()
********************************************************************************
* Summary:
*    This is a simple getter to check whether CM0 is ready for new IPC commands.
*
*******************************************************************************/
bool CM4_IsCM0Ready(void);

/*******************************************************************************
* Function Name: CM4_isDataAvailableFromCM0()
********************************************************************************
* Summary:
*    Check whether there is new unread data from CM0.
*
*******************************************************************************/
bool CM4_isDataAvailableFromCM0(void);

/*******************************************************************************
* Function Name: CM4_GetCM0Message()
********************************************************************************
* Summary:
*    Retrieve pointer to latest stored IPC message from CM0. It will mark
*    latest IPC data as read.
*    NOTE: This is a programmer responsibility to check for NULL pointer!
*
*******************************************************************************/
ipc_msg_t* CM4_GetCM0Message(void);

/*******************************************************************************
* Function Name: CM4_SendCM0Message()
********************************************************************************
* Summary:
*    Send message to CM0 core via IPC.
*
*******************************************************************************/
bool CM4_SendCM0Message(ipc_msg_t* msg);

#endif /* CM4_IPC_H */

/* [] END OF FILE */
