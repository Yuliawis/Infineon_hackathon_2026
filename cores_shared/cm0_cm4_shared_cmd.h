/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

// This file is intended so each core can know each other special commands.
// Actually not very applicable for CM0 since it mostly retranslates BLE commands.
// But you can be creative here.

#ifndef CM0_CM4_SHARED_COMMAND_LIST_H
#define CM0_CM4_SHARED_COMMAND_LIST_H

enum cm0SharedCMD
{
    CM0_SHARED_START = 0x00,
    CM0_SHARED_BLE_NTF_RELAY = 0x01,
    CM0_SHARED_CAR_SAY = 0x02,
    CM0_SHARED_END = CM0_SHARED_CAR_SAY
};
    
#endif /* CM0_CM4_SHARED_COMMAND_LIST_H */
    
/* [] END OF FILE */
