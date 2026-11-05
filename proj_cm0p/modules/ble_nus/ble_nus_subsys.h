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

#ifndef BLE_NUS_SUBSYS_H
#define BLE_NUS_SUBSYS_H

#include "cy_ble_stack.h"

enum blePayloadTypes {
    BLE_NUS_PAYLOAD_START = 0x00u,
    BLE_NUS_PAYLOAD_CM0_CMD = 0x01u,
    BLE_NUS_PAYLOAD_CM4_CMD = 0x02u,
    BLE_NUS_PAYLOAD_END = BLE_NUS_PAYLOAD_CM4_CMD
};

// Start the UART Service subroutine.
// It will loop forever, so no further statements would be executed.
void BLE_NUS_Init(void);

cy_en_ble_api_result_t sendDataNotificationNUS(uint8_t* data, uint16_t len);

#endif /* BLE_NUS_SUBSYS_H */

/* [] END OF FILE */
