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

#ifndef CM4_COMMAND_LIST_H
#define CM4_COMMAND_LIST_H
    
enum cm4CommandList
{
    CM4_COMMANDS_START = 0x00,
    // Those are dummy commands to give you the starting point.
    CM4_COMMAND_LED_ENA = 0x01,
    CM4_COMMAND_LED_DIS = 0x02,
    CM4_COMMAND_CAR_SAY = 0x03,
    CM4_COMMAND_ECHO    = 0x04,
    CM4_COMMAND_END = CM4_COMMAND_ECHO,
};

#endif /* CM4_COMMAND_LIST_H */

/* [] END OF FILE */
