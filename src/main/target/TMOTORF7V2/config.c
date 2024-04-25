/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "fc/fc_msp_box.h"

#include "io/piniobox.h"
#include "io/serial.h"

#define BLUETOOTH_MSP_BAUDRATE      BAUD_19200

void targetConfiguration(void)
{
    // serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(SERIAL_PORT_USART5)].functionMask = FUNCTION_MSP;
    // serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(SERIAL_PORT_USART5)].msp_baudrateIndex = BLUETOOTH_MSP_BAUDRATE;
    
    serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(SERIAL_PORT_USART5)].functionMask = FUNCTION_NONE;
    serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(SERIAL_PORT_USART5)].msp_baudrateIndex = BAUD_19200;

    pinioBoxConfigMutable()->permanentId[0] = BOX_PERMANENT_ID_USER1;
    pinioBoxConfigMutable()->permanentId[1] = BOX_PERMANENT_ID_USER2;
}
