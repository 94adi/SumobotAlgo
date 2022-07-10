/*
 * Bluetooth.h
 *
 *  Created on: May 26, 2017
 *      Author: uidq9116
 */

#ifndef SOURCES_HW_DRIVERS_BLUETOOTH_BLUETOOTH_H_
#define SOURCES_HW_DRIVERS_BLUETOOTH_BLUETOOTH_H_
#include "Platform_Types/platform_types.h"

extern uint8 state;
extern uint8 LED_STATE;
uint8 wait;
extern void bluetooth_state();

#define BT_WAIT 0
#define BT_RECEIVE 1
#define BT_GREEN			0
#define BT_RED				1
#define BT_BLUE				2
#define BT_DEFAULT			3
#endif /* SOURCES_HW_DRIVERS_BLUETOOTH_BLUETOOTH_H_ */
