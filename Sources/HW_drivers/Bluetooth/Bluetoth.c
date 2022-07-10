/*
 * Bluetoth.c
 *
 *  Created on: May 26, 2017
 *      Author: uidq9116
 */


#include "Bluetooth.h"
#include "MCU_drivers/Asy/io_asy.h"

uint8 LED_STATE = 0;
uint8 state = BT_WAIT;
uint8 wait = 0;

void bluetooth_state()
{
	switch(state)
	{
	/* Wait for 2,5 seconds for the system to start up*/
	case BT_WAIT:
	{
		if(wait<50)
			wait++;
		if(wait==50)
		{
			state=BT_RECEIVE;
		}
		break;
	}
	case BT_RECEIVE:
	{
		switch(reception[0])
		{
		case '0':
			LED_STATE = BT_GREEN;
			break;
		case '1':
			LED_STATE = BT_RED;
			break;
		case '2':
			LED_STATE = BT_BLUE;
			break;
		default:
			break;
		}
		//reception[0]='0';
		break;
	}
	default: break;
	}
}

