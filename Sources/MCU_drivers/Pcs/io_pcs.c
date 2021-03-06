/****************************************************************************

COPYRIGHT (C) $Date: Sept 24, 2015 $
$CompanyInfo: CONTINENTAL AUTOMOTIVE GMBH $
ALL RIGHTS RESERVED.

The reproduction, transmission or use of this document or its contents is
not permitted without express written authority.
Offenders will be liable for damages. All rights, including rights created
by patent grant or registration of a utility model or design, are reserved.
---------------------------------------------------------------------------

$ProjectName: FRDM KL26-Z PLATFORM $

$Log: io_pcs.c $

$Author: Flueran Gabriel $

 ****************************************************************************/

/** Includes **/

#include "MCU_drivers/Dio/io_dio.h"
#include "MCU_drivers/Int/io_int_cnf.h"
#include "MCU_drivers/Pcs/io_pcs_cnf.h"

#include "OTHER_FUNCTIONS/io_func.h"

#include "io_pcs_cnf.h"

/** end of Includes **/


/** Local Functions prototypes **/

// None so far.

/** end Local Functions prototypes **/


/** Global variables and constants **/

const Io_Pcs_CnfTypeStruct *Io_Pcs_CnfPtr;

/* TODO FLUERAN: Initialization of all global variables should be done in the Initialization function of the module */
const PORT_MemMapPtr Io_Pcs_PortTab[5] = PORT_BASE_PTRS;
const FGPIO_MemMapPtr Io_Pcs_FGPIO[5] = FGPIO_BASE_PTRS;

/** end of Global variables and constants **/


/** Functions implementation **/

/** External Interfaces implementation **/

/* FUNCTION NAME: Io_Pcs_Initialization
 * RETURN TYPE: void
 * PARAMETERS: const Io_Pcs_CnfTypeStruct *PtrCnf
 * DESCRIPTION: Initializes the uC's PORTS & PINS for desired functions.
 * OBSERVATIONS: -
 * 	FURTHER
 * IMPROVEMENTS: 1) The pin function is verified and:
 * 				 - if GPIO: set direction
 * 				 - if other: let it as default value
 */
void Io_Pcs_Initialization(const Io_Pcs_CnfTypeStruct *PtrCnf)
{
	uint16 i;

	uint8 Port;
	uint8 Pin;
	uint8 PinMuxing;
	uint8 InitLevel;
	uint8 PinDirection;
	uint8 InterruptType;
	uint32 Config;
	const Io_Pcs_PortTypeStruct *tmp_ptr;

	Io_Pcs_CnfPtr  = PtrCnf;
	tmp_ptr = Io_Pcs_CnfPtr->Io_Pcs_PinPtr;

	for (i = 0; i < Io_Pcs_CnfPtr->Nr_Of_Pins; i++)
	{
		Port = tmp_ptr->Port;
		Pin = tmp_ptr->Pin;
		PinMuxing = tmp_ptr->PinMuxing;
		InitLevel= tmp_ptr->InitLevel;
		PinDirection = tmp_ptr->Direction;
		InterruptType = tmp_ptr->InterruptType;
		Config = tmp_ptr->Cfg;

		/* Enables the clock gate to the PORTx module */
		SIM_SCGC5 |= (1 << Port) << 9;

		/* Write Pin Initial Level */
		if(InitLevel)
		{
			FGPIO_PSOR_REG(Io_Pcs_FGPIO[Port]) = FGPIO_PDOR_REG(Io_Pcs_FGPIO[Port]) | (1 << Pin);
		}
		else
		{
			FGPIO_PCOR_REG(Io_Pcs_FGPIO[Port]) = FGPIO_PCOR_REG(Io_Pcs_FGPIO[Port]) | (1 << Pin);
		}
		/* Write Pin Direction */
		if (PinDirection == IO_PCS_OUTPUT)
		{
			FGPIO_PDDR_REG(Io_Pcs_FGPIO[Port]) = FGPIO_PDDR_REG(Io_Pcs_FGPIO[Port]) | (1 << Pin);
		}

		else
		{
			FGPIO_PDDR_REG(Io_Pcs_FGPIO[Port]) = FGPIO_PDDR_REG(Io_Pcs_FGPIO[Port]) & ~(1 << Pin);
		}

		/* Write Pin configuration */
		PORT_PCR_REG(Io_Pcs_PortTab[Port], Pin) = PORT_PCR_IRQC(InterruptType) | PORT_PCR_MUX(PinMuxing) | Config;

		tmp_ptr++;
	}
}

/* FUNCTION NAME: Io_Pcs_SetPinDirection
 * RETURN TYPE: void
 * PARAMETERS: uint16 PortPin, uint8 dir
 * DESCRIPTION: Sets direction of a given pin.
 * OBSERVATIONS: -
 *  FURTHER
 * IMPROVEMENTS: 1) The pin function is verified and:
 * 				 - if GPIO: set direction
 * 				 - if other: let it as default value / return error
 */
void Io_Pcs_SetPinDirection(uint16 PortPin, uint8 dir)
{
	uint8 Port;
	uint8 Pin;

	/* Get the Port */
	Port = IO_PCS_GET_PORT(PortPin);
	/* Get the Pin */
	Pin  = IO_PCS_GET_PIN(PortPin);

	if (dir == IO_PCS_OUTPUT)
	{
		FGPIO_PDDR_REG(Io_Pcs_FGPIO[Port]) = FGPIO_PDDR_REG(Io_Pcs_FGPIO[Port]) | (1 << Pin);
	}

	else
	{
		FGPIO_PDDR_REG(Io_Pcs_FGPIO[Port]) = FGPIO_PDDR_REG(Io_Pcs_FGPIO[Port]) & ~(1 << Pin);
	}
}

/* FUNCTION NAME: Io_Pcs_GetPinDirection
 * RETURN TYPE: uint8
 * PARAMETERS: uint16 PortPin
 * DESCRIPTION:  Indicates direction of a pin.
 * OBSERVATIONS: -
 *  FURTHER
 * IMPROVEMENTS: 1) The pin function is verified and:
 * 				 - if GPIO: return direction
 * 				 - if other: return default value / return error
 */
uint8 Io_Pcs_GetPinDirection(uint16 PortPin)
{
	uint8 PinDirection;
	uint8 Port;
	uint8 Pin;

	Port = IO_PCS_GET_PORT(PortPin);
	Pin  = IO_PCS_GET_PIN(PortPin);

	PinDirection = (uint8)(Io_Func_ShrU32((FGPIO_PDDR_REG(Io_Pcs_FGPIO[Port]) & Io_Func_ShlU32(1, Pin)), Pin));

	return PinDirection;
}

/* FUNCTION NAME: Io_Pcs_GetPinInfo
 * RETURN TYPE: uint32
 * PARAMETERS: uint16 PortPin, uint8 detail
 * DESCRIPTION: Indicates the required information for a given pin.
 * OBSERVATIONS: -
 */
uint32 Io_Pcs_GetPinInfo(uint16 PortPin, uint8 detail)
{
	uint32 retValue = 0;
	uint16 i;
	uint8 Port;
	uint8 Pin;
	const Io_Pcs_PortTypeStruct *tmp_ptr;
	tmp_ptr = Io_Pcs_CnfPtr->Io_Pcs_PinPtr;

	Port = IO_PCS_GET_PORT(PortPin);
	Pin  = IO_PCS_GET_PIN(PortPin);

	for (i=0; i <= Io_Pcs_CnfPtr->Nr_Of_Pins; i++)
	{
		if (Port==tmp_ptr->Port && Pin==tmp_ptr->Pin)
		{
			switch(detail)
			{

			case IO_PCS_PIN_MUXING_INFO:
			{
				retValue = (uint32)tmp_ptr->PinMuxing;
				break;
			}
			case IO_PCS_INIT_LEVEL_INFO:
			{
				retValue = (uint32)tmp_ptr->InitLevel;
				break;
			}
			case IO_PCS_DIRECTION_INFO:
			{
				retValue = (uint32)tmp_ptr->Direction;
				break;
			}
			case IO_PCS_INTERRUPT_INFO:
			{
				retValue = (uint32)tmp_ptr->InterruptType;
				break;
			}
			case IO_PCS_CONFIG_INFO:
			{
				retValue = (uint32)tmp_ptr->Cfg;
				break;
			}
			default:
			{ break; }

			}
		}
		tmp_ptr++;
	}

	return retValue;
}

/** end of External Interfaces implementation **/

/** Local Functions implementation **/

// None so far.

/** end of Local Functions implementation **/

/** Interrupts Handlers implementation **/

/* FUNCTION NAME: Io_Pcs_Portc_Portd_Isr
 * RETURN TYPE: void
 * PARAMETERS: void
 * DESCRIPTION: Interrupt handler for PORTC and PORTD interrupts
 * OBSERVATIONS: -
 */
void Io_Pcs_Portc_Portd_Isr(void)
{
	uint32 portIsfr = 0;

	// Interrupt was generated on PORT C
	if(PORT_ISFR_REG(PORTC) != IO_PCS_ISFR_NO_ISF_SET)
	{
		portIsfr = PORT_ISFR_REG(PORTC);

		/* TODO FLUERAN: PCS test usage */
		if(portIsfr & IO_PCS_PIN_4_MASK)
		{
			//Interrupt on pin 4

			PORT_ISFR_REG(PORTC) |= IO_PCS_PIN_4_MASK;
		}
		else {}
		/* end of to do */
	}
	else {}

	// Interrupt was generated on PORT D
	if(PORT_ISFR_REG(PORTD) != IO_PCS_ISFR_NO_ISF_SET)
	{
		portIsfr = PORT_ISFR_REG(PORTD);

		if(portIsfr & IO_PCS_PIN_6_MASK)
		{
			// Interrupt on pin 6

			PORT_ISFR_REG(PORTD) |= IO_PCS_PIN_6_MASK;
		}
		else {}

		if(portIsfr & IO_PCS_PIN_7_MASK)
		{
			// Interrupt on pin 7

			PORT_ISFR_REG(PORTD) |= IO_PCS_PIN_7_MASK;
		}
		else {}
	}
	else {}
}

/** end of Interrupts Handlers implementation **/

/** end of Functions implementation **/
