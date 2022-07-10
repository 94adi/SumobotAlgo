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

$Log: io_pcs_cnf.c $

$Author: Flueran Gabriel $

 ****************************************************************************/

/** Includes **/

#include "io_pcs_cnf.h"

/** end of Includes **/

/** External Configuration structures **/

	/* NAME: PORTs & PINs configuration structure array */
const Io_Pcs_PortTypeStruct Io_Pcs_PortCnf[] =
{
/* DESCRIPTION OF STRUCTURE ELEMENTS:
* 1: Port.			 Possible values: CNF_IO_PCS_P0A_00 -:- CNF_IO_PCS_P0A_31, CNF_IO_PCS_P0B_00 -:- CNF_IO_PCS_P0B_31,
* 									  CNF_IO_PCS_P0C_00 -:- CNF_IO_PCS_P0C_31, CNF_IO_PCS_P0D_00 -:- CNF_IO_PCS_P0D_31,
* 									  CNF_IO_PCS_P0E_00 -:- CNF_IO_PCS_P0E_31.
* 2: Pin.		 	 Possible values: 0 -:- 31
* 3: Pin muxing. 	 Possible values: IO_PCS_MUX_DISABLE, IO_PCS_MUX_GPIO, IO_PCS_MUX_ALT2 -:- IO_PCS_MUX_ALT7
* 4: Initial level.  Possible values: IO_PCS_INIT_LOW, IO_PCS_INIT_HIGH
* 5: Pin direction.	 Possible values: IO_PCS_INPUT, IO_PCS_OUTPUT
* 6: Interrupt type. Possible values: IO_PCS_IRQC_DISABLED, IO_PCS_IRQC_INT_LOW_LEVEL, IO_PCS_IRQC_INT_HIGH_LEVEL,
* 									  IO_PCS_IRQC_INT_RIS_EDGE, IO_PCS_IRQC_INT_FALL_EDGE, IO_PCS_IRQC_INT_BOTH_EDGES
* 									  (see "io_pcs_cnf.h" for more values)
* 7: Configuration.  Possible values: CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE,
* 									  ...
* 									  CNF_IO_PCS_HIGH_DS__PF__SLOW_SLEW__PULL_UP
* 									  (see "io_pcs_cnf.h" for more values)
*/
/* Port,    	Pin,    Pin muxing,       Initial level,	Pin direction,	Interrupt type,			Configuration */
{IO_PCS_PORTA,  1,    IO_PCS_MUX_ALT3,    IO_PCS_INIT_LOW,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, /* PWM - ON TPM2_CH0 (HBR0_PWM) */
{IO_PCS_PORTA,  2,    IO_PCS_MUX_ALT3,    IO_PCS_INIT_LOW,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, /* PWM - ON TPM2_CH1 (HBR1_PWM) */
{IO_PCS_PORTC,  8,    IO_PCS_MUX_GPIO,    IO_PCS_INIT_LOW,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, /* GPIO OUT (8833 HBR0_IN2) */
{IO_PCS_PORTC,  9,    IO_PCS_MUX_GPIO,    IO_PCS_INIT_LOW,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, /* GPIO OUT (8833 HBR1_IN2) */
{IO_PCS_PORTC,  0,    IO_PCS_MUX_GPIO,    IO_PCS_INIT_LOW,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, /* GPIO OUT (HBR0_1_STBY) */
/*                                              UART PINS INITIALIZATION                                                                  */
{IO_PCS_PORTE,  0,    IO_PCS_MUX_ALT3,    IO_PCS_INIT_LOW,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, /* UART1 Tx */
{IO_PCS_PORTE,  1,    IO_PCS_MUX_ALT3,    IO_PCS_INIT_LOW,	IO_PCS_INPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, /* UART1 Rx */
/*                                              I2C PINS INITIALIZATION                                                                  */
{IO_PCS_PORTB,	0,	  IO_PCS_MUX_ALT2,    IO_PCS_INIT_LOW,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__PULL_UP}, /* I2C0_SCL */
{IO_PCS_PORTB,	1,	  IO_PCS_MUX_ALT2,    IO_PCS_INIT_LOW,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__PULL_UP}, /* I2C0_SDA */
{IO_PCS_PORTC,	10,	  IO_PCS_MUX_ALT2,    IO_PCS_INIT_LOW,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__PULL_UP}, /* I2C1_SCL */
{IO_PCS_PORTC,	11,	  IO_PCS_MUX_ALT2,    IO_PCS_INIT_LOW,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__PULL_UP}, /* I2C1_SDA */
/*                                              SPI PINS INITIALIZATION                                                                  */
{IO_PCS_PORTC,	4,	  IO_PCS_MUX_ALT2,    IO_PCS_INIT_LOW,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__PULL_UP}, /* SPI0_PCS */
{IO_PCS_PORTC,	5,	  IO_PCS_MUX_ALT2,    IO_PCS_INIT_LOW,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__PULL_UP}, /* SPI0_SCK */
{IO_PCS_PORTC,	6,	  IO_PCS_MUX_ALT2,    IO_PCS_INIT_LOW,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__PULL_UP}, /* SPI0_MOSI */
{IO_PCS_PORTC,	7,	  IO_PCS_MUX_ALT2,    IO_PCS_INIT_LOW,	IO_PCS_INPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__PULL_UP}, /* SPI0_MISO */
/* 													ADC PINS																						*/
{IO_PCS_PORTE,  20,   IO_PCS_MUX_DISABLE, IO_PCS_INIT_LOW,	IO_PCS_INPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, // #2 ADC - CH0
{IO_PCS_PORTE,  21,   IO_PCS_MUX_DISABLE, IO_PCS_INIT_LOW,	IO_PCS_INPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, //#0 ADC - CH4a
{IO_PCS_PORTE,  22,   IO_PCS_MUX_DISABLE, IO_PCS_INIT_LOW,	IO_PCS_INPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, // #3 ADC - CH3
{IO_PCS_PORTE,  23,   IO_PCS_MUX_DISABLE, IO_PCS_INIT_LOW,	IO_PCS_INPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, // #1 ADC- CH7a
{IO_PCS_PORTE,  30,   IO_PCS_MUX_DISABLE, IO_PCS_INIT_LOW,	IO_PCS_INPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, // #5 ADC - CH23
{IO_PCS_PORTB,  2,    IO_PCS_MUX_DISABLE, IO_PCS_INIT_LOW,	IO_PCS_INPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, // #9 ADC - CH12
{IO_PCS_PORTB,  3,    IO_PCS_MUX_DISABLE, IO_PCS_INIT_LOW,	IO_PCS_INPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, // #10 ADC - CH13
{IO_PCS_PORTC,  2,    IO_PCS_MUX_DISABLE, IO_PCS_INIT_LOW,	IO_PCS_INPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, // #8 ADC - CH11
/*													MOTOR PWM PINS																						*/
{IO_PCS_PORTA,  4,    IO_PCS_MUX_ALT3,    IO_PCS_INIT_LOW,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, //PWM1_A
{IO_PCS_PORTA,  5,    IO_PCS_MUX_ALT3,    IO_PCS_INIT_LOW,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, //PWM2_B
{IO_PCS_PORTA,  12,   IO_PCS_MUX_ALT3,    IO_PCS_INIT_LOW,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, //PWM1_A
{IO_PCS_PORTA,  13,   IO_PCS_MUX_ALT3,    IO_PCS_INIT_LOW,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE}, //PWM2_B
/*													IR 0-15CM SENSOR PINS																				*/
{IO_PCS_PORTC,  1,	  IO_PCS_MUX_GPIO,    IO_PCS_INIT_LOW,	IO_PCS_INPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE},
{IO_PCS_PORTD,  5,	  IO_PCS_MUX_GPIO,    IO_PCS_INIT_LOW,	IO_PCS_INPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE},
{IO_PCS_PORTD,  6,	  IO_PCS_MUX_GPIO,    IO_PCS_INIT_LOW,	IO_PCS_INPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE},
/*													OPTICAL ENCODER																						*/
{IO_PCS_PORTB,  18,	  IO_PCS_MUX_ALT3,    IO_PCS_INIT_LOW,	IO_PCS_INPUT,	IO_PCS_IRQC_INT_RIS_EDGE,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__PULL_DOWN},
{IO_PCS_PORTB,  19,	  IO_PCS_MUX_ALT3,    IO_PCS_INIT_LOW,	IO_PCS_INPUT,	IO_PCS_IRQC_INT_RIS_EDGE,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__PULL_DOWN},
/*													GREEN LED FOR SIGNALING																				*/
{IO_PCS_PORTE,  31,	  IO_PCS_MUX_GPIO,    IO_PCS_INIT_HIGH,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE},
{IO_PCS_PORTD,  5,	  IO_PCS_MUX_GPIO,    IO_PCS_INIT_HIGH,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE},
{IO_PCS_PORTE,  29,	  IO_PCS_MUX_GPIO,    IO_PCS_INIT_HIGH,	IO_PCS_OUTPUT,	IO_PCS_IRQC_DISABLED,	CNF_IO_PCS_LOW_DS__NO_PF__FAST_SLEW__NO_PE},
};
	/* end of PORTs & PINs configuration structure array */
//PTE29
	/* NAME: PCS configuration structure array
	 * DESCRIPTION: Array to be passed as parameter to the Io_Pcs_Initialization() interface.
	 * NOTE: If configuration arrays names are kept the same, please do not alter the content of the following structure !
	 */
const Io_Pcs_CnfTypeStruct Io_Pcs_Cnf[]=
{
		{
				sizeof(Io_Pcs_PortCnf)/sizeof(Io_Pcs_PortTypeStruct),
				(const Io_Pcs_PortTypeStruct*)Io_Pcs_PortCnf
		}
};
	/* end of PCS configuration structure array */

/** end of External Configuration structures **/




