/****************************************************************************

COPYRIGHT (C) $Date: Feb 8, 2017 $
$CompanyInfo: CONTINENTAL AUTOMOTIVE GMBH $
ALL RIGHTS RESERVED.

The reproduction, transmission or use of this document or its contents is
not permitted without express written authority.
Offenders will be liable for damages. All rights, including rights created
by patent grant or registration of a utility model or design, are reserved.
---------------------------------------------------------------------------

$ProjectName: FRDM KL26-Z PLATFORM $

$Log: io_spi_cnf.h  $

$Author: Cotuna Adrian $

****************************************************************************/

#ifndef SOURCES_MCU_DRIVERS_SPI_IO_SPI_CNF_H_
#define SOURCES_MCU_DRIVERS_SPI_IO_SPI_CNF_H_

#include "io_spi.h"
#include "MKL26Z4.h"
/*                       REGISTER DEFINES                     */
/*************** SPI Baud Rate Register defines ***************/
#define BaudRate_Prescaler_div_1 0b000
#define BaudRate_Prescaler_div_2 0b001
#define BaudRate_Prescaler_div_3 0b010
#define BaudRate_Prescaler_div_4 0b011
#define BaudRate_Prescaler_div_5 0b100
#define BaudRate_Prescaler_div_6 0b101
#define BaudRate_Prescaler_div_7 0b110
#define BaudRate_Prescaler_div_8 0b111

#define BaudRate_Divisor_2 0b0000
#define BaudRate_Divisor_4 0b0001
#define BaudRate_Divisor_8 0b0010
#define BaudRate_Divisor_16 0b0011
#define BaudRate_Divisor_32 0b0100
#define BaudRate_Divisor_64 0b0101
#define BaudRate_Divisor_128 0b0110
#define BaudRate_Divisor_256 0b0111
#define BaudRate_Divisor_512 0b1000
/*********END OF SPI Baud Rate Register defines ***************/

/*************** SPI Control Register 2 defines ***************/
#define SPI_8bit 0
#define SPI_16bit 1
/*********END OF SPI Control Register 2 defines ***************/

/*************** SPI Control Register 1 defines ***************/
#define CPOL_ACTIVE_HIGH 0
#define CPOL_ACTIVE_LOW 1
#define CPHA_MIDDLE 0
#define CPHA_START 1
#define MSB_FIRST 0
#define LSB_FIRST 1
/*********END OF SPI Control Register 1 defines ***************/
/*                 END OF REGISTER DEFINES                    */

extern const Io_SPI_CnfTypeStruct Io_SPI_Cnf[];

#endif /* SOURCES_MCU_DRIVERS_SPI_IO_SPI_CNF_H_ */
