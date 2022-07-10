/**************************************************************************

COPYRIGHT (C) $Date: Mar 1, 2017 $
$CompanyInfo: CONTINENTAL AUTOMOTIVE GMBH $
ALL RIGHTS RESERVED.

The reproduction, transmission or use of this document or its contents is
not permitted without express written authority.
Offenders will be liable for damages. All rights, including rights created
by patent grant or registration of a utility model or design, are reserved.
---------------------------------------------------------------------------

$ProjectName: FRDM KL26-Z PLATFORM $

$Log: io_hbr_ti8833.c $

$Author: Adrian Cotuna $

 ****************************************************************************/

#ifndef SOURCES_HW_DRIVERS_DRV8740_DRV8740_H_
#define SOURCES_HW_DRIVERS_DRV8740_DRV8740_H_
#include "MKL26Z4.h"
#include "Platform_Types/std_types.h"
#include "Platform_Types/powersar_addon_types.h"
#include "Platform_Types/platform_types.h"
#include "Other_functions/io_func.h"

#define SPI_READ 1
#define SPI_WRITE 0

#define SPI_DATA_MASK 4095
#define SPI_READ_WRITE_SHIFT 15
#define SPI_ADDRESS_SHIFT 12
#define SPI_NULL 0
#define CTRL_Register 0x00
#define TORQUE_Register 0x01
#define OFF_Register 0x02
#define BLANK_Register 0x03
#define DECAY_Register 0x04
#define DRIVE_Register 0x06
#define STATUS_Register 0x07
#define SET_LED_GREEN			CNF_IO_PCS_P0E_31
extern uint16 drv_spi_read; //contains data read from the driver
extern uint16 drv_spi_write; //contains data to be written into the driver's registers
extern uint8 drv_spi_flag;

extern void DRV8740_INIT();
extern void DRV_SPI_FORMAT(uint8 rdwrite, uint8 address, uint16 data);
extern void DRV_SPI_WRITE(SPI_Type * spi, uint8 address, uint16 data);
extern void DRV_SPI_READ(SPI_Type * spi, uint8 address, uint16 verify_data);
#endif /* SOURCES_HW_DRIVERS_DRV8740_DRV8740_H_ */
