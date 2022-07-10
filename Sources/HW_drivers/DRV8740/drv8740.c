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

#include "MCU_drivers/Dio/io_dio.h"
#include "MCU_drivers/Int/io_int_cnf.h"
#include "MCU_drivers/Pcs/io_pcs_cnf.h"
#include "HW_drivers/DRV8740/drv8740.h"
#include "MCU_drivers/SPI/io_spi_cnf.h"
#include "MCU_drivers/Tpm/io_tpm_cnf.h"

uint16 drv_spi_read = 0;
uint16 drv_spi_write = 0;
uint8 drv_spi_flag = 0;

void DRV8740_INIT()
{

}
/* FUNCTION NAME: DRV_SPI_FORMAT
 * RETURN TYPE: void
 * PARAMETERS: uint8 rdwrite, uint8 address, uint16 data
 * DESCRIPTION: Formats the parameters into a 16 bit unsigned integer (according to the device's SPI protocol)
 * 			    to be sent to the driver through SPI.
 * OBSERVATIONS: -
 */
void DRV_SPI_FORMAT(uint8 rdwrite, uint8 address, uint16 data)
{
	drv_spi_write = 0;
	drv_spi_write |= (data & SPI_DATA_MASK);
	drv_spi_write |= rdwrite << SPI_READ_WRITE_SHIFT;
	drv_spi_write |= address << SPI_ADDRESS_SHIFT;
}
/* FUNCTION NAME: DRV_SPI_WRITE
 * RETURN TYPE: void
 * PARAMETERS: SPI_Type * spi, uint8 rdwrite, uint8 address, uint16 data
 * DESCRIPTION: Sends data to the driver's register (selected through the address parameter)
 * 				via SPI
 * OBSERVATIONS: -
 */
void DRV_SPI_WRITE(SPI_Type * spi, uint8 address, uint16 data)
{
	DRV_SPI_FORMAT(SPI_WRITE, address, data);
	Io_SPI_Master_Write16bit(spi, drv_spi_write);
}
/* FUNCTION NAME: DRV_SPI_READ
 * RETURN TYPE: void
 * PARAMETERS: SPI_Type * spi, uint8 rdwrite, uint8 address, uint16 verify_data
 * DESCRIPTION: Selects the register from where to read and compares the read value
 * 				with the verify_data parameter
 * OBSERVATIONS: The function also checks if the correct value was sent to the
 * 				 slave device and handles both situations
 */
void DRV_SPI_READ(SPI_Type * spi, uint8 address, uint16 verify_data)
{
	DRV_SPI_FORMAT(SPI_READ, address, SPI_NULL);
	Io_SPI_Write_Read16bit(spi, drv_spi_write, &drv_spi_read);
	drv_spi_read &= SPI_DATA_MASK; //only the bits from 0 to 11 are relevant data
	if(verify_data == drv_spi_read)
	{
		Io_Dio_SetPinLevel(SET_LED_GREEN,LED_ON);
	}
	else
	{
		Io_Dio_SetPinLevel(SET_LED_GREEN,LED_OFF);
	}
}
