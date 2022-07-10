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

$Log: io_spi_cnf.c  $

$Author: Cotuna Adrian $

****************************************************************************/

#include "io_spi.h"
#include "io_spi_cnf.h"

const Io_SPI_DeviceStruct Io_SPI_CnfDevice[]=
{
		{
		SPI0 ,
		MASTER,
		BaudRate_Prescaler_div_8,
		BaudRate_Divisor_512,
		SPI_16bit,
		CPOL_ACTIVE_HIGH,
		CPHA_START,
		MSB_FIRST
		}
};

const Io_SPI_CnfTypeStruct Io_SPI_Cnf[]=
{
		{
		sizeof(Io_SPI_CnfDevice)/sizeof(Io_SPI_DeviceStruct), //number of SPI
		(const Io_SPI_DeviceStruct*) Io_SPI_CnfDevice //SPI settings
		}
};
