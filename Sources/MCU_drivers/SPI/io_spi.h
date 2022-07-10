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

$Log: io_spi.h  $

$Author: Cotuna Adrian $

****************************************************************************/

#ifndef SOURCES_MCU_DRIVERS_SPI_IO_SPI_H_
#define SOURCES_MCU_DRIVERS_SPI_IO_SPI_H_

#include "MCU_drivers/Dio/io_dio.h"
#include "MCU_drivers/Pcs/io_pcs.h"
#include "MCU_drivers/Pcs/io_pcs_cnf.h"
#include "MKL26Z4.h"



#define SPI_READ 1
#define SPI_WRITE 0
#define MSB_16bit(data) (data << 15)
#define Read_Address(a) (a & 4095) //a is type uint16
#define ENABLE 1
#define DISABLE 0
#define MASTER ENABLE
#define SLAVE DISABLE
#define SLAVE_SELECT_PIN 			CNF_IO_PCS_P0C_04


typedef struct Io_SPI_DeviceStruct_Tag Io_SPI_DeviceStruct;
typedef struct Io_SPI_CnfTypeStruct_Tag Io_SPI_CnfTypeStruct;

struct Io_SPI_DeviceStruct_Tag
{
	//to add DMA, INTERRUPTS (off default)
	SPI_Type * SPI;
	uint8 master_slave;
	uint8 baudrate_prescaler;
	uint8 baudrate_divider;
	uint8 SPImode;
	uint8 clock_polarity;
	uint8 clock_phase;
	uint8 MSB_LSB_first;
};

struct Io_SPI_CnfTypeStruct_Tag
{
	uint8 noOfSPI;
	const Io_SPI_DeviceStruct *Io_SPI_CnfPtr;
};

extern void Io_SPI_Init(const Io_SPI_CnfTypeStruct *Io_SPI_Cnf);
extern void Io_SPI_Write16bit(SPI_Type * spi, uint16 data);
extern void Io_SPI_Write(SPI_Type * spi, uint8 data);
extern void Io_SPI_Write_Read16bit(SPI_Type * spi, uint16 data_in, uint16 * data_out);
extern void Io_SPI_Write_Read(SPI_Type * spi, uint8 data_in, uint8 * data_out);
extern void Io_SPI_Read16bit(SPI_Type * spi, uint16 * data_out);
extern void Io_SPI_Read(SPI_Type * spi, uint8 * data_out);

static inline void Io_SPI_Write_Wait(SPI_Type * spi)
{
	while((SPI_S_REG(spi) & SPI_S_SPTEF_MASK) == 0) {}
}

static inline void Io_SPI_Read_Wait(SPI_Type * spi)
{
	while((SPI_S_REG(spi) & SPI_S_SPRF_MASK) == 0) {}
}

static inline void Io_SPI_SlaveSelect_Enable()
{
	if(Io_Dio_GetPinLevel(SLAVE_SELECT_PIN) == HIGH)
	Io_Dio_SetPinLevel(SLAVE_SELECT_PIN,LOW); //Slave connected through the pin is selected
}

static inline void Io_SPI_SlaveSelect_Disable()
{
	if(Io_Dio_GetPinLevel(SLAVE_SELECT_PIN) == LOW)
	Io_Dio_SetPinLevel(SLAVE_SELECT_PIN,HIGH); //Slave connected through the pin is deselected
}
#endif /* SOURCES_MCU_DRIVERS_SPI_IO_SPI_H_ */
