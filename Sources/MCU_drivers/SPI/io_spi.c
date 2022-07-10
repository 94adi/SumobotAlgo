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

$Log: io_spi.c  $

$Author: Cotuna Adrian $

****************************************************************************/

#include "io_spi.h"
#include "io_spi_cnf.h"
const Io_SPI_DeviceStruct *Io_DeviceCnf_Ptr1;

void Io_SPI_Init(const Io_SPI_CnfTypeStruct *Io_SPI_Cnf)
{
	int i;
	uint8 master_slave;
	uint8 baudrate_prescaler;
	uint8 baudrate_divider;
	uint8 SPImode;
	uint8 clock_polarity;
	uint8 clock_phase;
	uint8 MSB_LSB_first;
	SPI_Type *spi;
	Io_DeviceCnf_Ptr1 = Io_SPI_Cnf->Io_SPI_CnfPtr; //here are the selected settings for each SPI module
	uint8 low_byte;
	uint8 high_byte;
	/* initialize the selected SPI modules*/

	for(i=0;i<Io_SPI_Cnf->noOfSPI;i++)
	{
		spi = Io_DeviceCnf_Ptr1->SPI;
		master_slave = Io_DeviceCnf_Ptr1->master_slave;
		baudrate_prescaler = Io_DeviceCnf_Ptr1->baudrate_prescaler;
		baudrate_divider = Io_DeviceCnf_Ptr1->baudrate_divider;
		SPImode = Io_DeviceCnf_Ptr1->SPImode;
		clock_polarity = Io_DeviceCnf_Ptr1->clock_polarity;
		clock_phase = Io_DeviceCnf_Ptr1->clock_phase;
		MSB_LSB_first = Io_DeviceCnf_Ptr1->MSB_LSB_first;
		if(spi == SPI0)
		{
			SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK; //enable clock source for SPI0
			SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; //for SPI related pins
			SPI_BR_REG(spi) |= SPI_BR_SPR(baudrate_divider) | SPI_BR_SPPR(baudrate_prescaler); //baud rate
			SPI_C2_REG(spi) |= SPI_C2_SPIMODE(SPImode) | SPI_C2_MODFEN(ENABLE) | SPI_C2_BIDIROE(DISABLE) | SPI_C2_SPC0(DISABLE);
			SPI_C1_REG(spi) |= SPI_C1_CPHA(clock_phase) | SPI_C1_CPOL(clock_polarity) | SPI_C1_LSBFE(MSB_LSB_first) | SPI_C1_SPE(ENABLE) | SPI_C1_MSTR(master_slave) | SPI_C1_SSOE(ENABLE);
			/* Dummy Read/Write in order to generate CLK signal*/
			SPI0->DH = 0;
			SPI0->DL = 0;
			low_byte = SPI0->DL;
			high_byte = SPI0->DH;
		}
		else if(spi == SPI1)
		{
			SIM_SCGC4 |= SIM_SCGC4_SPI1_MASK; //enable clock source for SPI1
			//SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; //for SPI related pins
			SPI_BR_REG(spi) |= SPI_BR_SPR(baudrate_divider) | SPI_BR_SPPR(baudrate_prescaler); //baud rate
			SPI_C2_REG(spi) |= SPI_C2_SPIMODE(SPImode) | SPI_C2_MODFEN(ENABLE) | SPI_C2_BIDIROE(DISABLE) | SPI_C2_SPC0(DISABLE);
			SPI_C1_REG(spi) |= SPI_C1_CPHA(clock_phase) | SPI_C1_CPOL(clock_polarity) | SPI_C1_LSBFE(MSB_LSB_first) | SPI_C1_SPE(ENABLE) | SPI_C1_MSTR(master_slave) | SPI_C1_SSOE(ENABLE);
			SPI1->DH = 0;
			SPI1->DL = 0;
			low_byte = SPI1->DL;
			high_byte = SPI1->DH;
		}
		Io_DeviceCnf_Ptr1++;
	}
}
void Io_SPI_Write(SPI_Type * spi, uint8 data)
{
	Io_SPI_SlaveSelect_Enable();
	uint8 dummy = 0;
	Io_SPI_Write_Wait(spi);
	SPI_DL_REG(spi) = data;
	Io_SPI_Read_Wait(spi);
	dummy = SPI_DL_REG(spi);
}
void Io_SPI_Write16bit(SPI_Type * spi, uint16 data)
{
	Io_SPI_SlaveSelect_Enable();
	uint8 dummy_low;
	uint8 dummy_high;
	Io_SPI_Write_Wait(spi);
	SPI_DL_REG(spi) = (uint8)(data & 0xFF); //low byte
	SPI_DH_REG(spi) = (uint8)((data >> 8) & 0xFF);//high byte
	//dummy read
	Io_SPI_Read_Wait(spi);
	dummy_low = SPI_DL_REG(spi);
	dummy_high = SPI_DH_REG(spi);
}

void Io_SPI_Write_Read16bit(SPI_Type * spi, uint16 data_in, uint16 * data_out)
{
	Io_SPI_SlaveSelect_Enable();
	uint8 data_low = 0;
	uint8 data_high = 0;
	Io_SPI_Write_Wait(spi);
	SPI_DL_REG(spi) = (uint8)(data_in & 0xFF); //low byte
	SPI_DH_REG(spi) = (uint8)((data_in >> 8) & 0xFF);//high byte
	//Write process completed
	Io_SPI_Read_Wait(spi); //waiting for the byte/bytes to transfer
	//once all data was sent to the slave the SPI data register is ready to be read
	//reading SPI data registers
	data_low = SPI_DL_REG(spi);
	data_high = SPI_DH_REG(spi);
	//storing it into a 16bit integer
	*data_out = (uint16)(data_high << 8);
	*data_out |= data_low;
	//*data_out = Read_Address(*data_out);
}
void Io_SPI_Write_Read(SPI_Type * spi, uint8 data_in, uint8 * data_out)
{
	uint8 data = 0;
	Io_SPI_Write_Wait(spi);
	SPI_DL_REG(spi) = data_in; //low byte
	Io_SPI_Read_Wait(spi);
	data = SPI_DL_REG(spi);
	*data_out = data;
}

void Io_SPI_Read16bit(SPI_Type * spi, uint16 * data_out)
{
		uint8 data_low;
		uint8 data_high;
		//DUMMY WRITE FROM SLAVE TO MASTER
		Io_SPI_Write_Wait(spi);
		SPI_DL_REG(spi) = 0;
		SPI_DH_REG(spi) = 0;
		Io_SPI_Read_Wait(spi); //waiting for the byte/bytes to transfer
		//once all data was sent to the slave the SPI data register is ready to be read
		//reading SPI data registers
		data_low = SPI_DL_REG(spi);
		data_high = SPI_DH_REG(spi);
		//storing it into a 16bit integer
		*data_out = (uint16)(data_high << 8);
		*data_out |= data_low;
}

void Io_SPI_Read(SPI_Type * spi, uint8 * data_out)
{
		uint8 data;
		//DUMMY WRITE FROM SLAVE TO MASTER
		Io_SPI_Write_Wait(spi);
		SPI_DL_REG(spi) = 0;
		Io_SPI_Read_Wait(spi); //waiting for the byte/bytes to transfer
		//once all data was sent to the slave the SPI data register is ready to be read
		//reading SPI data registers
		data = SPI_DL_REG(spi);
		//storing it into a 16bit integer
		*data_out = data;
}

void Io_SPI_Sistem(SPI_Type * spi, uint8 state)
{
	SPI_C1_REG(spi) |= SPI_C1_SPE(state);
}
