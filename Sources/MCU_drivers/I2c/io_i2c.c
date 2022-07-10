/****************************************************************************

COPYRIGHT (C) $Date: Nov 6, 2015 $
$CompanyInfo: CONTINENTAL AUTOMOTIVE GMBH $
ALL RIGHTS RESERVED.

The reproduction, transmission or use of this document or its contents is
not permitted without express written authority.
Offenders will be liable for damages. All rights, including rights created
by patent grant or registration of a utility model or design, are reserved.
---------------------------------------------------------------------------

$ProjectName: FRDM KL26-Z PLATFORM $

$Log: io_i2c.c  $

$Author: Leonte Alexandru $

****************************************************************************/

#include "io_i2c_cnf.h"

/* Local Functions */
void Pause(void);
static void Io_I2c_CycleWrite(I2C_Type * i2c, uint8 byte_out);
static uint8 Io_I2c_CycleRead(I2C_Type * i2c,uint8 ack);

/* End of Local Functions */


/* Local Variables */
const Io_I2c_CnfDeviceStruct *Io_DeviceCnf_Ptr;

uint8 timeout;
uint32 error_log;
/* End of local variables */


void Io_I2c_Init(const Io_I2c_CnfTypeStruct *Io_I2c_Cnf) // enable clock source for I2C and set baud rate
{
	/* Dummy variables */
	uint8 i;

	/* end of Dummy variables */

	/* I2C structure variables */

	I2C_Type *i2c;

	/* end of I2C structure variables */

	Io_DeviceCnf_Ptr = Io_I2c_Cnf->Io_I2c_CnfPtr;

	/* initialize the selected I2C modules*/

	for(i=0;i<Io_I2c_Cnf->noOfI2c;i++)
	{
		i2c = Io_DeviceCnf_Ptr->i2c;

		if(i2c == I2C0)
		{
			//enable clock source for I2C
			SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK; //set the 6th bit of the register
			SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; //already enabled

			// Configure GPIO for I2C function <- DONE

			//I2C Frequency Divider register (I2Cx_F)
			//Set baud rate. I2C0 is clocked from the bus clock

			i2c->F |= 0 | I2C_F_ICR(Io_DeviceCnf_Ptr->i2c_icr) | I2C_F_MULT(Io_DeviceCnf_Ptr->i2c_mult) ;			// SDA hold time = 2.125us, SCL start hold time = 4.25us, SCL stop hold time = 5.125us
			i2c->C1 = I2C_C1_IICEN_MASK;		 // Enables I2C module operation.
			//i2c->SMB |= 0b100000000;
			//i2c->A1 = 0b0101001;
		}
		else if(i2c == I2C1)
		{
			SIM_SCGC4 |= SIM_SCGC4_I2C1_MASK;
			SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	        //  Configure GPIO for I2C function

			// Board is missing pull-ups on both lines, so enable internal pull-ups in PCS

			// Set baud rate. I2C1 is clocked from the system clock
			i2c->F  = 0 | I2C_F_MULT(Io_DeviceCnf_Ptr->i2c_mult) | I2C_F_ICR(Io_DeviceCnf_Ptr->i2c_icr);
			// Enable IIC
			i2c->C1 = I2C_C1_IICEN_MASK;
		}
		Io_DeviceCnf_Ptr++;
	}
}

void Io_I2c_Deinit(I2C_Type * i2c)
{
	i2c->C1 = 0x00;

	if(i2c == I2C0)
	{
	SIM_SCGC4 &= ~SIM_SCGC4_I2C0_MASK;
	}
	else if(i2c == I2C1)
	{
		SIM_SCGC4 &= ~SIM_SCGC4_I2C1_MASK;
	}
}


uint8 Io_I2c_RegRead(I2C_Type * i2c, uint8 SlaveAddress, uint8 RegisterAddress)
{
	uint8 result=0;
	Io_Int_DisableInterrupts();
	i2c->C1 |= I2C_C1_TX_MASK;					//set tx mode (transmit)
	i2c_Start(i2c);				//set MASTER mode					  		// send start signal on the bus
	//select slave device and register to read from (Master - write mode)
	Io_I2c_CycleWrite(i2c,SlaveAddress | I2C_WRITE);
	Io_I2c_CycleWrite(i2c,RegisterAddress);
	i2c_RepeatedStart(i2c);									// Do a repeated start *
	Io_I2c_CycleWrite(i2c,SlaveAddress | I2C_READ); //Master - read mode
	result = Io_I2c_CycleRead(i2c,1); //dummy read
	result = Io_I2c_CycleRead(i2c,1);
	i2c_Stop(i2c);											  // Send stop since about to read last byte
	Io_Int_EnableInterrupts();
	return result;
}
uint16 Io_I2c_RegRead16bit(I2C_Type * i2c, uint8 SlaveAddress, uint8 RegisterAddress)
{
		uint16 result;
		Io_Int_DisableInterrupts();
		i2c->C1 |= I2C_C1_TX_MASK;					//set tx mode
		i2c_Start(i2c);				//set MASTER mode					  		// send start signal
		//select slave device and register to read from (Master - write mode)
		Io_I2c_CycleWrite(i2c,SlaveAddress | I2C_WRITE);
		Io_I2c_CycleWrite(i2c,RegisterAddress);
		i2c_RepeatedStart(i2c);// Do a repeated start
		Io_I2c_CycleWrite(i2c,SlaveAddress | I2C_READ); //Master - read mode
		result = (uint16)Io_I2c_CycleRead(i2c,0);
		result = 0;
		result = (uint16)Io_I2c_CycleRead(i2c,0) << 8; //high byte
		result |= Io_I2c_CycleRead(i2c,1); //low byte
		i2c_Stop(i2c);											  // Send stop since about to read last byte
		Io_Int_EnableInterrupts();
		return result;
}
void Io_I2c_RegReadN(I2C_Type * i2c, uint8 SlaveAddress,uint8 RegisterAddress, uint8 N, uint8 *array)
{
    uint8 tmp_byte;
    Io_Int_DisableInterrupts();
	i2c->C1 |= I2C_C1_TX_MASK;					//set tx mode
	i2c_Start(i2c);									  		// send start signal
	Io_I2c_CycleWrite(i2c, SlaveAddress);               // Send IIC "Write" Address
	Io_I2c_CycleWrite(i2c, RegisterAddress);                  // Send Register
	i2c_RepeatedStart(i2c);									// Do a repeated start *
    Io_I2c_CycleWrite(i2c, SlaveAddress | I2C_READ);    // Send IIC "Read" Address
    tmp_byte = Io_I2c_CycleRead(i2c, 0);                  // *** Dummy read: reads "IIC_ReadAddress" value ***
    while (N-- > 1) {                           // Read N-1 Register Values
    	tmp_byte = Io_I2c_CycleRead(i2c, 0);
        *array = tmp_byte; //
        array++; //next element
    }
    tmp_byte = Io_I2c_CycleRead(i2c, 1);
    *array = tmp_byte;                                 // Read Last value
	i2c_Stop(i2c);											  // Send stop since about to read last byte
    Io_Int_EnableInterrupts();
}
void Io_I2c_WriteRegN(I2C_Type * i2c, uint8 SlaveAddress, uint8 RegisterAddress, uint8 N, uint8 *array)
{
	Io_Int_DisableInterrupts();

		i2c->C1 |= I2C_C1_TX_MASK;					//set tx mode
	    i2c_Start(i2c);							    //start
		Io_I2c_CycleWrite(i2c,SlaveAddress | I2C_WRITE);
		Io_I2c_CycleWrite(i2c,RegisterAddress);
		while (N-- > 1)
		{
			Io_I2c_CycleWrite(i2c,*array);
			array++; //iterate to the next element 0 -> n-2
		}
		Io_I2c_CycleWrite(i2c,*array); // n-1
		i2c_Stop(i2c);											  // Send stop since about to read last byte
		Io_Int_EnableInterrupts();
}
static uint8 Io_I2c_CycleRead(I2C_Type * i2c, uint8 ack)
{
	uint8 byte_read;
	timeout = 0;
	while ((i2c->S & I2C_S_TCF_MASK) == 0 && (timeout < I2C_MAX_TIMEOUT)) //TCF - Transfer Complete Flag
		timeout++;
	if(timeout>I2C_MAX_TIMEOUT)
		error_log |= 0x08;

	i2c_set_rx_mode(i2c);									 // Put in Rx Mode
	if (ack)
		i2c_give_nack(i2c);										//Turn off ACK since this is second to last byte being read
	else
		i2c->C1 &= ~I2C_C1_TXAK_MASK;
	byte_read = i2c_read_byte(i2c);
	i2c_Wait(i2c);
	return byte_read;
}


uint8 Io_I2c_RegWrite(I2C_Type * i2c, uint8 SlaveAddress, uint8 RegisterAddress, uint8 data)
{
	Io_Int_DisableInterrupts();
	i2c->C1 |= I2C_C1_TX_MASK;					//set tx mode
    i2c_Start(i2c);											//start
	Io_I2c_CycleWrite(i2c,SlaveAddress | I2C_WRITE);
	Io_I2c_CycleWrite(i2c,RegisterAddress);
	Io_I2c_CycleWrite(i2c,data);
	i2c_Stop(i2c);											  // Send stop since about to read last byte
	Io_Int_EnableInterrupts();
	return 0;
}

uint8 Io_I2c_RegWrite2(I2C_Type * i2c, uint8 SlaveAddress, uint8 RegisterAddress, uint8 * data)
{
	Io_Int_DisableInterrupts();
	i2c->C1 |= I2C_C1_TX_MASK;					//set tx mode
    i2c_Start(i2c);											//start
	Io_I2c_CycleWrite(i2c,SlaveAddress | I2C_WRITE);
	Io_I2c_CycleWrite(i2c,RegisterAddress);
	Io_I2c_CycleWrite(i2c,*data);
	i2c_Stop(i2c);											  // Send stop since about to read last byte
	Io_Int_EnableInterrupts();
	return 0;
}

static void Io_I2c_CycleWrite(I2C_Type * i2c, uint8 byte_out)
{
    timeout = 0;
    //if there's an ongoing transfer of data
    //wait until it's finished
    while ((i2c->S & I2C_S_TCF_MASK) == 0 && (timeout < I2C_MAX_TIMEOUT))
        timeout++;
    if (timeout >= I2C_MAX_TIMEOUT)
        error_log |= 0x08;
	i2c_write_byte(i2c, byte_out);			// send ID with W/R bit
	i2c_Wait(i2c);											// wait
	i2c_get_ack(i2c);	//if true -> success if false -> fail
}

void Io_I2c_WriteReg16Bit(I2C_Type * i2c, uint8 SlaveAddress, uint8 RegisterAddress, uint16_t value)
{
		Io_Int_DisableInterrupts();
		i2c->C1 |= I2C_C1_TX_MASK;					//set tx mode
	    i2c_Start(i2c);											//start
		Io_I2c_CycleWrite(i2c,SlaveAddress | I2C_WRITE);
		Io_I2c_CycleWrite(i2c,RegisterAddress);
		Io_I2c_CycleWrite(i2c,(value >> 8) & 0xFF); // high byte
		Io_I2c_CycleWrite(i2c,value & 0xFF); // low byte
		i2c_Stop(i2c);											  // Send stop since about to read last byte
		Io_Int_EnableInterrupts();
}
void Io_I2c_WriteReg32Bit(I2C_Type * i2c, uint8 SlaveAddress, uint8 RegisterAddress, uint32_t value)
{
		Io_Int_DisableInterrupts();
		i2c->C1 |= I2C_C1_TX_MASK;					//set tx mode
	    i2c_Start(i2c);											//start
		Io_I2c_CycleWrite(i2c,SlaveAddress | I2C_WRITE);
		Io_I2c_CycleWrite(i2c,RegisterAddress);
		Io_I2c_CycleWrite(i2c,(value >> 24) & 0xFF);
		Io_I2c_CycleWrite(i2c,(value >> 16) & 0xFF);
		Io_I2c_CycleWrite(i2c,(value >> 8) & 0xFF);
		Io_I2c_CycleWrite(i2c,value & 0xFF);
		i2c_Stop(i2c);											  // Send stop since about to read last byte
		Io_Int_EnableInterrupts();
}
void Io_I2C0_Isr()
{
}
