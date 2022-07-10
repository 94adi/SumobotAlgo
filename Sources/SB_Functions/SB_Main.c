#include"SB_Main.h"
#include "System/Os/io_os.h"
#include "System/Scheduler/io_sch_cnf.h"

#include "MKL26Z4.h"
#include "Platform_Types/std_types.h"
#include "Platform_Types/powersar_addon_types.h"
#include "Platform_Types/platform_types.h"
#include <stdio.h>

#include "Other_functions/io_func.h"
#include "HW_drivers/Sensors/Time_of_Flight/TOF.h"
#include "MCU_drivers/Adc/io_adc_cnf.h"
#include "MCU_drivers/Adc/io_adc.h"
#include "MCU_drivers/Asy/io_asy.h"
#include "MCU_drivers/Dio/io_dio.h"
#include "MCU_drivers/I2c/io_i2c_cnf.h"
#include "MCU_drivers/Int/io_int_cnf.h"
#include "MCU_drivers/Pcs/io_pcs_cnf.h"
#include "MCU_drivers/Timer/io_tim.h"
#include "MCU_drivers/Tpm/io_tpm_cnf.h"
#include "MCU_drivers/Tpm/io_tpm.h"
#include "MCU_drivers/Wdt/io_wdt.h"
#include "HW_drivers/Sensors/Accel_Magnet/FX0S8700.h"
#include "HW_drivers/DRV8740/drv8740.h"
#include "HW_drivers/Bluetooth/Bluetooth.h"
#include "SB_Main.h"
#include "MCU_drivers/SPI/io_spi_cnf.h"
#include <stdbool.h>
#include <math.h>
#include "MCU_drivers/Car_movement/app_robo_car_movement.h"

volatile uint32 system_miliseconds;

uint16 val0;
uint16 val1;
uint16 val2;
uint16 val3;
uint16 val4;
uint16 val5;
uint16 val6;
uint16 val7;
uint16 val8;


uint8 pwm_counter = 0;
uint8 pwm_slave = 0;

uint8 current_state = 0;
uint8 previous_state = 0;
uint8 increment = 0;
uint8 toggle_pin = 0;
uint8 increment_b = 0;
uint16 rez;

void SB_PWMSetPercentDuty_cycle(uint32 channel,uint8 dutycyclePercentage)
{
	uint16 dutycycle;
	dutycycle = (DUTYCYCLE_MAX_VALUE * dutycyclePercentage) / 100;
	Io_Tpm_PwmChangeDutycycle(channel,dutycycle);
}


uint16 SB_ReadAnalog(char channel)
{
	uint16 adc_value;
	adc_value = Io_Adc_GetResult(channel);
	return adc_value;
}

uint16 SB_ReadAnalog_mV(char channel)
{
	//in this case channel is 0
	uint16 result;
	result = (Io_Adc_GetResult(channel) * VREF_mV) / MAX_VALUE_15BITS;
	return result;
}

uint8 SB_ReanAnalog_Percent(char channel)
{
	uint16 adc_value;
	uint8 adc_percent;
	adc_value = SB_ReadAnalog(channel);
	adc_percent = (100 * adc_value) / MAX_VALUE_15BITS;
	return adc_percent;
}


void SB_RunningTask1000ms()
{

}

void SB_RunningTask100ms()
{
	rez = readRangeContinuousMillimeters();
	//rez = SB_ReadAnalog_mV(3);
	if(LED_STATE == BT_GREEN){
		Io_Dio_SetPinLevel(SET_LED_GREEN,LED_HIGH);
		Io_Dio_SetPinLevel(SET_LED_RED,LED_LOW);
		Io_Dio_SetPinLevel(SET_LED_BLUE,LED_LOW);
		Algo_Robo_Cm_BreakCar();

	}
	if(LED_STATE == BT_RED){
		Io_Dio_SetPinLevel(SET_LED_RED,LED_HIGH);
		Io_Dio_SetPinLevel(SET_LED_GREEN,LED_LOW);
		Io_Dio_SetPinLevel(SET_LED_BLUE,LED_LOW);
		Algo_Robo_Cm_MoveCarForward(50);
	}
	if(LED_STATE == BT_BLUE){
		Io_Dio_SetPinLevel(SET_LED_BLUE,LED_HIGH);
		Io_Dio_SetPinLevel(SET_LED_RED,LED_LOW);
		Io_Dio_SetPinLevel(SET_LED_GREEN,LED_LOW);
		Algo_Robo_Cm_MoveCarBackward(50);
	}
	//Test_DRV();
	//rez = readRangeContinuousMillimeters();
	//if(rez > 0 && rez < 100)
	//SB_PWMSetPercentDuty_cycle(GREEN_TEST,rez);
	//else
	//SB_PWMSetPercentDuty_cycle(GREEN_TEST,1);
	/*val0 = SB_ReanAnalog_Percent(0);
	val1 = SB_ReanAnalog_Percent(1);
	val2 = SB_ReanAnalog_Percent(2);
	val3 = SB_ReanAnalog_Percent(3);
	val4 = SB_ReanAnalog_Percent(4);
	val5 = SB_ReanAnalog_Percent(5);
	val6 = SB_ReanAnalog_Percent(6);
	val7 = SB_ReanAnalog_Percent(7);
	val8 = SB_ReanAnalog_Percent(8);*/
	//SB_PWMSetPercentDuty_cycle(GREEN_LED,100-val0);
	/*Master code*/
	/*pwm_counter += 1;
	if(pwm_counter > 99)
		pwm_counter=0;
	Io_SPI_Write16bit(SPI0, pwm_counter);
	SB_PWMSetPercentDuty_cycle(GREEN_LED,100-pwm_counter);*/
	/*Slave code*/
	//Io_SPI_Read16bit(SPI0, &pwm_slave);
	//SB_PWMSetPercentDuty_cycle(GREEN_LED,pwm_slave);
}

void SB_RunningTask50ms()
{
	bluetooth_state();
}

void SB_RunningTask10ms()
{
	Io_Adc_Autoscan();

	/*Io_Dio_SetPinLevel(CNF_IO_PCS_P0C_00,HIGH);
	SB_PWMSetPercentDuty_cycle(CNF_TPM2_CHANNEL0,50);
	Io_Dio_SetPinLevel(CNF_IO_PCS_P0C_08,HIGH);
	SB_PWMSetPercentDuty_cycle(CNF_TPM2_CHANNEL1,50);
	Io_Dio_SetPinLevel(CNF_IO_PCS_P0C_09,HIGH);*/

	/*if(rez < 100 && rez > 0)
	{
		SB_PWMSetPercentDuty_cycle(GREEN_LED,100-rez);
	}
	else
	{
		SB_PWMSetPercentDuty_cycle(GREEN_LED,5);
	}/*
	/*increment_b++;
	if(increment_b == 200){
		increment_b = 10;
	}
	else if(increment_b < 7){
	if(toggle_pin == 0){
		toggle_pin = 1;
		Io_Dio_SetPinLevel(ENC_B,PIN_LOW);
	}
	else if(toggle_pin == 1){
		toggle_pin = 0;
		Io_Dio_SetPinLevel(ENC_B,PIN_HIGH);
	}
	}*/
}

void SB_RunningTask1ms()
{
	system_miliseconds++;
	/*current_state = Io_Dio_GetPinLevel(ENC_A);
	if((previous_state == PIN_LOW) && (current_state == PIN_HIGH))
	{
		increment++;
	}
	previous_state = current_state;*/
}

void SB_InitVariables()
{
	system_miliseconds = 0;
}

void Test_DRV()
{
		//DRV_SPI_WRITE(SPI0,0x06,4005);
		//DRV_SPI_READ(SPI0,0x06,4005);
		//Io_SPI_Read16bit(SPI0, &data);

}
uint32 SB_get_milis()
{
	return system_miliseconds;
}
