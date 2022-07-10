/****************************************************************************

COPYRIGHT (C) $Date: Nov 3, 2015 $
$CompanyInfo: CONTINENTAL AUTOMOTIVE GMBH $
ALL RIGHTS RESERVED.

The reproduction, transmission or use of this document or its contents is
not permitted without express written authority.
Offenders will be liable for damages. All rights, including rights created
by patent grant or registration of a utility model or design, are reserved.
---------------------------------------------------------------------------

$ProjectName: FRDM KL26-Z PLATFORM $

$Log: io_sch_cnf.c  $

$Author: PES contribution $

 ****************************************************************************/

#include "System/Os/io_os.h"
#include "System/Scheduler/io_sch_cnf.h"

#include "MCU_drivers/Adc/io_adc_cnf.h"
#include "MCU_drivers/Asy/io_asy.h"
#include "MCU_drivers/Dio/io_dio.h"
#include "MCU_drivers/I2c/io_i2c_cnf.h"
#include "MCU_drivers/Int/io_int_cnf.h"
#include "MCU_drivers/Pcs/io_pcs_cnf.h"
#include "MCU_drivers/Timer/io_tim.h"
#include "MCU_drivers/Tpm/io_tpm_cnf.h"
#include "MCU_drivers/Wdt/io_wdt.h"

#include <stdio.h>

#include "HW_drivers/H-bridge/ti8833/io_hbr_cnf_ti8833.h"
#include "HW_drivers/Sensors/Proximity/io_sens_ir.h"
#include "HW_drivers/Sensors/Accel_Magnet/FX0S8700.h"
#include "HW_drivers/Stepper_motor/io_smc_cnf.h"

#include "io_sch_cnf.h"
#include"SB_Functions/SB_Main.h"


static void Io_Sch_PrjHookTaskBackground(void);
static void Io_Sch_PrjHookTask1ms(void);
static void Io_Sch_PrjHookTask10ms(void);
static void Io_Sch_PrjHookTask50ms(void);
static void Io_Sch_PrjHookTask100ms(void);
static void Io_Sch_PrjHookTask1000ms(void);

uint32 Io_Sch_TaskCounter0ms;
uint32 Io_Sch_TaskCounter1ms;
uint32 Io_Sch_TaskCounter10ms;
uint32 Io_Sch_TaskCounter50ms;
uint32 Io_Sch_TaskCounter100ms;
uint32 Io_Sch_TaskCounter1000ms;

//---------------------------------------------------


const Io_Sch_OsConfigTaskStruct Io_Sch_OsTasks[]=
{
		/*  Period         Function           */
		{    0, Io_Sch_PrjHookTaskBackground},
		{    1, Io_Sch_PrjHookTask1ms},
		{   10, Io_Sch_PrjHookTask10ms},
		{   50, Io_Sch_PrjHookTask50ms},
		{  100, Io_Sch_PrjHookTask100ms},
		{ 1000, Io_Sch_PrjHookTask1000ms},
};

const Io_Sch_OsConfigStruct Io_Sch_OsConfig[] =
{
		{
				sizeof(Io_Sch_OsTasks)/sizeof(Io_Sch_OsConfigTaskStruct),
				(Io_Sch_OsConfigTaskStruct *) Io_Sch_OsTasks
		}
};


void Io_Sch_PrjHookTaskBackground(void)
{
	Io_Sch_TaskCounter0ms++;
	Io_Os_PriorityHandler();

}

void Io_Sch_PrjHookTask1ms(void)
{
	Io_Sch_TaskCounter1ms++;
	Io_Os_PeriodicHandler();
	SB_RunningTask1ms();
}

void Io_Sch_PrjHookTask10ms(void)
{
	Io_Sch_TaskCounter10ms++;

	/* Call the Watchdog periodic service */
	//Io_Wdt_Service();

	/* Read data from all the ADC channels */
	SB_RunningTask10ms();
	/* Call the SH_functions needed at 10msTask.*/
}

void Io_Sch_PrjHookTask50ms(void)
{

	Io_Sch_TaskCounter50ms++;
	SB_RunningTask50ms();
}


void Io_Sch_PrjHookTask100ms(void)
{
	Io_Sch_TaskCounter100ms++;
	SB_RunningTask100ms();
}
uint32 get_millis()
{
	return Io_Sch_TaskCounter1ms;
}

void Io_Sch_PrjHookTask1000ms(void)
{
	Io_Sch_TaskCounter1000ms++;
	SB_RunningTask1000ms();
}

uint32 get_seconds()
{
	return Io_Sch_TaskCounter1000ms;
}

