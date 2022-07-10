/**************************************************************************

COPYRIGHT (C) $Date: Nov 3, 2015 $
$CompanyInfo: CONTINENTAL AUTOMOTIVE GMBH $
ALL RIGHTS RESERVED.

The reproduction, transmission or use of this document or its contents is
not permitted without express written authority.
Offenders will be liable for damages. All rights, including rights created
by patent grant or registration of a utility model or design, are reserved.
---------------------------------------------------------------------------

$ProjectName: FRDM KL26-Z PLATFORM $

$Log: main.c $

$Author: PES contribution $

 ****************************************************************************/

#include "System/Os/io_os.h"
#include "System/Scheduler/io_sch_cnf.h"
#include <stdio.h>

#include "MKL26Z4.h"
#include "Platform_Types/std_types.h"
#include "Platform_Types/powersar_addon_types.h"
#include "Platform_Types/platform_types.h"

#include "Other_functions/io_func.h"

#include "MCU_drivers/Adc/io_adc_cnf.h"
#include "MCU_drivers/Asy/io_asy.h"
#include "MCU_drivers/Dio/io_dio.h"
#include "MCU_drivers/I2c/io_i2c_cnf.h"
#include "MCU_drivers/I2c/io_i2c.h"
#include "MCU_drivers/SPI/io_spi.h"
#include "MCU_drivers/SPI/io_spi_cnf.h"
#include "MCU_drivers/Int/io_int_cnf.h"
#include "MCU_drivers/Pcs/io_pcs_cnf.h"
#include "MCU_drivers/Timer/io_tim.h"
#include "MCU_drivers/Tpm/io_tpm_cnf.h"
#include "MCU_drivers/Tpm/io_tpm.h"
#include "MCU_drivers/Wdt/io_wdt.h"

#include"SB_Functions/SB_Main.h"
#include"HW_drivers/Sensors/Accel_Magnet/FX0S8700.h"
#include "HW_drivers/H-bridge/ti8833/io_hbr_cnf_ti8833.h"
#include "HW_drivers/Sensors/Proximity/io_sens_ir.h"
#include "HW_drivers/Stepper_motor/io_smc_cnf.h"
#include "HW_drivers/Sensors/Time_of_Flight/TOF.h"


int main(void)
{
	SIM_COPC = SIM_COPC_COPT(0);
	/* Temporary disable all interrupts, until all initialization interfaces have been called */
 	Io_Int_DisableInterrupts();

    /** Initialization calls for MCU Drivers **/

	/* Call the initialization interfaces for INT (Interrupts) driver */
	Io_Int_Initialization(Io_Int_Cnf);

	/* Call the initialization interfaces for PCS (Port Control) driver */
	Io_Pcs_Initialization(Io_Pcs_Cnf);

	/* Call the initialization interfaces for ADC driver */
	Io_Adc_Initialization(config_adc_ptr);

	/* Call the initialization interfaces for TPM (Timer PWM) driver */
	Io_Tpm_Initialization(Io_Tpm_Cnf);

	/* Call the initialization interfaces for I2C driver */
	Io_I2c_Init(Io_I2c_Cnf);
	Io_SPI_Init(Io_SPI_Cnf);
	/* Call the initialization interfaces for ASY driver*/
	Io_Asy_Initialization(&config_asy_device);
	/* Call the initialization interfaces for Lcd driver*/
    //Io_Lcd_Init(IO_LCD_DISP_ON);
    /* Call the initialization variables for SH_aplication*/
	/* Call the initialization interface for WDT (Watchdog) driver (not configurable) */
	//Io_Wdt_Init();
    //Io_Dio_SetPinLevel(CNF_IO_PCS_P0E_29,LED_ON);


/** end of Initialization calls for MCU Drivers **/


/** Initialization calls for HW Interfaces **/

	/* Call the initialization interfaces for HBR (H-Bridge) driver */
	Io_Hbr_Drv8833_Initialization(Io_Hbr_Drv8833_Cnf);

    //initialise_fxos8700();
	/* Call the initialization interfaces for SENSORS driver */
	//Io_Sens_Initialization();
	SB_InitVariables();
	//SB_Init_ToF();
	init(true);
	setTimeout(500);
	startContinuous(30);
	//initialise_fxos8700(); //accelerometru si magnetometru
	/* Call the initialization interfaces for SMC (Stepper Motor Control) driver */
	// to be called if used

/** end of Initialization calls for HW Interfaces **/


/** Initialization calls for Application interfaces **/

/** end of Initialization calls for Application interfaces **/


/** Initialization calls for Test interfaces **/

/** end of Initialization calls for Test interfaces **/


/** Initialization calls for OS & Interrupts Interfaces **/

	/* Call the initialization interfaces for OS functionality */
	Io_Os_Init();
	 /* Interrupts enabled after all drivers initialization */
	Io_Int_EnableInterrupts();
	Io_Sch_StartOs(0, &Io_Sch_OsConfig[0]);
	//Io_Tpm_PwmChangeDutycycle(TEST_PWM_PIN,50);
	return 0;
}

