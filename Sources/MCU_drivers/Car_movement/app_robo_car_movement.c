/**************************************************************************

COPYRIGHT (C) $Date: Sep 4, 2015 $
$CompanyInfo: CONTINENTAL AUTOMOTIVE GMBH $
ALL RIGHTS RESERVED.

The reproduction, transmission or use of this document or its contents is
not permitted without express written authority.
Offenders will be liable for damages. All rights, including rights created
by patent grant or registration of a utility model or design, are reserved.
---------------------------------------------------------------------------

$ProjectName: FRDM KL26-Z PLATFORM $

$Log: app_robo_car_movement.c $

$Author: Flueran Gabriel $

 ****************************************************************************/

#include "MCU_drivers/Tpm/io_tpm_cnf.h"
#include "HW_drivers/H-bridge/ti8833/io_hbr_cnf_ti8833.h"
#include "HW_drivers/H-bridge/ti8833/io_hbr_ti8833.h"
#include "app_robo_car_movement.h"

/** Global variables and constants **/

uint32 Algo_Robo_Cm_TimerRotateLeft;
uint32 Algo_Robo_Cm_TimerRotateRight;
uint32 Algo_Robo_Cm_TimerMoveForward;
uint32 Algo_Robo_Cm_TimerMoveBackward;

/** end of Global variables and constants **/

/** Local Functions prototypes **/

static uint16 Algo_Robo_Cm_LimitDutycycle(uint16 dutycycle);

/** end of Local Functions prototypes **/

/** Functions implementation **/

/* FUNCTION NAME: Algo_Robo_Cm_InitializeCarMovement
 * RETURN TYPE: void
 * PARAMETERS: void
 * DESCRIPTION: Initializes the variables used by the Car Movement module.
 * OBSERVATIONS: -
 */
void Algo_Robo_Cm_InitializeCarMovement(void)
{
	Algo_Robo_Cm_TimerRotateLeft = 0;
	Algo_Robo_Cm_TimerRotateRight = 0;
	Algo_Robo_Cm_TimerMoveForward = 0;
	Algo_Robo_Cm_TimerMoveBackward = 0;
}

/* FUNCTION NAME: Algo_Robo_Cm_UpdateCarSpeed
 * RETURN TYPE: void
 * PARAMETERS: uint8 direction,uint16 dutycycle
 * DESCRIPTION: Abruptly changes the car speed to the desired dutycycle, in the specified direction.
 * OBSERVATIONS: -
 */
void Algo_Robo_Cm_UpdateCarSpeed(uint8 direction,uint16 dutycycle)
{
	uint16 dutycycleHex = IO_TPM_PERCENT_TO_4000HEX(dutycycle);

	Io_Hbr_Drv8833_Update(direction,dutycycleHex,IO_HBR_MOTORS_LEFT);
	Io_Hbr_Drv8833_Update(direction,dutycycleHex,IO_HBR_MOTORS_RIGHT);

	//Algo_Robo_Pp_CarCurrentSpeedCmPerS = ALGO_ROBO_SC_CALCULATE_CAR_SPEED(dutycycle);
}

/* FUNCTION NAME: Algo_Robo_Cm_DifferentDutycyle
 * RETURN TYPE: void
 * PARAMETERS: uint8 direction,uint16 dutycycleLeft,uint16 dutycycleRight
 * DESCRIPTION: Abruptly changes the car speed with the 2 dutycycles, in the specified direction.
 * OBSERVATIONS: -
 */
void Algo_Robo_Cm_DifferentDutycyle(uint8 direction_left,uint8 direction_right,uint16 dutycycleLeft,uint16 dutycycleRight)
{

	Io_Hbr_Drv8833_Update(direction_left,dutycycleLeft,IO_HBR_MOTORS_LEFT);
	Io_Hbr_Drv8833_Update(direction_right,dutycycleRight,IO_HBR_MOTORS_RIGHT);

	//Algo_Robo_Pp_CarCurrentSpeedCmPerS = ALGO_ROBO_SC_CALCULATE_CAR_SPEED(dutycycle);
}

/* FUNCTION NAME: Algo_Robo_Cm_MoveCarForward
 * RETURN TYPE: void
 * PARAMETERS: uint16 dutycycle
 * DESCRIPTION: Moves the car forward, incrementing the speed up to the specified dutycycle.
 * OBSERVATIONS: -
 */
void Algo_Robo_Cm_MoveCarForward(uint16 dutycycle)
{
	dutycycle = Algo_Robo_Cm_LimitDutycycle(dutycycle);

	Algo_Robo_Cm_UpdateCarSpeed(IO_HBR_DRV8833_STATE_FORWARD,dutycycle);
}

/* FUNCTION NAME: Algo_Robo_Cm_MoveCarBackward
 * RETURN TYPE: void
 * PARAMETERS: uint16 dutycycle
 * DESCRIPTION: Moves the car backward, incrementing the speed up to the specified dutycycle.
 * OBSERVATIONS: -
 */
void Algo_Robo_Cm_MoveCarBackward(uint16 dutycycle)
{
	dutycycle = Algo_Robo_Cm_LimitDutycycle(dutycycle);

	Algo_Robo_Cm_UpdateCarSpeed(IO_HBR_DRV8833_STATE_REVERSE,dutycycle);
	//Algo_Robo_Pp_CarMovement = ALGO_ROBO_PP_CAR_MOVING_BACKWARD;
}

/* FUNCTION NAME: Algo_Robo_Cm_RotateCarLeft
 * RETURN TYPE: void
 * PARAMETERS: uint16 dutycycle
 * DESCRIPTION: Moves the car to the left, updating the speed to the set dutycycle.
 * OBSERVATIONS: -
 */
void Algo_Robo_Cm_RotateCarLeft(uint16 dutycycle)
{
	uint16 dutycycleHex;

	dutycycle = Algo_Robo_Cm_LimitDutycycle(dutycycle);
	dutycycleHex = IO_TPM_PERCENT_TO_4000HEX(dutycycle);

	Io_Hbr_Drv8833_Update(IO_HBR_DRV8833_STATE_REVERSE,dutycycleHex,IO_HBR_MOTORS_LEFT);
	Io_Hbr_Drv8833_Update(IO_HBR_DRV8833_STATE_FORWARD,dutycycleHex,IO_HBR_MOTORS_RIGHT);
	//Algo_Robo_Pp_CarMovement = ALGO_ROBO_PP_CAR_ROTATING_LEFT;

	//Algo_Robo_Pp_CarCurrentSpeedCmPerS = 0;
}

/* FUNCTION NAME: Algo_Robo_Cm_RotateCarRight
 * RETURN TYPE: void
 * PARAMETERS: uint16 dutycycle
 * DESCRIPTION: Moves the car to the right, updating the speed to the set dutycycle.
 * OBSERVATIONS: -
 */
void Algo_Robo_Cm_RotateCarRight(uint16 dutycycle)
{
	uint16 dutycycleHex;

	dutycycle = Algo_Robo_Cm_LimitDutycycle(dutycycle);
	dutycycleHex = IO_TPM_PERCENT_TO_4000HEX(dutycycle);

	Io_Hbr_Drv8833_Update(IO_HBR_DRV8833_STATE_FORWARD,dutycycleHex,IO_HBR_MOTORS_LEFT);
	Io_Hbr_Drv8833_Update(IO_HBR_DRV8833_STATE_REVERSE,dutycycleHex,IO_HBR_MOTORS_RIGHT);
	//Algo_Robo_Pp_CarMovement = ALGO_ROBO_PP_CAR_ROTATING_RIGHT;

	//Algo_Robo_Pp_CarCurrentSpeedCmPerS = 0;
}

/* FUNCTION NAME: Algo_Robo_Cm_BreakCar
 * RETURN TYPE: void
 * PARAMETERS: void
 * DESCRIPTION: Car shortbrake is performed.
 * OBSERVATIONS: -
 */
void Algo_Robo_Cm_BreakCar(void)
{
	Io_Hbr_Drv8833_Set_State(IO_HBR_MOTORS_LEFT,IO_HBR_DRV8833_STATE_BRAKE);
	Io_Hbr_Drv8833_Set_State(IO_HBR_MOTORS_RIGHT,IO_HBR_DRV8833_STATE_BRAKE);
	//Algo_Robo_Pp_CarMovement = ALGO_ROBO_PP_CAR_STOPPED;

	//Algo_Robo_Pp_CarCurrentSpeedCmPerS = 0;
}

/* FUNCTION NAME: Algo_Robo_Cm_StopCar
 * RETURN TYPE: void
 * PARAMETERS: void
 * DESCRIPTION: Car is stopped.
 * OBSERVATIONS: -
 */
void Algo_Robo_Cm_StopCar(void)
{
	Io_Hbr_Drv8833_Set_State(IO_HBR_MOTORS_LEFT,IO_HBR_DRV8833_STATE_BRAKE);
	Io_Hbr_Drv8833_Set_State(IO_HBR_MOTORS_RIGHT,IO_HBR_DRV8833_STATE_BRAKE);
	//Algo_Robo_Pp_CarMovement = ALGO_ROBO_PP_CAR_STOPPED;

	//Algo_Robo_Pp_CarCurrentSpeedCmPerS = 0;
}

/* FUNCTION NAME: Algo_Robo_Cm_LimitDutycycle
 * RETURN TYPE: void
 * PARAMETERS: uint16 dutycycle
 * DESCRIPTION: Limits the dutycycle so that calculated speed is approx. accurate.
 * OBSERVATIONS: private to "app_robo_car_movement.c"
 */
uint16 Algo_Robo_Cm_LimitDutycycle(uint16 dutycycle)
{
	uint16 limitedDutycycle = dutycycle;

	if(limitedDutycycle < ALGO_ROBO_CM_MIN_DUTYCYCLE)
	{
		limitedDutycycle = ALGO_ROBO_CM_MIN_DUTYCYCLE;
	}
	else
	{
		if(limitedDutycycle > ALGO_ROBO_CM_MAX_DUTYCYCLE)
		{
			limitedDutycycle = ALGO_ROBO_CM_MAX_DUTYCYCLE;
		}
		else {}
	}
	return limitedDutycycle;
}
/** end of Functions implementation **/



