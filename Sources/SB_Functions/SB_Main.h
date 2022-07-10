#ifndef SOURCES_SB_FUNCTIONS_SB_MAIN_H_
#define SOURCES_SB_FUNCTIONS_SB_MAIN_H_
#include "Platform_Types/std_types.h"

#define MAX_VALUE_15BITS            32736
#define VREF_mV                      3300
#define DUTYCYCLE_MAX_VALUE         16384  /* see the MCU_drivers ->Tpm ->io_tpm_cnf.c ->const Io_Tpm_PwmChannelCnfStruct Io_Tpm_Pwm_Ch_Cnf[] ->DESCRIPTION */
#define BUTTON_TIME 500
#define TIME_ON_THRESH 500
#define LED_HIGH 0
#define LED_LOW 1
extern void SB_RunningTask1ms();
extern void SB_RunningTask10ms();
extern void SB_RunningTask100ms();
extern void SB_RunningTask1000ms();
extern void SB_RunningTask50ms();

extern void SB_InitVariables();

extern void SB_PWMSetPercentDuty_cycle(uint32 channel,uint8 dutycyclePercentage);
extern uint16 SB_ReadAnalog(char channel);
extern uint16 SB_ReadAnalog_mV(char channel);
extern uint8 SB_ReanAnalog_Percent(char channel);
extern void Read_Potentiometer();
extern uint32 SB_get_milis();
extern void Read_Telemetric_Module();
extern volatile uint32 system_miliseconds;

extern uint8 pwm_counter;
extern uint8 pwm_slave;
#endif
