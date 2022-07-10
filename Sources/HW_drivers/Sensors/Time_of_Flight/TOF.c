/*
 * TOF.c
 *
 *  Created on: Feb 1, 2017
 *      Author: uidq9116
 */
#include"TOF.h"
#include "MCU_drivers/I2c/io_i2c_cnf.h"
#include "SB_Functions/SB_Main.h"
#include "MCU_drivers/I2c/io_i2c.h"
#include "Platform_Types/std_types.h"
#include "Platform_Types/platform_types.h"
#include <stdbool.h>

#define ADDRESS_DEFAULT 0x52
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

void startTimeout() {timeout_start_ms = SB_get_milis();}
bool checkTimeoutExpired()
{
	if(io_timeout > 0 && (SB_get_milis() - timeout_start_ms) > io_timeout)
		return true;
	return false;
}
void setTimeout(uint32 timeout) { io_timeout = timeout; }
bool init(bool io_2v8)
{

uint8 value_read = 0;
volatile uint8 temp_val = 0;


 if (io_2v8)
  {
    temp_val = Io_I2c_RegRead(I2C1,ADDRESS_DEFAULT,VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV);
	temp_val |= 0x01;
    Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, temp_val); // set bit 0
  }

  // "Set I2C standard mode"
  Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x88, 0x00);
  Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x80, 0x01);
  Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x01);
  Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x00, 0x00);
  stop_variable = Io_I2c_RegRead(I2C1,ADDRESS_DEFAULT,0x91);
  Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x00, 0x01);
  Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x00);
  Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x80, 0x00);
  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  value_read = Io_I2c_RegRead(I2C1,ADDRESS_DEFAULT,MSRC_CONFIG_CONTROL);
  value_read |= 0x12;
  Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,MSRC_CONFIG_CONTROL, value_read);
	 setSignalRateLimit(0.25);

  Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,SYSTEM_SEQUENCE_CONFIG, 0xFF);

  // VL53L0X_DataInit() end

  // VL53L0X_StaticInit() begin

  uint8 spad_count;
  bool spad_type_is_aperture;
  if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  uint8 ref_spad_map[6];
  Io_I2c_RegReadN(I2C1,ADDRESS_DEFAULT,GLOBAL_CONFIG_SPAD_ENABLES_REF_0,6,ref_spad_map);

  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8 first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  uint8 spads_enabled = 0;

  for (uint8 i = 0; i < 48; i++)
  {
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
    {
      spads_enabled++;
    }
  }

 // Io_I2c_WriteRegN(I2C1,ADDRESS_DEFAULT,GLOBAL_CONFIG_SPAD_ENABLES_REF_0,6,ref_spad_map);
  Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map[0]);
  Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map[1]);
  Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map[2]);
  Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map[3]);
  Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map[4]);
  Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map[5]);
  // -- VL53L0X_set_reference_spads() end

  // -- VL53L0X_load_tuning_settings() begin
  // DefaultTuningSettings from vl53l0x_tuning.h

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x00, 0x00);

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x09, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x10, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x11, 0x00);

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x24, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x25, 0xFF);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x75, 0x00);

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x4E, 0x2C);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x48, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x30, 0x20);

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x30, 0x09);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x54, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x31, 0x04);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x32, 0x03);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x40, 0x83);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x46, 0x25);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x60, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x27, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x50, 0x06);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x51, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x52, 0x96);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x56, 0x08);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x57, 0x30);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x61, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x62, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x64, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x65, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x66, 0xA0);

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x22, 0x32);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x47, 0x14);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x49, 0xFF);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x4A, 0x00);

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x7A, 0x0A);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x7B, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x78, 0x21);

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x23, 0x34);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x42, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x44, 0xFF);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x45, 0x26);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x46, 0x05);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x40, 0x40);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x0E, 0x06);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x20, 0x1A);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x43, 0x40);

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x34, 0x03);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x35, 0x44);

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x31, 0x04);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x4B, 0x09);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x4C, 0x05);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x4D, 0x04);

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x44, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x45, 0x20);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x47, 0x08);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x48, 0x28);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x67, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x70, 0x04);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x71, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x72, 0xFE);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x76, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x77, 0x00);

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x0D, 0x01);

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x80, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x01, 0xF8);

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x8E, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x00, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x80, 0x00);

  // -- VL53L0X_load_tuning_settings() end

  // "Set interrupt config to new sample ready"
  // -- VL53L0X_SetGpioConfig() begin
	uint8 temp_value = 0;
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
   temp_value = Io_I2c_RegRead(I2C1,ADDRESS_DEFAULT,GPIO_HV_MUX_ACTIVE_HIGH);
   temp_value &= ~0x10;
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,GPIO_HV_MUX_ACTIVE_HIGH,  temp_value); // active low
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,SYSTEM_INTERRUPT_CLEAR, 0x01);

  // -- VL53L0X_SetGpioConfig() end

  measurement_timing_budget_us = getMeasurementTimingBudget();

  // "Disable MSRC and TCC by default"
  // MSRC = Minimum Signal Rate Check
  // TCC = Target CentreCheck
  // -- VL53L0X_SetSequenceStepEnable() begin

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // -- VL53L0X_SetSequenceStepEnable() end

  // "Recalculate timing budget"
  setMeasurementTimingBudget(measurement_timing_budget_us);

  // VL53L0X_StaticInit() end

  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

  // -- VL53L0X_perform_vhv_calibration() begin

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!performSingleRefCalibration(0x40)) { return false; }

  // -- VL53L0X_perform_vhv_calibration() end

  // -- VL53L0X_perform_phase_calibration() begin

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!performSingleRefCalibration(0x00)) { return false; }

  // -- VL53L0X_perform_phase_calibration() end

  // "restore the previous Sequence Config"
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // VL53L0X_PerformRefCalibration() end
  return true;
}

bool setSignalRateLimit(float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  Io_I2c_WriteReg16Bit(I2C1,ADDRESS_DEFAULT,FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
  return true;
}

float getSignalRateLimit(void)
{
  return (float)Io_I2c_RegRead16bit(I2C1,ADDRESS_DEFAULT,FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

bool setMeasurementTimingBudget(uint32 budget_us)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16 const StartOverhead      = 1320; // note that this is different than the value in get_
  uint16 const EndOverhead        = 960;
  uint16 const MsrcOverhead       = 660;
  uint16 const TccOverhead        = 590;
  uint16 const DssOverhead        = 690;
  uint16 const PreRangeOverhead   = 660;
  uint16 const FinalRangeOverhead = 550;

  uint32 const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return false; }

  uint32 used_budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return false;
    }

    uint32 final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16 final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(final_range_timeout_us,timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    Io_I2c_WriteReg16Bit(I2C1,ADDRESS_DEFAULT,FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,encodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return true;
}

uint32 getMeasurementTimingBudget(void)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16 const StartOverhead     = 1910; // note that this is different than the value in set_
  uint16 const EndOverhead        = 960;
  uint16 const MsrcOverhead       = 660;
  uint16 const TccOverhead        = 590;
  uint16 const DssOverhead        = 690;
  uint16 const PreRangeOverhead   = 660;
  uint16 const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32 budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

bool setVcselPulsePeriod(vcselPeriodType type, uint8 period_pclks)
{
  uint8 vcsel_period_reg = encodeVcselPeriod(period_pclks);

  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."


  if (type == VcselPeriodPreRange)
  {
    // "Set phase check limits"
    switch (period_pclks)
    {
      case 12:
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

      case 14:
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

      case 16:
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

      case 18:
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

      default:
        // invalid period
        return false;
    }
     Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
     Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16 new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

   Io_I2c_WriteReg16Bit(I2C1,ADDRESS_DEFAULT,PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,encodeTimeout(new_pre_range_timeout_mclks));

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16 new_msrc_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

     Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,MSRC_CONFIG_TIMEOUT_MACROP,(new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (type == VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x01);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,ALGO_PHASECAL_LIM, 0x30);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x00);
        break;

      case 10:
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x01);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,ALGO_PHASECAL_LIM, 0x20);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x00);
        break;

      case 12:
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x01);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,ALGO_PHASECAL_LIM, 0x20);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x00);
        break;

      case 14:
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x01);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,ALGO_PHASECAL_LIM, 0x20);
         Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x00);
        break;

      default:
        // invalid period
        return false;
    }

    // apply new VCSEL period
     Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    uint16 new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range)
    {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    Io_I2c_WriteReg16Bit(I2C1,ADDRESS_DEFAULT,FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,encodeTimeout(new_final_range_timeout_mclks));

    // set_sequence_step_timeout end
  }
  else
  {
    // invalid type
    return false;
  }

  // "Finally, the timing budget must be re-applied"

  setMeasurementTimingBudget(measurement_timing_budget_us);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin

  uint8 sequence_config = Io_I2c_RegRead(I2C1,ADDRESS_DEFAULT,SYSTEM_SEQUENCE_CONFIG);
  Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,SYSTEM_SEQUENCE_CONFIG, 0x02);
  performSingleRefCalibration(0x0);
  Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,SYSTEM_SEQUENCE_CONFIG, sequence_config);

  // VL53L0X_perform_phase_calibration() end

  return true;
}

uint8 getVcselPulsePeriod(vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  {
    return decodeVcselPeriod(Io_I2c_RegRead(I2C1,ADDRESS_DEFAULT,PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(Io_I2c_RegRead(I2C1,ADDRESS_DEFAULT,FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; }
}

void startContinuous(uint32 period_ms)
{
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x80, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x00, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x91, stop_variable);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x00, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x80, 0x00);

  if (period_ms != 0)
  {
    // continuous timed mode

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

    uint16 osc_calibrate_val = Io_I2c_RegRead16bit(I2C1,ADDRESS_DEFAULT,OSC_CALIBRATE_VAL);

    if (osc_calibrate_val != 0)
    {
      period_ms *= osc_calibrate_val;
    }

   Io_I2c_WriteReg32Bit(I2C1,ADDRESS_DEFAULT,SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

     Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
  }
  else
  {
    // continuous back-to-back mode
     Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
  }
}

void stopContinuous(void)
{
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x00, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x91, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x00, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x00);
}

uint16 readRangeContinuousMillimeters(void)
{
  startTimeout();
  while ((Io_I2c_RegRead(I2C1,ADDRESS_DEFAULT,RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (checkTimeoutExpired())
    {
      did_timeout = true;
      return 65535;
    }
  }
  uint16 range = Io_I2c_RegRead16bit(I2C1,ADDRESS_DEFAULT,RESULT_RANGE_STATUS + 10);
  /*uint8 rangearray[2];
  Io_I2c_RegReadN(I2C1,ADDRESS_DEFAULT,RESULT_RANGE_STATUS + 10,2,rangearray);
  uint16 range = (uint16) rangearray[0] << 8;
  range |= rangearray[1];*/
  range /= 10; //get range in centimeters
  Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,SYSTEM_INTERRUPT_CLEAR, 0x01);
  return range;
}

uint16 readRangeSingleMillimeters(void)
{
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x80, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x00, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x91, stop_variable);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x00, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x80, 0x00);

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,SYSRANGE_START, 0x01);

  // "Wait until start bit has been cleared"
  startTimeout();
  while (Io_I2c_RegRead(I2C1,ADDRESS_DEFAULT,SYSRANGE_START) & 0x01)
  {
    if (checkTimeoutExpired())
    {
      did_timeout = true;
      return 65535;
    }
  }

  return readRangeContinuousMillimeters();
}

bool timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

bool getSpadInfo(uint8 * count, bool * type_is_aperture)
{
  volatile uint8 tmp;
  volatile uint8 tmp1;
  volatile uint8 tmp2;
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x80, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x00, 0x00);

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x06);
   tmp1 = Io_I2c_RegRead(I2C1,ADDRESS_DEFAULT,0x83);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x83,  tmp1 | 0x04);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x07);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x81, 0x01);

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x80, 0x01);

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x94, 0x6b);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x83, 0x00);
  startTimeout();
  while (Io_I2c_RegRead(I2C1,ADDRESS_DEFAULT,0x83) == 0x00)
  {
    if (checkTimeoutExpired()) { return false; }
  }
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x83, 0x01);
  tmp = Io_I2c_RegRead(I2C1,ADDRESS_DEFAULT,0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x81, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x06);
   tmp2 = Io_I2c_RegRead(I2C1,ADDRESS_DEFAULT, 0x83  & ~0x04);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x83, tmp2);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x00, 0x01);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0xFF, 0x00);
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,0x80, 0x00);

  return true;
}

void getSequenceStepEnables(SequenceStepEnables * enables)
{
  uint8 sequence_config = Io_I2c_RegRead(I2C1,ADDRESS_DEFAULT,SYSTEM_SEQUENCE_CONFIG);
  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = Io_I2c_RegRead(I2C1,ADDRESS_DEFAULT,MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
    timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    decodeTimeout(Io_I2c_RegRead16bit(I2C1,ADDRESS_DEFAULT,PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
    timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);

  timeouts->final_range_mclks =
    decodeTimeout(Io_I2c_RegRead16bit(I2C1,ADDRESS_DEFAULT,FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32, but the return value is
// always stored in a uint16.
uint16 decodeTimeout(uint16 reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16)((reg_val & 0x00FF) <<(uint16)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16, but the argument passed to it
// is always a uint16.
uint16 encodeTimeout(uint16 timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32 ls_byte = 0;
  uint16 ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32 timeoutMclksToMicroseconds(uint16 timeout_period_mclks, uint8 vcsel_period_pclks)
{
  uint32 macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32 timeoutMicrosecondsToMclks(uint32 timeout_period_us, uint8 vcsel_period_pclks)
{
  uint32 macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}


// based on VL53L0X_perform_single_ref_calibration()
bool performSingleRefCalibration(uint8 vhv_init_byte)
{
   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

  startTimeout();
  while ((Io_I2c_RegRead(I2C1,ADDRESS_DEFAULT,RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (checkTimeoutExpired()) { return false; }
  }

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,SYSTEM_INTERRUPT_CLEAR, 0x01);

   Io_I2c_RegWrite(I2C1,ADDRESS_DEFAULT,SYSRANGE_START, 0x00);

  return true;
}
