/**
 ******************************************************************************
 * @file           : laserlock.c
 * @brief          : Code for locking laser frequency to dip
 * @author		   : Stuart Kenny
 ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 Stuart Kenny.
  * All rights reserved.
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "laserlock.h"
#include "main.h" //needed for port and timer definitions
//#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
//#include <string.h>
//#include <math.h>

/* Typedefs -----------------------------------------------------------*/

/* Defines ------------------------------------------------------------*/
#define LASER_MIN_MOD 0 //read-only
#define LASER_MAX_MOD 1735 //corresponds to 1.4V DAC output for 35mA current
#define LOCK_TO_DIP 2 //should be 2 or 3 to select that absorption dip

/* laser states */
enum laser_states
{
  LASER_ON_FREQ = 0,
  LASER_STEPPED_UP,
  LASER_STEPPED_DOWN,
  LASER_RAMPING
};

//enum sweep_types
//{
//  FIXED_STEPS = 0,
//  FIXED_TIME
//};
//
//enum sweep_mode
//{
//  CONTINUOUS_SWEEP = 0,
//  POP_CAL_ONLY,
//  SWEEP_ONCE
//};

//struct laser_struct
//{
//  uint8_t state;             /* current laser state */
//  uint8_t k;
//  uint32_t NINT;
//  uint32_t NFRAC_start; //ramp starting value of fractional register
//  uint32_t num_steps;
//  uint32_t step_size; //in multiples of minimum step
//  uint32_t pop_cycles_per_point;
//  uint32_t stabilise_time; //in us
//  uint32_t dwell_time; //in us
//  uint32_t MW_processing_time; //in us
//  uint32_t current_point; //keeps track of which point
//  double centre_freq; //in Hz
//  double span; //in Hz
//  double sweep_period; //period of sweep in s
//  bool sweep_type; //fixed steps or fixed time
//  uint8_t sweep_mode; //continuous, POP period calibration, or single sweep
//};
//
//struct laser_struct laser_settings;  //create a structure to store the laser settings

/* Variables ---------------------------------------------------------*/
static uint8_t laser_state = LASER_ON_FREQ;
static const uint16_t LASER_STEP = 3; //circa 60uA @ a calculated 20.1uA/step
static uint16_t LASER_MOD_VALUE = LASER_MIN_MOD; //can vary between 0 and 4095
static uint16_t F2_MOD_VALUE = 0; //for storing the initial current modulation value of the F=2 absorption dip
static uint16_t F3_MOD_VALUE = 0; //for storing the initial current modulation value of the F=3 absorption dip
static uint32_t max_adc_val = 0; // for temporary storage of the largest adc reading
static uint32_t min_adc_val = 0xffffffff; //for temporary storage of the smallest adc reading
//extern volatile uint16_t sample_count; //counts number of times the sample line drives the ADC trigger high, updated when ADC completes conversion
extern uint32_t adc_val; //used to store adc3 readings

/* Function prototypes -----------------------------------------------*/
//__attribute__((section(".itcm"))) static uint32_t template_function(const uint32_t data, const bool verify);
//extern void Error_Handler(void);
extern void reset_adc_samples(void);

/**
  * @brief  Function x.
  * @retval int
  */

void startlaserlockfunc1(const bool num_samples) {
	/* Requires ADC to be initialised and for HAL_ADC_ConvCpltCallback to be active */
	HAL_GPIO_WritePin(MW_INVALID_GPIO_Port, MW_INVALID_Pin, GPIO_PIN_SET); 	//Sets MW_invalid pin high to reset POP cycle
	HAL_Delay(10); // 10ms in case ADC was part-way through a conversion
	reset_adc_samples(); //reset ADC samples including sample count
	laser_state = LASER_RAMPING;
	mw_sweep_settings.state = MW_STOPPED; //but need a function call to do this from a separate file
	start_timer(MW_TIMER); //reset MW_timer and start counting
	HAL_GPIO_WritePin(MW_INVALID_GPIO_Port, MW_INVALID_Pin, GPIO_PIN_RESET); //Restart POP cycle
	#ifdef MW_VERBOSE
		printf("POP calibration started\r\n");
	#endif //MW_VERBOSE
}

//MW_invalid low to ensure sample pulse is generated - not strictly needed in FPGA state 0
HAL_GPIO_WritePin(MW_INVALID_GPIO_Port, MW_INVALID_Pin, GPIO_PIN_RESET); //Sets MW_invalid pin low
//laser_tuning (FPGA input pin 18) must be high for probe on and MW off
HAL_GPIO_WritePin(LASER_TUNING_GPIO_Port, LASER_TUNING_Pin, GPIO_PIN_SET); // Laser_tuning output high
reset_adc_samples(); //reset ADC samples including sample count

/**
  * @brief  Stops laser tuning ready to return to POP
  * @retval None
  */
void stop_laser_tuning(void) {
	laser_state = LASER_ON_FREQ;
	//stop_timer(MW_TIMER);
	HAL_GPIO_WritePin(LASER_TUNING_GPIO_Port, LASER_TUNING_Pin, GPIO_PIN_RESET); // Laser_tuning output low
	reset_adc_samples(); //reset ADC samples including sample count
}

/**
  * @brief  Checks laser locking status to see if an action is needed.
  * @retval True if an action was taken
  */
const bool laser_update(void) {
	uint8_t local_copy_of_laser_state = laser.state; //hack to make switch statement behave
	bool action_taken = false;
	uint32_t sweep_period_us;
	switch (local_copy_of_laser_state)
	{
		case ON_FREQ:
			break; //no action to take

		case MW_STABILISING: //waiting for MW output to stabilise
			if (check_timer(MW_TIMER) < MW_STABILISE_TIME_US) return(false); //Still waiting, no action taken
			//Otherwise MW stabilisation timer has elapsed
			stop_timer(MW_TIMER);
			HAL_GPIO_WritePin(MW_INVALID_GPIO_Port, MW_INVALID_Pin, GPIO_PIN_RESET); //Sets MW_invalid pin low as MW now stable
			mw_sweep_settings.state = MW_DWELL;
			start_timer(MW_TIMER); //Restart timer for DWELL time
			action_taken = true;
			break;

		case MW_DWELL: //valid MW output waiting for end of dwell time
			if (check_timer(MW_TIMER) < mw_sweep_settings.dwell_time) return(false); //Still waiting
			//Otherwise dwell timer has elapsed
			action_taken = true;
			stop_timer(MW_TIMER);
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin); //toggles red LED
			HAL_GPIO_WritePin(MW_INVALID_GPIO_Port, MW_INVALID_Pin, GPIO_PIN_SET); //Sets MW_invalid pin high as about to change frequency
			mw_sweep_settings.state = MW_STABILISING;
			if (mw_sweep_settings.current_point == mw_sweep_settings.num_steps) { // All steps completed, tidy up and restart next sweep
				HAL_GPIO_WritePin(SCOPE_TRIG_OUT_GPIO_Port, SCOPE_TRIG_OUT_Pin, GPIO_PIN_SET); // Sets trigger output high
				sweep_period_us=check_timer(SWEEP_TIMER);
				stop_timer(SWEEP_TIMER);
				printf("Sweep complete in %.4g s. Expected %.4g s. %u samples\r\n", (double)(sweep_period_us)/1000000, mw_sweep_settings.sweep_period, sample_count);
				/* Check if the ADC registered the correct number of samples */
				uint16_t expected_samples = mw_sweep_settings.pop_cycles_per_point * (mw_sweep_settings.num_steps + 1);
				uint16_t possible_samples = expected_samples + mw_sweep_settings.num_steps + 1;
//				printf("Sweep generated %u full POP cycles and registered %u samples\r\n", expected_samples, sample_count);
				if ((sample_count == expected_samples) || (sample_count == possible_samples)) {
					#ifdef MW_VERBOSE
					printf("Sweep generated and successfully registered %u samples\r\n", sample_count);
					#endif //MW_VERBOSE
				} else {
					printf("Warning - sweep generated %u samples but %u registered\r\n", expected_samples, sample_count);
					printf("Timing of last sample is marginal\r\n");
				}
				#ifdef MW_VERBOSE
				/* calculate measured time per point */
				printf("%lu points\r\n", mw_sweep_settings.num_steps + 1);
				uint32_t measured_time_per_point_us = (double)(sweep_period_us)/(mw_sweep_settings.num_steps+1);
				printf("measured_time_per_point, %lu us\r\n", measured_time_per_point_us);
//				calculated processing time
//				uint32_t measured_processing_time_us = (measured_time_per_point_us - TIMING_MARGIN_US - MW_STABILISE_TIME_US - mw_sweep_settings.dwell_time);
//				printf("MW processing time: %lu us\r\n", measured_processing_time_us);
//				if ((double)(measured_processing_time_us)/MW_PROCESSING_TIME_US > 1.1) {
//					printf("Warning - measured MW processing time (%lu us)is larger than the %lu us expected\r\n", measured_processing_time_us, MW_PROCESSING_TIME_US);
//				}
				#endif //MW_VERBOSE
				if (mw_sweep_settings.sweep_mode == SWEEP_ONCE) {//have reached the end of a single sweep and should stop
					mw_sweep_settings.state = MW_STOPPED;
				} else {
					start_MW_sweep(false); //restart the next MW sweep without updating mw_sweep_settings.sweep_mode
//					start_POP_calibration(false); //check the POP period and restart the next MW sweep without updating mw_sweep_settings.sweep_mode
				}
			} else {
				/* next MW step */
				mw_sweep_settings.current_point++; //increment point counter
				//calculate the new MW frequency and program Hittite with new NFRAC
				uint32_t local_NFRAC = mw_sweep_settings.NFRAC_start + mw_sweep_settings.step_size * mw_sweep_settings.current_point;
				set_freq_regs(mw_sweep_settings.NINT, local_NFRAC, mw_sweep_settings.k); //program new MW frequency
				start_timer(MW_TIMER); //Restart timer for MW stabilisation time
				#ifdef RAMP_DAC
					dac_val = dac_val + (4096.0/mw_sweep_settings.num_steps);
					if(HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)dac_val) != HAL_OK){
						printf("Failure to program value to DAC \r\n");
						Error_Handler();
					}
				#endif //RAMP_DAC
			}
			break;

		case MW_CALIBRATE: //Measures the elapsed time taken for 101 samples (100 POP cycles)
			if (sample_count >= 100) {//100 or more POP cycles have elapsed
				uint32_t total_POP_cal_period = check_timer(MW_TIMER);
				POP_period_us = (float)(total_POP_cal_period) / 100 + 0.5;
				stop_timer(MW_TIMER);
				printf("POP period, averaged over 100 cycles: %lu us\r\n", POP_period_us);
				action_taken = true;
				if (mw_sweep_settings.sweep_mode == POP_CAL_ONLY) {//have reached the end of calibration and should stop
					mw_sweep_settings.state = MW_STOPPED;
				} else {
						start_MW_sweep(false); //start MW_sweep without updating mw_sweep_settings.sweep_mode
				}
			}
			break;

		default: // Other state
	       printf("MW_update has detected illegal state: %u \r\n", mw_sweep_settings.state);
	       printf("local version: %u \r\n", local_copy_of_MW_state);
	}
    return(action_taken);
}
