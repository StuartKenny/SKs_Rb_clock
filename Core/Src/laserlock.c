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
#include "main.h" //needed for port, timer and ADC definitions
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
//#include <string.h>
//#include <math.h>

/* Typedefs -----------------------------------------------------------*/

/* Defines ------------------------------------------------------------*/
#define LASER_MIN_MOD 10 //read-only
//#define LASER_MAX_MOD 1738 //corresponds to 1.4V DAC output for 35mA LD current
#define LASER_MAX_MOD 3000 //trying a higher voltage due to low modulation sensitivity
#define LOCK_TO_DIP 3 //should be 2 or 3 to select the F=? absorption dip
#define DIP_THRESHOLD 248 //approx 200mV
#define LASER_STAB_US 1000 //1ms time for LD temp to stabilise after polling

/* laser states */
enum laser_states
{
  LASER_ON_FREQ = 0,
  LASER_TEMP_STABILISING,
  LASER_STEPPED_UP,
  LASER_STEPPED_DOWN,
  LASER_RAMP_PHASE_ONE,
  LASER_RAMP_PHASE_TWO,
  LASER_RAMP_PHASE_THREE,
  LASER_RAMP_PHASE_FOUR,
  LASER_RAMP_PHASE_FIVE
};

/* Variables ---------------------------------------------------------*/
static uint8_t laser_state = LASER_ON_FREQ;
static const uint16_t LASER_STEP = 3; //circa 60uA @ a calculated 20.1uA/step
static uint16_t laser_mod_value = LASER_MIN_MOD; //can vary between 0 and 4095
static uint16_t saved_mod_value = 0; //for temporary storage of a current modulation value
uint16_t moving_average_offset = 0; //half the width of the moving average
static uint16_t F2_mod_value = 0; //for storing the initial current modulation value of the F=2 absorption dip
static uint16_t F3_mod_value = 0; //for storing the initial current modulation value of the F=3 absorption dip
//static uint32_t max_adc_val = 0; // for temporary storage of the largest adc reading
//static uint32_t min_adc_val = 0xffffffff; //for temporary storage of the smallest adc reading
//extern volatile uint16_t sample_count; //counts number of times the sample line drives the ADC trigger high, updated when ADC completes conversion
extern uint32_t adc_averaged_val, adc_averaged_max, adc_averaged_min; //used to store adc3 readings
extern bool adc_average_updated; //allows polling without needing to compare previous values
extern uint32_t adc_polled_above, adc_polled_below; //used to store adc3 readings

extern uint32_t start_timer(TIM_TypeDef * timer);
extern uint32_t stop_timer(TIM_TypeDef * timer);
extern uint32_t check_timer(TIM_TypeDef *timer);
extern DAC_HandleTypeDef hdac1; //declared in main.c

/* Function prototypes -----------------------------------------------*/
__attribute__((section(".itcm"))) void start_laser_tuning(void);
__attribute__((section(".itcm"))) void start_laser_ramp(void);
__attribute__((section(".itcm"))) void stop_laser_tuning(void);
__attribute__((section(".itcm"))) const bool laser_update(void);
extern void Error_Handler(void);
extern void reset_adc_samples(void);
extern void stop_MW_operation(void);

/**
  * @brief  Function x.
  * @retval None
  */

void start_laser_tuning(void) {
	stop_MW_operation(); //releases timers and ensures that sample pulse is generated for ADC
	laser_state = LASER_STEPPED_UP;
	HAL_GPIO_WritePin(LASER_TUNING_GPIO_Port, LASER_TUNING_Pin, GPIO_PIN_SET); //Laser_tuning output high
	if (laser_mod_value > (LASER_MAX_MOD - LASER_STEP)) {
	    printf("LOSS OF LASER LOCK\r\n");
	    printf("Modulation value outside bounds: %u\r\n", laser_mod_value);
		Error_Handler();
	}
	laser_mod_value += LASER_STEP;
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, laser_mod_value); //
	reset_adc_samples(); //reset ADC samples including sample count
	#ifdef LASER_VERBOSE
	printf("Started laser tuning\r\n");
	#endif //LASER_VERBOSE
}

/**
  * @brief  Starts a laser scan
  * @retval None
  * This function initiates a laser frequency scan using an n-point moving average
  * ADC value where n = ADC_SAMPLES
  */
void start_laser_ramp(void) {
	stop_MW_operation(); //releases timers and ensures that sample pulse is generated for ADC
	laser_state = LASER_RAMP_PHASE_ONE;
	HAL_GPIO_WritePin(LASER_TUNING_GPIO_Port, LASER_TUNING_Pin, GPIO_PIN_SET); //Laser_tuning output high
	HAL_GPIO_WritePin(SCOPE_TRIG_OUT_GPIO_Port, SCOPE_TRIG_OUT_Pin, GPIO_PIN_SET); // Sets trigger output high
	adc_averaged_max = 0;
	adc_averaged_min = 0xFFFF;
	laser_mod_value = LASER_MIN_MOD;
	if (ADC_SAMPLES > 1) moving_average_offset = ADC_SAMPLES / 2;
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, laser_mod_value); //
	start_timer(MW_TIMER); //using MW for 1s delay
    printf("LD temperature equalising.\r\n");
}

/**
  * @brief  Stops laser tuning ready to return to POP
  * @retval None
  */
void stop_laser_tuning(void) {
	laser_state = LASER_ON_FREQ;
	stop_timer(MW_TIMER); //release MW_timer
	stop_timer(SWEEP_TIMER); //release SWEEP_timer
	HAL_GPIO_WritePin(LASER_TUNING_GPIO_Port, LASER_TUNING_Pin, GPIO_PIN_RESET); // Laser_tuning output low
	reset_adc_samples(); //reset ADC samples including sample count
}

/**
  * @brief  Checks laser locking status to see if an action is needed.
  * @retval True if an action was taken
  */
const bool laser_update(void) {
	bool action_taken = false;
	double sweep_time_s = 0;
	uint8_t local_copy_of_laser_state = laser_state; //hack to make switch statement behave
	switch (local_copy_of_laser_state)
	{
		case LASER_ON_FREQ:
			break; //no action to take
		case LASER_TEMP_STABILISING:
			if (check_timer(MW_TIMER) < LASER_STAB_US) return(false); //Still waiting, no action taken
			action_taken = true;
			stop_timer(MW_TIMER); //release timer
			laser_state = LASER_ON_FREQ;
			reset_adc_samples(); //reset ADC samples including sample count
		case LASER_STEPPED_UP:
			if(adc_average_updated) {
				adc_polled_above = adc_averaged_val;
				laser_state = LASER_STEPPED_DOWN;
				if (laser_mod_value < LASER_MIN_MOD + (2 * LASER_STEP)) {
				    printf("LOSS OF LASER LOCK\r\n");
				    printf("Modulation value outside bounds: %u\r\n", laser_mod_value);
					Error_Handler();
				}
				laser_mod_value = laser_mod_value - (2 * LASER_STEP);
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, laser_mod_value); //
				reset_adc_samples(); //reset ADC samples including sample count
				action_taken = true;
			}
			break;
		case LASER_STEPPED_DOWN:
			if(adc_average_updated) {
				adc_polled_below = adc_averaged_val;
				laser_mod_value += LASER_STEP; //return laser modulation value to pre-tuned value
				action_taken = true;
				if (adc_polled_below > adc_polled_above) {
					laser_mod_value++; //increase current by incrementing laser modulation value
				}
				if (adc_polled_above > adc_polled_below) {
					laser_mod_value--; //decrease current by decrementing laser modulation value
				}
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, laser_mod_value);
				/* If adding a short delay for LD to stabilise after polling */
				laser_state = LASER_TEMP_STABILISING;
				start_timer(MW_TIMER); //using MW for short delay
				/* Substituted with this if no stabilising time is required after polling
				 * laser_state = LASER_ON_FREQ;
				 * reset_adc_samples(); //reset ADC samples including sample count
				 */
			}
			break;
		case LASER_RAMP_PHASE_ONE: //waiting for the LD temperature to stabilise
			if (check_timer(MW_TIMER) < 1000000) return(false); //Still waiting, no action taken
			action_taken = true;
			stop_timer(MW_TIMER); //release MW_timer
			laser_state = LASER_RAMP_PHASE_TWO;
			HAL_GPIO_WritePin(SCOPE_TRIG_OUT_GPIO_Port, SCOPE_TRIG_OUT_Pin, GPIO_PIN_RESET); // Sets trigger output low
			reset_adc_samples(); //reset ADC samples including sample count
			start_timer(SWEEP_TIMER); //
		    printf("Starting laser frequency scan. Allow %.2g seconds \r\n", (float)((LASER_MAX_MOD - LASER_MIN_MOD) / 186));
		    //break statement not required here

		case LASER_RAMP_PHASE_TWO: //finding F=2 dip
			if(adc_average_updated) {
				if (adc_averaged_val < adc_averaged_min) {//if new minimum detected
					adc_averaged_min = adc_averaged_val; //record new mininum
					saved_mod_value = laser_mod_value; //record the associated modulation value
					//adc_averaged_max = 0; //reset max
				}
				/* Detect when we've passed F=2 dip
				 * If the latest reading is significantly higher than the minimum
				 * then record F=2 and look for F=3
				 */
				if ((adc_averaged_val - adc_averaged_min) >= DIP_THRESHOLD ) {//if the latest reading is significantly above the minimum
					F2_mod_value = saved_mod_value - moving_average_offset; //record the modulation value for the F=2 dip
					adc_averaged_min = 0xFFFF; //reset the saved minimum
					laser_state = LASER_RAMP_PHASE_THREE;
				}
				laser_mod_value += LASER_STEP; //next laser step
				if (laser_mod_value >= LASER_MAX_MOD) {//if no longer in range
				    printf("Have completed absorption scan without detecting any dips.\r\n");
				    printf("DIP_THRESHOLD: %u\r\n", DIP_THRESHOLD);
					sweep_time_s = (double) (stop_timer(SWEEP_TIMER)) / 1000000;
					printf("Sweep complete in %.3g s.\r\n", sweep_time_s);
					Error_Handler();
				}
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, laser_mod_value); //set DAC output pin
				//reset_adc_samples(); //reset ADC samples including sample count
				adc_average_updated = false; //clears new reading flag
				action_taken = true;
			}
			break;
		case LASER_RAMP_PHASE_THREE: //finding F=3 dip
			if(adc_average_updated) {
				if (adc_averaged_val < adc_averaged_min) {//if new minimum detected
					adc_averaged_min = adc_averaged_val; //record new mininum
					saved_mod_value = laser_mod_value; //record the associated modulation value
				}
				/* Detect when we've passed F=3 dip
				 * If the latest reading is significantly higher than the minimum
				 * then record F=3
				 */
				if ((adc_averaged_val - adc_averaged_min) >= DIP_THRESHOLD) {//if the latest reading is significant above the minimum
					F3_mod_value = saved_mod_value - moving_average_offset; //record the modulation value for the F=3 dip
					laser_state = LASER_RAMP_PHASE_FOUR;
				}
				laser_mod_value += LASER_STEP; //next laser step
				if (laser_mod_value >= LASER_MAX_MOD) {//if no longer in range
					printf("Have completed absorption scan without detecting F=3 DIP.\r\n");
					printf("DIP_THRESHOLD: %u\r\n", DIP_THRESHOLD);
					sweep_time_s = (double) (stop_timer(SWEEP_TIMER)) / 1000000;
					printf("Sweep complete in %.3g s.\r\n", sweep_time_s);
					Error_Handler();
				}
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, laser_mod_value); //set DAC output pin
				//reset_adc_samples(); //reset ADC samples including sample count
				adc_average_updated = false; //clears new reading flag
				action_taken = true;
			}
			break;
		case LASER_RAMP_PHASE_FOUR: //finishing the current sweep
			if(adc_average_updated) {
				laser_mod_value += LASER_STEP; //next laser step
				if (laser_mod_value >= LASER_MAX_MOD) {//if no longer in range
					sweep_time_s = (double) (stop_timer(SWEEP_TIMER)) / 1000000;
					printf("Absorption spectroscopy complete in %.3g s.\r\n", sweep_time_s);
					printf("F=2 dip detected at step %u.\r\n", F2_mod_value);
					printf("F=3 dip detected at step %u.\r\n", F3_mod_value);
					HAL_GPIO_WritePin(SCOPE_TRIG_OUT_GPIO_Port, SCOPE_TRIG_OUT_Pin, GPIO_PIN_SET); // Resets trigger output
					if (LOCK_TO_DIP == 2) {
						laser_mod_value = F2_mod_value;
					} else if (LOCK_TO_DIP == 3) {
						laser_mod_value = F3_mod_value;
					} else {
						printf("Illegal DIP specified. LOCK_TO_DIP = %u.\r\n", LOCK_TO_DIP);
						Error_Handler();
					}
					printf("F=%u dip selected.\r\n", LOCK_TO_DIP);
					laser_state = LASER_TEMP_STABILISING;
				}
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, laser_mod_value); //set DAC output pin
				//reset_adc_samples(); //reset ADC samples including sample count
				adc_average_updated = false; //clears new reading flag
				action_taken = true;
			}
			break;
		case LASER_RAMP_PHASE_FIVE: //waiting for the LD temperature to stabilise
			if (check_timer(MW_TIMER) < 1000000) return(false); //Still waiting, no action taken
			action_taken = true;
			stop_timer(MW_TIMER); //release MW_timer
			laser_state = LASER_ON_FREQ;
			reset_adc_samples(); //reset ADC samples including sample count
		    printf("LD temperature stabilised.\r\n");

		default: // Other state
	       printf("laser_update has detected illegal state: %u \r\n", laser_state);
	       printf("local version: %u \r\n", local_copy_of_laser_state);
	}
    return(action_taken);
}
