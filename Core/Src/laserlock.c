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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
//#include <string.h>
//#include <math.h>

/* Typedefs -----------------------------------------------------------*/

/* Defines ------------------------------------------------------------*/
#define LASER_MIN_MOD 0 //read-only
#define LASER_MAX_MOD 1735 //corresponds to 1.4V DAC output for 35mA current
#define LOCK_TO_DIP 2 //should be 2 or 3 to select the F=? absorption dip

/* laser states */
enum laser_states
{
  LASER_ON_FREQ = 0,
  LASER_STEPPED_UP,
  LASER_STEPPED_DOWN,
  LASER_RAMPING
};

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
extern bool adc_average_updated; //allows polling without needing to compare previous values
extern uint32_t start_timer(TIM_TypeDef * timer);
extern uint32_t stop_timer(TIM_TypeDef * timer);
extern uint32_t check_timer(TIM_TypeDef *timer);

/* Function prototypes -----------------------------------------------*/
//__attribute__((section(".itcm"))) static uint32_t template_function(const uint32_t data, const bool verify);
//extern void Error_Handler(void);
extern void reset_adc_samples(void);
extern void stop_MW_operation(void);

/**
  * @brief  Function x.
  * @retval int
  */

void startlaserlockfunc1(const bool num_samples) {
	/* Requires ADC to be initialised and for HAL_ADC_ConvCpltCallback to be active */
	//MW_invalid low to ensure sample pulse is generated - not strictly needed in FPGA state 0
	HAL_GPIO_WritePin(MW_INVALID_GPIO_Port, MW_INVALID_Pin, GPIO_PIN_RESET); //Sets MW_invalid pin low
	//laser_tuning (FPGA input pin 18) must be high for probe on and MW off
	HAL_GPIO_WritePin(LASER_TUNING_GPIO_Port, LASER_TUNING_Pin, GPIO_PIN_SET); // Laser_tuning output high
	reset_adc_samples(); //reset ADC samples including sample count

	HAL_GPIO_WritePin(MW_INVALID_GPIO_Port, MW_INVALID_Pin, GPIO_PIN_SET); 	//Sets MW_invalid pin high to reset POP cycle
	HAL_Delay(10); // 10ms in case ADC was part-way through a conversion
	reset_adc_samples(); //reset ADC samples including sample count
	laser_state = LASER_RAMPING;
	stop_MW_operation();
	start_timer(MW_TIMER); //reset MW_timer and start counting
	HAL_GPIO_WritePin(MW_INVALID_GPIO_Port, MW_INVALID_Pin, GPIO_PIN_RESET); //Restart POP cycle
	#ifdef MW_VERBOSE
		printf("POP calibration started\r\n");
	#endif //MW_VERBOSE
}



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
	bool action_taken = false;
	if (laser_state == LASER_ON_FREQ) {
		return(action_taken); //no action to be taken - return zero
	}
	if (!adc_average_updated) {
		return(action_taken); //no action to be taken - return zero
	}
	/* Action needed - laser tuning is happening and ADC reading is valid */
	uint8_t local_copy_of_laser_state = laser_state; //hack to make switch statement behave
	switch (local_copy_of_laser_state)
	{
		case LASER_ON_FREQ:
			break; //no action to take

		case LASER_STEPPED_UP:
			action_taken = true;
			break;
		case LASER_STEPPED_DOWN:
		case LASER_RAMPING:

		default: // Other state
	       printf("laser_update has detected illegal state: %u \r\n", laser_state);
	       printf("local version: %u \r\n", local_copy_of_laser_state);
	}
    return(action_taken);
}
