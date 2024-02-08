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
//#include <stdbool.h>
//#include <string.h>
//#include <math.h>

/* Typedefs -----------------------------------------------------------*/

/* Defines ------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/
static const uint16_t LASER_STEP = 3; //circa 60uA @ a calculated 20.1uA/step
static uint16_t LASER_MOD_VALUE = 0; //can vary between 0 and 4095
static uint16_t F2_MOD_VALUE = 0; //for storing the current modulation value of the F=2 absorption dip
static uint16_t F3_MOD_VALUE = 0; //for storing the current modulation value of the F=3 absorption dip
static uint32_t max_adc_val = 0; // for temporary storage of the largest adc reading
static uint32_t min_adc_val = 4294967295; //for temporary storage of the smallest adc reading
extern volatile uint16_t sample_count; //counts number of times the sample line drives the ADC trigger high, updated when ADC completes conversion
extern uint32_t adc_val; //used to store adc3 readings

/* Function prototypes -----------------------------------------------*/
//__attribute__((section(".itcm"))) static uint32_t template_function(const uint32_t data, const bool verify);
//extern void Error_Handler(void);

/**
  * @brief  Function x.
  * @retval int
  */

void startlaserlockfunc1(const bool num_samples) {
	/* Requires ADC to be initialised and for HAL_ADC_ConvCpltCallback to be active */
	HAL_GPIO_WritePin(MW_INVALID_GPIO_Port, MW_INVALID_Pin, GPIO_PIN_SET); 	//Sets MW_invalid pin high to reset POP cycle
	HAL_Delay(10); // 10ms in case ADC was part-way through a conversion
	sample_count = 0; //reset sample count
	mw_sweep_settings.state = MW_STOPPED; //but need a function call to do this from a separate file
	start_timer(MW_TIMER); //reset MW_timer and start counting
	HAL_GPIO_WritePin(MW_INVALID_GPIO_Port, MW_INVALID_Pin, GPIO_PIN_RESET); //Restart POP cycle
	#ifdef MW_VERBOSE
		printf("POP calibration started\r\n");
	#endif //MW_VERBOSE
}

//MW_invalid low to ensure sample pulse is generated - not strictly needed if using FPGA pin17 output
HAL_GPIO_WritePin(MW_INVALID_GPIO_Port, MW_INVALID_Pin, GPIO_PIN_RESET); //Sets MW_invalid pin low
//laser_tuning must be high for probe on and MW off
HAL_GPIO_WritePin(LASER_TUNING_GPIO_Port, LASER_TUNING_Pin, GPIO_PIN_SET); // Laser_tuning output high

