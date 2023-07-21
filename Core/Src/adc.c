/**
 ******************************************************************************
 * @file           : adc.c
 * @brief          : ADC related functions
 * @author		   : Stuart Kenny
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"
#include "main.h" //needed for port definitions
//#include <stdio.h>
//#include <stdint.h>
//#include <stdbool.h>
//#include <string.h>
//#include <math.h>

/* Private typedef -----------------------------------------------------------*/
extern ADC_HandleTypeDef hadc3; //declared in main.c
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
//__attribute__((section(".itcm"))) static uint32_t synth_writereg(const uint32_t data, const uint32_t reg_address, const uint32_t chip_address, const bool verify);
//extern void Error_Handler(void);


	  // start ADC convertion
//	  HAL_ADC_Start(&hadc3);
	  // ADC poll for conversion
//	  HAL_ADC_PollForConversion(&hadc3, 100);
//	  // get the ADC conversion value
//	  adc_value = HAL_ADC_GetValue(&hadc3);
//	  // end ADC convertion
//	  HAL_ADC_Stop(&hadc3);
//	  // convert ADC value into voltage
//	  voltage = (adc_value*3.3)/4096;
//	  // convert the voltage into temperature
//	  temp = voltage*100;
