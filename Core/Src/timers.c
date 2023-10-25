/**
 ******************************************************************************
 * @file           : timers.c
 * @brief          : Timer functionality including STM32-based POP signal generation
 * @authors		   : Simon J. Bale with modifications by Stuart Kenny
 ******************************************************************************
   * @attention
  *
  * Copyright (c) 2023 Stuart Kenny.
  * All rights reserved.
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "timers.h"
#include "main.h" //needed for port and timer definitions
#include "mw_gen.h" //needed for sweep_settings type definition
#include <stdio.h>
//#include <stdint.h>
#include <stdbool.h>
//#include <string.h>
//#include <math.h>

/* Typedefs -----------------------------------------------------------*/
//#define ATTENUATOR_CODE //include code related to AOM attenuator (SBJK clock)
#ifdef ATTENUATOR_CODE
struct AttenuatorSettings
{
    const GPIO_PinState ATT_0DB25 : 1;  // 1-bit unsigned field: 0.25 dB
    const GPIO_PinState ATT_0DB5 : 1;  // 1-bit unsigned field: 0.5 dB
    const GPIO_PinState ATT_1DB : 1;  // 1-bit unsigned field: 1 dB
    const GPIO_PinState ATT_2DB : 1;  // 1-bit unsigned field: 2 dB
    const GPIO_PinState ATT_4DB : 1;  // 1-bit unsigned field: 4 dB
    const GPIO_PinState ATT_8DB : 1;  // 1-bit unsigned field: 8 dB
    const GPIO_PinState ATT_16DB : 1;  // 1-bit unsigned field: 16 dB
};
#endif //ATTENUATOR_CODE

extern ADC_HandleTypeDef hadc3; //declared in main.c

/* Defines ------------------------------------------------------------*/
//#define TIMER_VERBOSE

/* Variables ---------------------------------------------------------*/
//Timers declared in main.c
extern HRTIM_HandleTypeDef hhrtim;
//LPTIM_HandleTypeDef hlptim1;
//TIM_HandleTypeDef htim1;
//TIM_HandleTypeDef htim3;

extern SweepSettings const sweep_settings; //declared in mw_gen.c

static volatile uint32_t pop_cycle_count = 0;
static volatile bool pop_running = false;

//Timers defined here but declared in main.h
//TIM1 is a 16-bit advanced control timer
TIM_TypeDef * SLOW_TIMER = TIM1; // Clocked at 100 kHz
//TIM3 is a 16-bit general purpose timer
TIM_TypeDef * FAST_TIMER = TIM3; // Clocked at 1 MHz
//TIM2 and TIM5 are 32-bit general purpose timers
TIM_TypeDef * MW_TIMER = TIM2; // Clocked 1MHz
TIM_TypeDef * SWEEP_TIMER = TIM5; // Clocked 1MHz
//TIM12 is a 16-bit general purpose timer
TIM_TypeDef * ETHERNET_TIMER = TIM12; // Clocked at 2 kHz

/* Function prototypes -----------------------------------------------*/
__attribute__((section(".itcm"))) uint32_t start_timer(TIM_TypeDef * timer);
__attribute__((section(".itcm"))) uint32_t stop_timer(TIM_TypeDef * timer);
__attribute__((section(".itcm"))) uint32_t check_timer(TIM_TypeDef *timer);
__attribute__((section(".itcm"))) void timer_delay(TIM_TypeDef *timer, uint32_t delay_us);
__attribute__((section(".itcm"))) uint32_t measure_POP_cycle(void);
__attribute__((section(".itcm"))) void start_pop();
__attribute__((section(".itcm"))) void stop_pop();
#ifdef ATTENUATOR_CODE
__attribute__((section(".itcm"))) static void set_aom_atten(const struct AttenuatorSettings a);
#endif //ATTENUATOR_CODE

/**
  * @brief  Function x.
  * @retval int
  */

#ifdef ATTENUATOR_CODE
static void set_aom_atten(const struct AttenuatorSettings a) {
	HAL_GPIO_WritePin(ATT_LE_GPIO_Port, ATT_LE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ATT_025_GPIO_Port, ATT_025_Pin, a.ATT_0DB25);
	HAL_GPIO_WritePin(ATT_05_GPIO_Port, ATT_05_Pin, a.ATT_0DB5);
	HAL_GPIO_WritePin(ATT_1_GPIO_Port, ATT_1_Pin, a.ATT_1DB);
	HAL_GPIO_WritePin(ATT_2_GPIO_Port, ATT_2_Pin, a.ATT_2DB);
	HAL_GPIO_WritePin(ATT_4_GPIO_Port, ATT_4_Pin, a.ATT_4DB);
	HAL_GPIO_WritePin(ATT_8_GPIO_Port, ATT_8_Pin, a.ATT_8DB);
	HAL_GPIO_WritePin(ATT_16_GPIO_Port, ATT_16_Pin, a.ATT_16DB);
}
#endif //ATTENUATOR_CODE

/**
  * @brief  Starts a timer.
  * @retval uint32_t
  */
uint32_t start_timer(TIM_TypeDef * timer) {

	timer->CR1 &= ~(TIM_CR1_CEN);
	timer->EGR |= TIM_EGR_UG;  // Reset CNT and PSC
	timer->CR1 |= TIM_CR1_CEN;
	//printf("Started timer with returned CNT value: %ld \r\n", timer->CNT);
	return timer->CNT;
}

/**
  * @brief  Stops a timer.
  * @retval uint32_t
  */
uint32_t stop_timer(TIM_TypeDef *timer) {

	timer->CR1 &= ~(TIM_CR1_CEN);
	return timer->CNT;
}

/**
  * @brief  Returns timer counter value
  * @param  Timer
  * @retval Counter value
  */
uint32_t check_timer(TIM_TypeDef *timer) {

	return timer->CNT;
}

/**
  * @brief  Uses a H/W timer to loop for the cycle count requested.
  */
void timer_delay(TIM_TypeDef *timer, const uint32_t delay_count){

	/* Note that we don't consider overflow.
	 * FAST_TIMER will take approximately 65 ms to overflow.
	 * SLOW_TIMER will take 650ms
	 * MW_TIMER and SWEEP_TIMER will take 71 minutes */

	uint32_t start = start_timer(timer);
//	timer->CR1 &= ~(TIM_CR1_CEN); // Disable the timer
//	timer->EGR |= TIM_EGR_UG;  // Reset CNT and PSC
//	timer->CR1 |= TIM_CR1_CEN; // Enable the timer
//	uint32_t start = timer->CNT; // Get the start value of the timer

//	while((timer->CNT - start) < delay_count){} // Loop until delay_us has expired
	while(timer->CNT < delay_count){} // Loop until delay_us has expired

	stop_timer(timer);
//	timer->CR1 &= ~(TIM_CR1_CEN); // Disable the timer

}

/**
  * @brief  Returns the measured period of a POP cycle as averaged over 20 cycles
  * @param  None
  * @retval Period expressed as an integer number of microseconds
  */
uint32_t measure_POP_cycle(void){

	/* Measures the elapsed time taken for 20 POP cycles
	 * Relies on the ADC value changing every time a sample is taken
	 * ADC must be initialised before running
	 */
	uint32_t adc_value = 0;
	uint32_t last_adc_value = 9999;
	uint8_t cycle_count = 0;
	uint32_t period;
	const uint8_t iterations = 20;

	HAL_GPIO_WritePin(MW_INVALID_GPIO_Port, MW_INVALID_Pin, GPIO_PIN_SET); 	//Sets MW_invalid pin high to reset POP cycle
	start_timer(MW_TIMER); //reset MW_timer and start counting
	HAL_GPIO_WritePin(MW_INVALID_GPIO_Port, MW_INVALID_Pin, GPIO_PIN_RESET); //Start POP cycle

	// get the ADC conversion value
	adc_value = HAL_ADC_GetValue(&hadc3);
	while (cycle_count < iterations) {
		while (adc_value == last_adc_value) {
			adc_value = HAL_ADC_GetValue(&hadc3); //keep reading ADC until value changes
		}
		last_adc_value = adc_value;
		cycle_count++;
	}

	uint32_t total_period = check_timer(MW_TIMER);
	period = (float)(check_timer(MW_TIMER)) / iterations + 0.5;
	stop_timer(MW_TIMER);
	#ifdef TIMER_VERBOSE
		printf("Time for %u POP cycles: %lu us\r\n", iterations, total_period);
		printf("POP period: %lu us\r\n", period);
	#endif //TIMER_VERBOSE
	return (period);

}



void stop_pop() {

	/* Timer A is the LASER enable, Timer E is the microwave pulse */
	if (HAL_HRTIM_WaveformOutputStop(&hhrtim,
	HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TE1) != HAL_OK) {
		printf("POP failure point A!\r\n");
		Error_Handler();
	}

	if (HAL_HRTIM_WaveformCounterStop_IT(&hhrtim,
	HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_E) != HAL_OK) {
		printf("POP failure point B!\r\n");
		Error_Handler();
	}

	pop_cycle_count = 0;
	pop_running = false;
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0); //turn off amber LED

	printf("POP cycle stopped!\r\n");

}

void start_pop() {

	/* Timer A is the LASER enable, Timer E is the microwave pulse */
	if (HAL_HRTIM_WaveformOutputStart(&hhrtim,
	HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TE1) != HAL_OK) {
		printf("Failed to start POP!\r\n");
		Error_Handler();
	}

#ifdef POP_START_PULSE

	if (HAL_HRTIM_WaveformSetOutputLevel(&hhrtim,
	HRTIM_TIMERINDEX_TIMER_A,
	HRTIM_OUTPUT_TA2, HRTIM_OUTPUTLEVEL_INACTIVE) != HAL_OK) {
		printf("POP failure point C!\r\n");
		Error_Handler();
	}

	timer_delay(SLOW_TIMER, 1000); //100ms delay

	if (HAL_HRTIM_WaveformSetOutputLevel(&hhrtim,
	HRTIM_TIMERINDEX_TIMER_A,
	HRTIM_OUTPUT_TA2, HRTIM_OUTPUTLEVEL_ACTIVE) != HAL_OK) {
		printf("POP failure point D!\r\n");
		Error_Handler();
	}

#endif

	if (HAL_HRTIM_WaveformCounterStart_IT(&hhrtim,
	HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_E) != HAL_OK) {
		printf("POP failure point E!\r\n");
		Error_Handler();
	}

	pop_running = true;

	printf("POP cycle running!\r\n");

}

//Simon was using this interrupt callback function to detect when the blue button was pressed
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//
////	static bool synth_init = false; //Simon declared here but not needed if Hittite initialised in main code.
//
//
////#ifdef RAMP_DAC
////	static bool dac_enabled = false;
////#endif
//
//	SystemClock_Config(); // We were in STOP mode so the HSI is selected.
//	HAL_ResumeTick();
//
////#ifdef RAMP_DAC
////	/* Start the DAC and zero its output */
////	if (!dac_enabled) {
////		if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) {
////			printf("Failure point F!\r\n");
////			Error_Handler();
////		}
////		if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0)
////				!= HAL_OK) {
////			printf("Failure point G!\r\n");
////			Error_Handler();
////		}
////		dac_enabled = true;
////	}
////#endif
//
////#ifdef SYNTH_ENABLE
////	if (!synth_init) {
////		if (init_synthesiser() != SUCCESS) {
////			printf("Synthesiser initialisation failed!\r\n");
////			Error_Handler();
////		}
////		synth_init = true;
////	}
////#endif
//
//	if (GPIO_Pin == GPIO_PIN_13) { // Blue button
//		printf("Blue button pressed....\r\n");
//
//		/* If the button is held down for more than one second then run the POP cycle */
//		HAL_Delay(1000);
//
//		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
//			printf("Long press\r\n");
//			if (pop_running) {
//				return;
//			}
//
//			start_pop();
//
//		} else {
//			printf("Short press\r\n");
//			/* We want to run CW so stop the POP cycle if it's running */
//			if (pop_running) {
//				stop_pop();
//				return;
//			}
//
//#ifdef ATTENUATOR_CODE
//			/* Set the attenuator for minimum attenuation */
//			const struct AttenuatorSettings attenuator_settings = {0,0,0,0,0,0,0}; // 0 dB
//			set_aom_atten(attenuator_settings);
//#endif //ATTENUATOR_CODE
//
//			/* Enable the AOM drive power */
//			if (HAL_HRTIM_WaveformOutputStart(&hhrtim,
//			HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TE1) != HAL_OK) {
//				printf("Failure point H!\r\n");
//				Error_Handler();
//			}
//
//			if (HAL_HRTIM_WaveformSetOutputLevel(&hhrtim,
//					HRTIM_TIMERINDEX_TIMER_A,
//					HRTIM_OUTPUT_TA1, HRTIM_OUTPUTLEVEL_INACTIVE) != HAL_OK) {
//				printf("Failure point I!\r\n");
//				Error_Handler();
//			}
//
//			/* Enable the Microwaves */
//			if (HAL_HRTIM_WaveformSetOutputLevel(&hhrtim,
//					HRTIM_TIMERINDEX_TIMER_E,
//					HRTIM_OUTPUT_TE1, HRTIM_OUTPUTLEVEL_ACTIVE) != HAL_OK) {
//				printf("Failure point J!\r\n");
//				Error_Handler();
//			};
//
//			/* Run the frequency sweep */
//			printf("Initiating sweep.\r\n");
//			while (1) {
//				HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET); //turn on red LED
//				run_sweep();
//				HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); //turn off red LED
//				printf("Sweep complete.\r\n");
//			}
//		}
//
//	}
//}

void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim){
	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin); //toggle green LED
}

void HAL_HRTIM_Compare2EventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx) {

	/* Called when the first microwave pulse goes low */
	if (TimerIdx == HRTIM_TIMERINDEX_TIMER_E) {
#ifdef ATTENUATOR_CODE
		/* Configure the LASER AOM drive attenuator */
		const struct AttenuatorSettings a = {0,0,0,0,0,1,0}; // 8 dB
		set_aom_atten(a);
#endif //ATTENUATOR_CODE
	}

}

void HAL_HRTIM_Compare3EventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx) {

	/* Called at the end of a POP cycle */
	if (TimerIdx == HRTIM_TIMERINDEX_TIMER_A) {
#ifdef ATTENUATOR_CODE
		/* Reset the attenuator to 0 dB */
		const struct AttenuatorSettings a = { 0, 0, 0, 0, 0, 0, 0 }; // 0 dB
		set_aom_atten(a);
#endif //ATTENUATOR_CODE

		const double start_freq = ((long)(sweep_settings.req_start_freq/sweep_settings.step_size)) * sweep_settings.step_size;
		const double stop_freq = ((long)((sweep_settings.req_stop_freq/sweep_settings.step_size) + 0.5)) * sweep_settings.step_size;
		const uint32_t num_points = ((stop_freq - start_freq)/sweep_settings.step_size) + 1;
		static uint32_t i = 0;

		/* Configure the Microwave frequency */
		if (i == num_points) {
			stop_pop();
			i = 0;
			start_pop();
		}

#ifdef SYNTH_ENABLE
		set_frequency_hz(start_freq + (i * sweep_settings.step_size));
#endif

		i = i + 1;

		pop_cycle_count = pop_cycle_count + 1;
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); //toggle amber LED
		printf("POP Cycle %lu done.\r\n", pop_cycle_count);

	}

	/* Called when the second microwave pulse goes high */
	if (TimerIdx == HRTIM_TIMERINDEX_TIMER_E) {
	}

}

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim,
		uint32_t TimerIdx) {

	/* Called at the start of the next POP cycle */
	if (TimerIdx == HRTIM_TIMERINDEX_TIMER_A) {
	}
}
