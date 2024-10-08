/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * @author		   : Simon J. Bale
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mw_gen.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ATTENUATOR_CODE //include code related to AOM attenuator (SBJK clock)
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
<<<<<<< Updated upstream
#define SYNTH_ENABLE
=======
//#define SYNTH_ENABLE
//#define TELNET_ENABLE
>>>>>>> Stashed changes
#define POP_START_PULSE
//#define QUANTIFY_ADC_NOISE
#define MW_VERBOSE

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac1;

ETH_HandleTypeDef heth;

HRTIM_HandleTypeDef hhrtim;

LPTIM_HandleTypeDef hlptim1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//Timers defined here but declared in main.h
TIM_TypeDef * SLOW_TIMER = TIM1; // Clocked at 10 kHz
TIM_TypeDef * FAST_TIMER = TIM3; // Clocked at 100 kHz
static const uint32_t ERROR_LED_DELAY = 1000; // 100 ms

extern SweepSettings const sweep_settings;

extern uint32_t _siitcm;
extern uint32_t _sitcm;
extern uint32_t _eitcm;

static volatile bool pop_running = false;
static volatile bool mw_sweep_started = false;
static volatile uint32_t pop_cycle_count = 0;
volatile bool blue_button_status; //blue button state. 1 when pressed
//static volatile bool pin_status; //blue button state. 1 when pressed
//static bool last_pin_status; //previous blue button state
static uint8_t MW_power = 0x1; // Initial MW power i.e. LO2GAIN
//3 is max, 0 is min, log scale with 2dB between points
//max is +5dBm output, min is -1dBm out.
//NOTE - these values are measured and not consistent with datasheet
uint32_t adc_val; //used to store adc3 readings
#ifdef QUANTIFY_ADC_NOISE
uint32_t adc_max, adc_min; //used to store adc3 readings
#endif //QUANTIFY_ADC_NOISE
uint32_t dac_val; //for dac1 output channel 1

<<<<<<< Updated upstream
=======
const double HYPERFINE = 3035736939; //Rb85 hyperfine frequency
//static const double MW_DELTA = -1817; //MW offset
static const double MW_DELTA = 1000; //MW offset
>>>>>>> Stashed changes

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_DAC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_HRTIM_Init(void);
static void MX_ADC3_Init(void);
static void MX_ETH_Init(void);
/* USER CODE BEGIN PFP */

extern uint32_t set_MW_power (const uint8_t mw_power);
extern uint32_t init_synthesiser(const uint8_t mw_power);
extern void set_frequency_hz(const double fo);
extern void run_sweep();
extern void MW_frequency_toggle (const double f_one, const double f_two);
<<<<<<< Updated upstream
__attribute__((section(".itcm"))) uint32_t start_timer(TIM_TypeDef * timer);
__attribute__((section(".itcm"))) uint32_t stop_timer(TIM_TypeDef * timer);
__attribute__((section(".itcm"))) void timer_delay(TIM_TypeDef *timer, uint32_t delay_us);
__attribute__((section(".itcm"))) static void start_pop();
__attribute__((section(".itcm"))) static void stop_pop();
#ifdef ATTENUATOR_CODE
__attribute__((section(".itcm"))) static void set_aom_atten(const struct AttenuatorSettings a);
#endif //ATTENUATOR_CODE
=======
//extern uint32_t start_timer(TIM_TypeDef * timer);
//extern uint32_t stop_timer(TIM_TypeDef * timer);
extern void timer_delay(TIM_TypeDef *timer, uint32_t delay_us);
extern uint32_t start_timer(TIM_TypeDef * timer);
extern uint32_t stop_timer(TIM_TypeDef * timer);
extern uint32_t check_timer(TIM_TypeDef *timer);
//extern static void start_pop();
//extern static void stop_pop();
extern bool calc_defined_step_MW_sweep(const double centre_freq, const double span, const uint32_t pop_cycles_per_step, const uint32_t num_points_req);
extern bool calc_fixed_time_MW_sweep(const double centre_freq, const double span, const double requested_sweep_period, const bool scope_sync_time);
extern const bool MW_update(void);
extern void start_POP_calibration(const bool cal_only);
extern void start_continuous_MW_sweep(void);
extern uint32_t measure_POP_cycle(void);
//extern void initiate_MW_calibration_sweep(const uint32_t POP_period_us);
>>>>>>> Stashed changes

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

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

uint32_t start_timer(TIM_TypeDef * timer) {

	timer->CR1 &= ~(TIM_CR1_CEN);
	timer->EGR |= TIM_EGR_UG;  // Reset CNT and PSC
	timer->CR1 |= TIM_CR1_CEN;
	return timer->CNT;

}

uint32_t stop_timer(TIM_TypeDef *timer) {

	timer->CR1 &= ~(TIM_CR1_CEN);
	return timer->CNT;
}

void timer_delay(TIM_TypeDef *timer, const uint32_t delay_count){

	timer->CR1 &= ~(TIM_CR1_CEN); // Disable the timer
	timer->EGR |= TIM_EGR_UG;  // Reset CNT and PSC
	timer->CR1 |= TIM_CR1_CEN; // Enable the timer
	uint32_t start = timer->CNT; // Get the start value of the timer

	/* Note that we don't consider overflow, if the timer is clocked at 1 MHz
	 * a 16 bit counter will take approximately 65 ms to overflow. */

	while((timer->CNT - start) < delay_count){} // Loop until delay_us has expired
	timer->CR1 &= ~(TIM_CR1_CEN); // Disable the timer

}

static void stop_pop() {

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

static void start_pop() {

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

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { //Simon was using this interrupt callback function to detect when the blue button was pressed
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
//		static const double start_freq = ((long)(sweep_settings.req_start_freq/sweep_settings.step_size)) * sweep_settings.step_size;
//		static const double stop_freq = ((long)((sweep_settings.req_stop_freq/sweep_settings.step_size) + 0.5)) * sweep_settings.step_size;
//		static const uint32_t num_points = ((stop_freq - start_freq)/sweep_settings.step_size) + 1;
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* Copy from FLASH to itcm */
  memcpy(&_sitcm, &_siitcm, ((void*) &_eitcm - (void*) &_sitcm));

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPTIM1_Init();
  MX_DAC1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_HRTIM_Init();
  MX_ADC3_Init();
  MX_ETH_Init();
  /* USER CODE BEGIN 2 */
  printf("\033c"); //clears screen
  printf("Atomic Clock - Source __TIMESTAMP__: %s\r\n", __TIMESTAMP__);

#ifdef SYNTH_ENABLE
	if (init_synthesiser(MW_power) != SUCCESS) {
		printf("Synthesiser initialisation failed!\r\n");
		Error_Handler();
	}
#ifdef MW_VERBOSE
	printf("LO2GAIN set at: 0x%x \r\n", MW_power);
#endif	//MW_VERBOSE
#endif //SYNTH_ENABLE

	/* Start a low power timer to flash an LED approximately every second */
	if (HAL_LPTIM_Counter_Start_IT(&hlptim1, 1024) != HAL_OK) {
		printf("Failed to start slow flashing LED!\r\n");
		Error_Handler();
	}

	/* Start the DAC and zero its output */
	if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) {
		printf("Failure to initialise DAC \r\n");
		Error_Handler();
	}
	printf("Setting DAC output to 1.00V \r\n");
	if(HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1241) != HAL_OK){
			printf("DAC setup failed!\r\n");
		Error_Handler();
	}

	/* Output used for triggering external scope */
//	HAL_GPIO_WritePin(SCOPE_TRIG_OUT_GPIO_Port, SCOPE_TRIG_OUT_Pin, GPIO_PIN_SET); // Sets trigger output high
//	printf("Setting trigger output high \r\n");
//	HAL_GPIO_WritePin(SCOPE_TRIG_OUT_GPIO_Port, SCOPE_TRIG_OUT_Pin, GPIO_PIN_RESET); // Sets trigger output low
//	printf("Setting trigger output low \r\n");

	/* Spare SMA pin */
//	SPARE_OUT_GPIO_Port, SPARE_OUT_Pin
//	HAL_GPIO_WritePin(SPARE_OUT_GPIO_Port, SPARE_OUT_Pin, GPIO_PIN_SET); // Sets spare SMA output high
//	printf("Setting spare SMA output high \r\n");
//	HAL_GPIO_WritePin(SPARE_OUT_GPIO_Port, SPARE_OUT_Pin, GPIO_PIN_RESET); // Sets spare SMA output low

	/* Laser tuning pin */
//	LASER_TUNING_GPIO_Port, LASER_TUNING_Pin
	HAL_GPIO_WritePin(LASER_TUNING_GPIO_Port, LASER_TUNING_Pin, GPIO_PIN_SET); // Laser_tuning output high
	printf("Requesting FPGA CW absorption \r\n");
//	HAL_GPIO_WritePin(LASER_TUNING_GPIO_Port, LASER_TUNING_Pin, GPIO_PIN_RESET); // Laser_tuning SMA output low

	/* MW invalid */
//	MW_INVALID_GPIO_Port, MW_INVALID_Pin
//	HAL_GPIO_WritePin(MW_INVALID_GPIO_Port, MW_INVALID_Pin, GPIO_PIN_SET); // MW_invalid output high
	printf("Setting MW invalid output low \r\n");
	HAL_GPIO_WritePin(MW_INVALID_GPIO_Port, MW_INVALID_Pin, GPIO_PIN_RESET); // MW_invalid output low

	/* Fire up the ADC */
	// external trigger, single conversion selected in ioc file
	// calibrate ADC for better accuracy and start it w/ interrupt
	if(HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK){
		printf("ADC calibration failure \r\n");
		Error_Handler();
	}
	printf("ADC calibrated successfully \r\n");
	//Start the ADC with interrupts enabled
	if(HAL_ADC_Start_IT(&hadc3) != HAL_OK){
		printf("Failed to start ADC with interrupt capability \r\n");
	                Error_Handler();
	}
	printf("ADC interrupt callback enabled \r\n");
#ifdef QUANTIFY_ADC_NOISE
	adc_max = 0;
	adc_min = 60000;
#endif //QUANTIFY_ADC_NOISE

//	pin_status = HAL_GPIO_ReadPin(BLUE_BUTTON_GPIO_Port, BLUE_BUTTON_Pin);
//	printf("Blue button status: %u \r\n", pin_status);
//	last_pin_status = pin_status;
<<<<<<< Updated upstream
=======

//	timer_delay(MW_TIMER, 7000);
//	timer_delay(MW_TIMER, 50000);

//	start_timer(SWEEP_TIMER); //reset SWEEP_TIMER and start counting
//	while (!is_telnet_initialised() && (check_timer(SWEEP_TIMER) < 15000000)) {
//		//loop here until telnet is initialised or 15s has elapsed
////		printf("Waiting for telnet to initialise\r\n");
////		printf("Telnet initialisation status %u, SWEEP_TIMER value %lu \r\n", is_telnet_initialised(), check_timer(SWEEP_TIMER));
//	}
//	stop_timer(SWEEP_TIMER); //stop SWEEP_TIMER
//	printf("Telnet initialisation status %u, SWEEP_TIMER value %lu \r\n", is_telnet_initialised(), check_timer(SWEEP_TIMER));

//	printf("Sending test packets\r\n");
//	one_off();

>>>>>>> Stashed changes
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		//POP can run in SLEEP mode
		//
//		if (!pop_running) {
//			HAL_SuspendTick(); // Needs to be paused or the interrupt will bring us out of STOP mode.
//			HAL_PWREx_EnableFlashPowerDown();
//			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI); // We will only resume when an interrupt occurs
//		} else {
//			HAL_SuspendTick(); // Needs to be paused or the interrupt will bring us out of SLEEP mode.
//			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI); // We will only resume when an interrupt occurs.
//		}

//		pin_status = HAL_GPIO_ReadPin(BLUE_BUTTON_GPIO_Port, BLUE_BUTTON_Pin);
//		if (pin_status != last_pin_status) {
//			printf("Blue button status: %u \r\n", pin_status);
//			last_pin_status = pin_status;
//		}

		blue_button_status = HAL_GPIO_ReadPin(BLUE_BUTTON_GPIO_Port, BLUE_BUTTON_Pin);
		if (blue_button_status) {// If blue button is pressed
			printf("Blue button pressed....\r\n");
<<<<<<< Updated upstream
			printf("Requesting FPGA POP \r\n");
			HAL_GPIO_WritePin(LASER_TUNING_GPIO_Port, LASER_TUNING_Pin, GPIO_PIN_RESET); // Laser_tuning SMA output low

			/* CODE FOR CHARACTERISING MW GENERATOR FREQUENCY SETTLING TIME */
			//MW_frequency_toggle (3035735189, 3035734189); //infinite loop toggling between centre of DR dip and 1kHz left of dip
			//MW_frequency_toggle (3035735189, 3035736189); //infinite loop toggling between centre of DR dip and 1 kHz right of dip
			//set_MW_power(0x03); //set maximum MW power to improve contrast
			//MW_frequency_toggle (3035733689, 3035733789); //infinite loop toggling 100Hz on left of DR dip
			//MW_frequency_toggle (3035733689, 3035733699); //infinite loop toggling 10Hz on left of DR dip

			//change the MW power each time the button is pressed, unless it's the first time round this loop
			if (mw_sweep_started) {
				++MW_power; //increase MW_power value by 1
				if (MW_power>3) { //Loop MW_power back round to 0 if above maximum permissible value i.e. 3
					MW_power = 0;
				}
				set_MW_power(MW_power);
#ifdef MW_VERBOSE
				printf("LO2GAIN changed to: 0x%x \r\n", MW_power);
#endif //MW_VERBOSE
			} else {
					printf("Initiating sweep.\r\n");
					mw_sweep_started = true;
			}
=======
//			HAL_GPIO_WritePin(LASER_TUNING_GPIO_Port, LASER_TUNING_Pin, GPIO_PIN_RESET); // Laser_tuning SMA output low
//
//			/* CODE FOR CHARACTERISING MW GENERATOR FREQUENCY SETTLING TIME */
//			//MW_frequency_toggle (3000000010, 3000010010); //infinite loop toggling between centre of DR dip and 100kHz left of dip
//			//MW_frequency_toggle (3035735189, 3035734189); //infinite loop toggling between centre of DR dip and 1kHz left of dip
//			//MW_frequency_toggle (3035735189, 3035736189); //infinite loop toggling between centre of DR dip and 1 kHz right of dip
//			//set_MW_power(0x03); //set maximum MW power to improve contrast
//			//MW_frequency_toggle (3035733689, 3035733789); //infinite loop toggling 100Hz on left of DR dip
//			//MW_frequency_toggle (3035733689, 3035733699); //infinite loop toggling 10Hz on left of DR dip
//
//			//change the MW power each time the button is pressed, unless it's the first time round this loop
//			if (mw_sweep_started) {
//				++MW_power; //increase MW_power value by 1
//				if (MW_power>3) { //Loop MW_power back round to 0 if above maximum permissible value i.e. 3
//					MW_power = 0;
//				}
//				set_MW_power(MW_power);
//			#ifdef MW_VERBOSE
//				printf("LO2GAIN changed to: 0x%x \r\n", MW_power);
//			#endif //MW_VERBOSE
//			} else {
//				printf("Initiating sweep.\r\n");
//				mw_sweep_started = true;
//				start_continuous_MW_sweep();
//			}
>>>>>>> Stashed changes
			while(blue_button_status) {//remain here polling button until it is released
				timer_delay(SLOW_TIMER, 100); //10ms delay
				blue_button_status = HAL_GPIO_ReadPin(BLUE_BUTTON_GPIO_Port, BLUE_BUTTON_Pin);
			}
		}

		if (mw_sweep_started) {//won't execute until the first time the blue button is pressed
			/* Run the frequency sweep */
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET); //turn on red LED
			run_sweep();
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); //turn off red LED
			//printf("Sweep complete.\r\n");
			printf("LO2GAIN: 0x%x \r\n", MW_power);
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV16;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_LSE;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,32768);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIG_EXT_IT11;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */
  /** DAC auto-calibration
  */
//  if (HAL_DACEx_SelfCalibrate(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
//  {
//    printf("Failed to auto-calibrate DAC \r\n");
//    Error_Handler();
//  }

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief HRTIM Initialization Function
  * @param None
  * @retval None
  */
static void MX_HRTIM_Init(void)
{

  /* USER CODE BEGIN HRTIM_Init 0 */

  /* USER CODE END HRTIM_Init 0 */

  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM_Init 1 */

  /* USER CODE END HRTIM_Init 1 */
  hhrtim.Instance = HRTIM1;
  hhrtim.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = PUMP_WIDTH+(2*MICROWAVE_DELAY)+(2*MICROWAVE_WIDTH)+RAMSEY_TIME+PROBE_WIDTH+POP_CYCLE_DELAY;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_DIV4;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_CMP3|HRTIM_TIM_IT_REP;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_CMP2|HRTIM_TIM_IT_CMP3;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_D_E_DELAYEDPROTECTION_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_E, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = PUMP_WIDTH;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = PUMP_WIDTH+(2*MICROWAVE_DELAY)+(2*MICROWAVE_WIDTH)+RAMSEY_TIME;
  pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  pCompareCfg.AutoDelayedTimeout = 0x0000;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = PUMP_WIDTH+MICROWAVE_DELAY+MICROWAVE_WIDTH;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = PUMP_WIDTH+(2*MICROWAVE_DELAY)+(2*MICROWAVE_WIDTH)+RAMSEY_TIME+PROBE_WIDTH;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = PUMP_WIDTH+MICROWAVE_DELAY+MICROWAVE_WIDTH+RAMSEY_TIME+MICROWAVE_WIDTH;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_4, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_LOW;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP1|HRTIM_OUTPUTSET_TIMCMP3;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP2|HRTIM_OUTPUTRESET_TIMPER;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_ACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP2|HRTIM_OUTPUTRESET_TIMCMP4;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_LOW;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP3;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP2;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_ACTIVE;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_E, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = PUMP_WIDTH+MICROWAVE_DELAY;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = PUMP_WIDTH+MICROWAVE_DELAY+MICROWAVE_WIDTH+RAMSEY_TIME;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM_Init 2 */

  /* USER CODE END HRTIM_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim);

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV32;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1249;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 124;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 1000000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ATT_4_Pin|ATT_8_Pin|ATT_16_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|MW_INVALID_Pin|LASER_TUNING_Pin|LD3_Pin
                          |SPARE_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SCOPE_TRIG_OUT_GPIO_Port, SCOPE_TRIG_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SCLK_Pin|MOSI_Pin|REG_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, ATT_2_Pin|ATT_1_Pin|ATT_05_Pin|ATT_025_Pin
                          |ATT_LE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ATT_4_Pin ATT_8_Pin ATT_16_Pin LD2_Pin */
  GPIO_InitStruct.Pin = ATT_4_Pin|ATT_8_Pin|ATT_16_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BLUE_BUTTON_Pin */
  GPIO_InitStruct.Pin = BLUE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLUE_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin MW_INVALID_Pin LASER_TUNING_Pin LD3_Pin
                           SPARE_OUT_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|MW_INVALID_Pin|LASER_TUNING_Pin|LD3_Pin
                          |SPARE_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PF11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : SCOPE_TRIG_OUT_Pin */
  GPIO_InitStruct.Pin = SCOPE_TRIG_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SCOPE_TRIG_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin ATT_2_Pin ATT_1_Pin ATT_05_Pin
                           ATT_025_Pin ATT_LE_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|ATT_2_Pin|ATT_1_Pin|ATT_05_Pin
                          |ATT_025_Pin|ATT_LE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCLK_Pin MOSI_Pin SEN_Pin REG_EN_Pin */
  GPIO_InitStruct.Pin = SCLK_Pin|MOSI_Pin|SEN_Pin|REG_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MISO_Pin */
  GPIO_InitStruct.Pin = MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MISO_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  adc_val = HAL_ADC_GetValue(&hadc3);
  //printf("ADC value: %lu \r\n", adc_val);
  dac_val = adc_val >> 4;
#ifdef QUANTIFY_ADC_NOISE
  if (adc_val < adc_min) {
	  adc_min = adc_val;
	  printf("ADC reading: %lu, max: %lu, min: %lu \r\n", adc_val, adc_max, adc_min);
  }
  if (adc_val > adc_max) {
	  adc_max = adc_val;
	  printf("ADC reading: %lu, max: %lu, min: %lu \r\n", adc_val, adc_max, adc_min);
  }
  //printf("ADC reading: %lu, max: %lu, min: %lu \r\n", adc_val, adc_max, adc_min);
#endif //QUANTIFY_ADC_NOISE
  //printf("ADC value: %lu, DAC value: %lu \r\n", adc_val, dac_val);
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_val);
  //HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	__disable_irq();

	printf("Error handler routine called\r\n");

	/* Disable the AOM */
	HAL_HRTIM_WaveformOutputStop(&hhrtim, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TE1);
	HAL_HRTIM_WaveformCounterStop_IT(&hhrtim, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_E);

	/* Power down the synthesiser */
	HAL_GPIO_WritePin(REG_EN_GPIO_Port, REG_EN_Pin, 0);

	while (1) {
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin); //toggle red LED
		timer_delay(SLOW_TIMER, ERROR_LED_DELAY);
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
