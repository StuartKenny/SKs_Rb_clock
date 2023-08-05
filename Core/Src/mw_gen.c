/**
 ******************************************************************************
 * @file           : template.c
 * @brief          : Hittite HMC835 driver & MW sweep generation
 * @authors		   : Simon J. Bale with updates and modifications by Stuart Kenny
 ******************************************************************************
   * @attention
  *
  * Copyright (c) 2023 Simon J. Bale & Stuart Kenny.
  * All rights reserved.
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mw_gen.h"
#include "main.h" //needed for port and timer definitions
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define SYNTH_ENABLE
//#define SPI_DEBUG
//#define MW_F_CHECK
//#define MW_VERBOSE
//#define RAMP_DAC // Use a DAC output to represent increasing MW frequency
#define HALT_ON_LOSS_OF_LOCK
#define VERIFY 1 //constant, don't touch
#define DONT_VERIFY 0 //constant, don't touch
#define OPTIMISED_FOR_3_035GHZ_GENERATION //skips calculation of k as k is always 1

/* HMC835LP6GE Register addresses */
#define ID_REGISTER 0x00 //read-only
#define OPEN_MODE_READ_ADDRESS 0x00 //write-only
#define REFDIV_REGISTER 0x02
#define INTEGER_FREQUENCY_REGISTER 0x03
#define FRACTIONAL_FREQUENCY_REGISTER 0x04
#define LOCK_DETECT_REGISTER 0x07
#define ANALOG_EN_REGISTER 0x08
#define GPO_REGISTER 0x0F
#define GPOLD_REGISTER 0x12
#define GAIN_DIVIDER_REGISTER 0x16
#define MODES_REGISTER 0x17

/* MW states */
enum mw_states
{
  MW_STOPPED = 0,
  MW_FIXED_FREQ, //
  MW_STABILISING, //waiting for MW output to stabilise
  MW_DWELL //MW output valid
};

struct MW_struct
{
  uint8_t state;             /* current MW state */
  uint8_t k;
  uint32_t NINT;
  uint32_t NFRAC_start; //ramp starting value of fractional register
  uint32_t num_steps;
  uint32_t step_size; //in multiples of minimum step
  uint32_t stabilise_time;
  uint32_t dwell_time;
  uint32_t current_point; //keeps track of which point
};

//MW_struct mw_sweep_settings
struct MW_struct mw_sweep_settings;  //create a structure to store the sweep settings

/* Private variables ---------------------------------------------------------*/
//Timers are declared in main.h and defined in main.c
static const uint32_t SYNTH_SPI_BITS = 32;
static const uint32_t SYNTH_ID = 0xC7701A;

//The following timer constants are used to program a 16-bit register i.e. 65535 max
//static const uint32_t MW_STABILISE_TIME_US = 10000; // 10ms for MW output to stabilise before signalling FPGA
static const uint32_t MW_STABILISE_TIME_US = 5000; // 5ms for MW output to stabilise before signalling FPGA
//static const uint32_t DWELL_TIME_US = 4360; // (4.36ms + 10ms MW_stabilise + measured 3.8ms processing) x 1679 steps for 30.5s ramp
//static const uint32_t DWELL_TIME_US = 1000; // (1ms + 1ms MW_stabilise + measured 0.86ms processing) x 1679 steps for 4.8s ramp
//static const uint32_t DWELL_TIME_US = 1180; // (1.2ms + 1ms MW_stabilise + measured 0.86ms processing) x 1679 steps for 5.1s ramp
//static const uint32_t DWELL_TIME_US = 12000; //
static const uint32_t POP_CYCLE_TIME_US = 21204; //21.2ms (53011 x 400ns cycles)
//static const uint32_t MW_PROCESSING_TIME = 11270; //measured at 11.27ms
static const uint32_t MW_PROCESSING_TIME = 8140; //Calculated value for fixed time, 1150 steps

static const double VCO_MAX_FREQ = 4100E6;
//static const double VCO_MIN_FREQ = 2050E6;
static const double REF_FREQ = 50E6;

//static const uint8_t LO2GAIN = 0x3; // 3 is max, 0 is min, log scale with 2dB between points
//max is +5dBm output, min is -1dBm out.
//NOTE - these values are measured and not consistent with datasheet
static const double HYPERFINE = 3035736939; //Rb85 hyperfine frequency

/* Mute MW generation whilst changing frequency
 * None of these appear to work particularly well all require 9-10ms to stabilise afterwards.
 * If both are disabled then stabilising time is slightly improved */
static const bool AUTO_MUTE = true; //0 is disabled, 1 is enabled
//static const bool MANUAL_MUTE = true; //0 is disabled, 1 is enabled. Approx 9ms to re-stabilise MW output if enabled

/* MW sweep settings {req_start_freq, req_stop_freq, step_size}
 * Requested frequencies will potentially be tweaked into values that can be programmed into the HMC835
 * Step size must be a multiple of 2.98Hz (see datasheet S1.3.7.4.4 on fractional tuning)
 */
//For 5kHz sweep, 2.98Hz x 1679 steps, centred around 3.035736939GHz
SweepSettings const sweep_settings = {3.0357344390E9, 3.0357394390e9, (50e6 / (1 << 24)) }; //SB original settings
//For 5kHz sweep, 2.98Hz x 1679 steps, centred around 3.035735189GHz
//SweepSettings const sweep_settings = {3.035732689E9, 3.035737689E9, (50e6 / (1 << 24)) }; //SK settings based on measured centre of DR
//For 10kHz sweep, 5.96Hz x 1679 steps, centred around 3.035736939GHz
//SweepSettings const sweep_settings = {3.0357319390E9, 3.0357419390e9, (50e6 / (1 << 23)) };
//For 10kHz sweep, 190.7Hz step, centred around 3.035736939GHz
//SweepSettings const sweep_settings = {3.0357319390E9, 3.0357419390e9, (50e6 / (1 << 18)) };

extern DAC_HandleTypeDef hdac1; //declared in main.c
extern volatile bool blue_button_status; //declared in main.c

/* Private function prototypes -----------------------------------------------*/
__attribute__((section(".itcm"))) static uint32_t synth_writereg(const uint32_t data, const uint32_t reg_address, const uint32_t chip_address, const bool verify);
__attribute__((section(".itcm"))) static uint32_t synth_readreg(const uint32_t reg_address);
__attribute__((section(".itcm"))) uint32_t set_MW_power (const uint8_t mw_power);
__attribute__((section(".itcm"))) uint32_t init_synthesiser(const uint8_t mw_power);
//__attribute__((section(".itcm"))) static const bool poll_until_locked(uint32_t timeout);
__attribute__((section(".itcm"))) static const bool lock_status(void);
//__attribute__((section(".itcm"))) static void mute_mw_outputs();
//__attribute__((section(".itcm"))) static void set_frequency(const uint32_t integer, const uint32_t fraction, const uint32_t vco_divider, bool mute);
__attribute__((section(".itcm"))) void set_frequency_hz(const double fo);
__attribute__((section(".itcm"))) static void set_freq_regs(const uint32_t integer, const uint32_t fraction, const uint32_t vco_divider);
__attribute__((section(".itcm"))) void run_sweep();
__attribute__((section(".itcm"))) static void print_mw_sweep_settings (void);
__attribute__((section(".itcm"))) bool calc_defined_step_MW_sweep(const double centre_freq, const double span, const uint32_t pop_cycles_per_step, const uint32_t num_points_req);
__attribute__((section(".itcm"))) bool calc_fixed_time_MW_sweep(const double centre_freq, const double span, const double requested_sweep_period, const bool scope_sync_time);
__attribute__((section(".itcm"))) static const uint32_t calculate_k(const double frequency);
__attribute__((section(".itcm"))) const bool start_MW_sweep(void);
__attribute__((section(".itcm"))) const bool MW_update(void);
__attribute__((section(".itcm"))) void MW_frequency_toggle (const double f_one, const double f_two);
__attribute__((section(".itcm"))) void set_SDO_output(const uint32_t GPO_setting);
extern uint32_t start_timer(TIM_TypeDef * timer);
extern uint32_t stop_timer(TIM_TypeDef * timer);
extern uint32_t check_timer(TIM_TypeDef *timer);
extern void timer_delay(TIM_TypeDef *timer, uint32_t delay_us);
extern void Error_Handler(void);

/**
  * @brief  Writes to a register over SPI.
  * @param  Data
  * @param  Address
  * @param  Chip address
  * @param  Verify
  * @retval Contents read back from register
  */
static uint32_t synth_writereg(const uint32_t data, const uint32_t reg_address, const uint32_t chip_address, const bool verify) {

	uint32_t read_data = 0;
	const uint32_t write_data = (data << 8) | (reg_address << 3) | chip_address; // This is what we will write, 32 bits in total.
#ifdef SPI_DEBUG
	printf("SPI BYTES WRITTEN: 0x%X \r\n", write_data);
#endif
	HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, 0);
	HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, 0); // Take SEN low to indicate we are sending data

	/* Clock in the data */
	for (uint32_t i = 0; i < SYNTH_SPI_BITS; i++) {

		/* Data written on the rising edge */
		uint32_t bit = (SYNTH_SPI_BITS - 1 - i);
		HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, !!(write_data & (1 << bit)));
		HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, 1);
		HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, 0);

		/* Data read on the falling edge */
		read_data = read_data
				| (HAL_GPIO_ReadPin(MISO_GPIO_Port, MISO_Pin)
						<< (SYNTH_SPI_BITS - 1 - i));
	}

	HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, 1); // Assert the SEN line to register the transmitted data

	if (verify) {
		const uint32_t verify_data = synth_readreg(reg_address); // Data returned on the second cycle
		if (verify_data != data) {
			printf("SPI transmission error!\n");
			Error_Handler(); // We enter an infinite loop here
		}
	}

	return read_data;
}

/**
  * @brief  Reads a register.
  * @param  Address
  * @retval Register contents
  */
static uint32_t synth_readreg(const uint32_t reg_address){

    synth_writereg(reg_address, 0x0, 0x0, DONT_VERIFY); // First cycle to send the read address
    const uint32_t read_data = synth_writereg(reg_address, 0x0, 0x0, DONT_VERIFY);  // Data returned on the second cycle

    return (read_data >> 8); // We only care about the first 24 bits returned.

}

/**
  * @brief  Program LO2 output gain.
  * @param  MW power setting
  * @retval Success/fail
  */
uint32_t set_MW_power (const uint8_t mw_power) {
	if (mw_power > 3) {//check that LO2GAIN is an integer from 0 to 3 inclusive
		printf("illegal mw_power - must be an integer from 0 to 3!\n");
		Error_Handler(); // We enter an infinite loop here
	}
	uint32_t read_data = synth_readreg(GAIN_DIVIDER_REGISTER); // Get the current value.
	read_data &= 0xFFFFFCFF; 		// Zero bits 8:9.
	read_data |= (mw_power << 8);	// Set LO2GAIN value.
	synth_writereg(read_data, GAIN_DIVIDER_REGISTER, 0x0, VERIFY); // Update the VCO divide register.
	#ifdef MW_VERBOSE
		printf("PROGRAMMED GAIN DIVIDER REGISTER: 0x%lX \r\n", read_data);
	#endif
	printf("LO2 gain setting: %u \r\n", mw_power);
	return SUCCESS;
}

/**
  * @brief  Initialises HMC835 synthesiser.
  * @param  MW power setting
  * @retval Success/fail
  */
uint32_t init_synthesiser(const uint8_t mw_power) {

	//Set pins to required initial conditions
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // Turn off the amber lock LED
	HAL_GPIO_WritePin(SCOPE_TRIG_OUT_GPIO_Port, SCOPE_TRIG_OUT_Pin, GPIO_PIN_SET); // Sets trigger output high
	HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, 0);
	HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, 1);
	HAL_GPIO_WritePin(REG_EN_GPIO_Port, REG_EN_Pin, 1); // Enable the main regulator.

	HAL_Delay(100); // Wait 100 ms for the supply to stabilise.

	synth_writereg(0x1UL << 5, OPEN_MODE_READ_ADDRESS, 0x0, DONT_VERIFY); // Soft reset.
	synth_writereg(0x41BFFF, ANALOG_EN_REGISTER, 0x0, VERIFY); // Set the SDO output level to 3.3 Volts

	uint32_t read_data = synth_readreg(ID_REGISTER); // Read the ID register to check the chip is communicating
	/* Check we have the correct ID */
	if (read_data != SYNTH_ID) {
		HAL_GPIO_WritePin(REG_EN_GPIO_Port, REG_EN_Pin, 0); // Disable the main regulator.
		printf("Incorrect synthesiser ID!\r\n");
		return ERROR;
	}

	/* Everything looks good, we can communicate with the chip :-) */
	printf("HMC835 Detected.\r\n");

	/* Enables Single-Ended output mode for LO2 output */
	read_data = synth_readreg(MODES_REGISTER); // Get the current value of the modes register
#ifdef MW_VERBOSE
	printf("READ MODES REGISTER: 0x%lX \r\n", read_data);
#endif
	read_data |= (0x1UL << 9);     // Enable single ended output for LO2 (LO2_P)
	read_data  &= ~(!AUTO_MUTE << 7); //can disable auto_mute - see variable declarations
	synth_writereg(read_data, MODES_REGISTER, 0x0, VERIFY); // Send
#ifdef MW_VERBOSE
	printf("PROGRAMMED MODES REGISTER: 0x%lX \r\n", read_data);
#endif

	/* Update lock detect window */
	//read_data = synth_readreg(LOCK_DETECT_REGISTER); // Get the current value.
	//read_data &= 0xFFFFFFF8; // Zero the first 3 LSBs.
	//read_data |= 0x07;
	//synth_writereg(read_data, LOCK_DETECT_REGISTER, 0x0, VERIFY); // Update the VCO divide register.

	synth_writereg(1, REFDIV_REGISTER, 0x0, VERIFY); // Reference divider setting.
#ifdef MW_VERBOSE
	printf("PROGRAMMED DIVIDER REGISTER: 0x01 \r\n");
#endif

	/* Lock detect training: This must be done after any change to the PD
	 * reference frequency or after power cycle. */
	read_data = synth_readreg(LOCK_DETECT_REGISTER); // Get contents of lock detect register
#ifdef MW_VERBOSE
	printf("READ LOCK_DETECT_REGISTER: 0x%lX \r\n", read_data);
#endif
	read_data |= (0x1UL << 11);      // Enable lock-detect counters.
	read_data |= (0x1UL << 14);      // Enable the lock-detect timer.
	read_data |= (0x1UL << 20);      // Train the lock-detect timer.
	synth_writereg(read_data, LOCK_DETECT_REGISTER, 0x0, VERIFY); // Send
#ifdef MW_VERBOSE
	printf("PROGRAMMED LOCK DETECT REGISTER: 0x%lX \r\n", read_data);
#endif
	HAL_Delay(10); // Wait 10 ms for training to complete, not sure if we really need to do this.

	/* Program LO2 output gain */
	if (mw_power > 3) {//check that LO2GAIN is an integer from 0 to 3 inclusive
		printf("illegal mw_power - must be an integer from 0 to 3!\n");
		Error_Handler(); // We enter an infinite loop here
	}
	read_data = synth_readreg(GAIN_DIVIDER_REGISTER); // Get the current value.
	read_data &= 0xFFFFFCFF; 		// Zero bits 8:9.
	read_data |= (mw_power << 8);	// Set LO2GAIN value.
	synth_writereg(read_data, GAIN_DIVIDER_REGISTER, 0x0, VERIFY); // Update the VCO divide register.
#ifdef MW_VERBOSE
	printf("PROGRAMMED GAIN DIVIDER REGISTER: 0x%lX \r\n", read_data);
	printf("LO2 gain setting: %u \r\n", mw_power);
#endif

	/* Sets output frequency to the hyperfine value */
	set_frequency_hz(HYPERFINE);
	//printf("Single frequency output: %f Hz \r\n", HYPERFINE);
	printf("Single frequency output: %.10g Hz \r\n", HYPERFINE);
//	struct MW_struct *mw_sweep_settings = 0;  //create a structure to store the sweep settings
	mw_sweep_settings.state = MW_FIXED_FREQ;
	return SUCCESS;
}

//static const bool poll_until_locked(uint32_t timeout) {
//
//	bool locked = false;
//
//	/* Check for lock */
//	uint32_t start = start_timer(MW_TIMER);
//
//	while ((MW_TIMER->CNT - start) < timeout) {
//		//printf("Debug lock while condition\r\n");
//		locked = synth_readreg(GPOLD_REGISTER) & (1UL << 1);
//		if (locked) {
//			stop_timer(MW_TIMER);
//			return true;
//		}
//	}
//
//	stop_timer(MW_TIMER);
//	return false;
//}

/**
  * @brief  Checks for MW frequency lock
  * @param  None
  * @retval Lock status
  */
static const bool lock_status(void) {

	bool locked = synth_readreg(GPOLD_REGISTER) & (1UL << 1);
	return locked;

}

/**
  * @brief  Mute MW output
  * @param  None
  * @retval None
  */
//static void mute_mw_outputs() {
//	uint32_t read_data = 0x0;
//
//	/* Mute the outputs by setting k value to zero */
//	read_data = synth_readreg(GAIN_DIVIDER_REGISTER); // Get the current value.
//	read_data &= 0xFFFFFFC0; // Zero the first 6 LSBs (VCO division value - mute).
//	synth_writereg(read_data, GAIN_DIVIDER_REGISTER, 0x0, VERIFY); // Update the VCO divide register.
//}

/**
  * @brief  Program HMC835 microwave frequency registers
  * @param  Integer frequency
  * @param  Fractional frequency
  * @param  VCO divider value
  * @param  Manually mute whilst changing frequency
  * @retval None
  */
//static void set_frequency(const uint32_t integer, const uint32_t fraction, const uint32_t vco_divider, bool mute) {
//
//	static uint32_t last_integer = -1, last_fraction = -1, last_vcodiv = -1;
//
//	uint32_t read_data = 0x0;
//
//	if (mute) {
//		mute_mw_outputs();
//	}
//
//	if (last_integer == -1 || (last_integer != integer)) {
//		synth_writereg(integer, INTEGER_FREQUENCY_REGISTER, 0x0, VERIFY);   // Integer register.
//		last_integer = integer;
//	}
//
//	if (last_fraction == -1 || (last_fraction != fraction)) {
//		synth_writereg(fraction, FRACTIONAL_FREQUENCY_REGISTER, 0x0, VERIFY);  // Fractional register.
//		last_fraction = fraction;
//	}
//
//	if (last_vcodiv == -1 || (last_vcodiv != vco_divider) || mute) {
//		read_data = synth_readreg(GAIN_DIVIDER_REGISTER); // Get the current value.
//		read_data &= 0xFFFFFFC0; // Zero the first 6 LSBs (VCO division value - mute).
//		read_data |= vco_divider; // This will set k which will un-mute the outputs */
//		synth_writereg(read_data, GAIN_DIVIDER_REGISTER, 0x0, VERIFY); // Update the VCO divide register.
//		last_vcodiv = vco_divider;
//	}
//
//}

/**
  * @brief  Program HMC835 microwave frequency registers
  * @param  Integer frequency
  * @param  Fractional frequency
  * @param  VCO divider value
  * @retval None
  */
static void set_freq_regs(const uint32_t integer, const uint32_t fraction, const uint32_t vco_divider) {

	static uint32_t last_integer = -1, last_fraction = -1, last_vcodiv = -1;

	uint32_t read_data = 0x0;

	if (last_vcodiv == -1 || (last_vcodiv != vco_divider)) {
		read_data = synth_readreg(GAIN_DIVIDER_REGISTER); // Get the current value.
		read_data &= 0xFFFFFFC0; // Zero the first 6 LSBs (VCO division value - mute).
		read_data |= vco_divider; // This will set k which will un-mute the outputs */
		synth_writereg(read_data, GAIN_DIVIDER_REGISTER, 0x0, VERIFY); // Update the VCO divide register.
		last_vcodiv = vco_divider;
	}

	if (last_integer == -1 || (last_integer != integer)) {
		synth_writereg(integer, INTEGER_FREQUENCY_REGISTER, 0x0, VERIFY);   // Integer register.
		last_integer = integer;
	}

	//writing fractional register last as this triggers auto-calibration
	if (last_fraction == -1 || (last_fraction != fraction)) {
		synth_writereg(fraction, FRACTIONAL_FREQUENCY_REGISTER, 0x0, VERIFY);  // Fractional register.
		last_fraction = fraction;
	}

}


/**
  * @brief  Translate a frequency into register values for programming to HMC835
  * @param  Frequency
  * @retval None
  */
void set_frequency_hz(const double fo) {

#ifdef OPTIMISED_FOR_3_035GHZ_GENERATION
	/* Code optimisation for Generation of frequencies close to 3.035GHz
	 * k always equals 1
	 */
	uint32_t k = 1;
#endif //OPTIMISED_FOR_3_035GHZ_GENERATION
#ifndef OPTIMISED_FOR_3_035GHZ_GENERATION
	/* Generic code that will work for any frequency supported by HMC835 */
	/* For the k divider we need to find the smallest even integer or use a max of 62*/
	uint32_t k = VCO_MAX_FREQ / fo;

	if (k != 1) {
		while (k > 62 || k % 2) {
			k = k - 1;
		}
	}
#endif //OPTIMISED_FOR_3_035GHZ_GENERATION

	/* Calculate the N division ratio */
	const double N = ((fo * k) / REF_FREQ);

	/* Extract the fractional and integer parts */
	const uint32_t NINT = N;
	const uint32_t NFRAC = ((N - NINT) * (1 << 24)) + 0.5;

#ifdef MW_F_CHECK
	//checks that the frequency requested can be exactly represented as a binary value
	//enters error loop with message if unsuccessful
	const double fo_check = (REF_FREQ * (NINT + (NFRAC / (double) (1 << 24)))) / k;
	const double fo_error = fo - fo_check;
	//add "-u _printf_float" to GCC linker flags to enable printf float support
	printf("Frequency requested: %.17g Hz\r\n", fo);
	printf("Setting frequency: k=%ld; N=%.17g; NINT=%ld; NFRAC=%ld\r\n", k,N, NINT, NFRAC);
	printf("Frequency error: %.4g Hz\r\n", fo_error);
	if (fo != fo_check) {
		printf("Failed to establish synthesiser frequency accurately\r\n");
		Error_Handler();
	}
#endif

	HAL_GPIO_WritePin(MW_INVALID_GPIO_Port, MW_INVALID_Pin, GPIO_PIN_SET); //Sets MW_invalid pin high
	//set_frequency(NINT, NFRAC, k, MANUAL_MUTE); //Sets only the necessary Hittite registers
	set_freq_regs(NINT, NFRAC, k); //Sets only the necessary Hittite registers

	//MW stabilisation delay and check for lock
	timer_delay(MW_TIMER, MW_STABILISE_TIME_US);
	//if (!poll_until_locked(LOCK_WAIT_US)) {
	if (!lock_status()) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); //turn off amber LED
		printf("Failed to establish MW Lock within %ld us of setting frequency!\r\n", MW_STABILISE_TIME_US);
#ifdef HALT_ON_LOSS_OF_LOCK
		Error_Handler();
#endif //HALT_ON_LOSS_OF_LOCK
		HAL_GPIO_WritePin(MW_INVALID_GPIO_Port, MW_INVALID_Pin, GPIO_PIN_RESET); //Sets MW_invalid pin low
	}

}

//Simon's MW sweep function
//void run_sweep() {
//
//	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // Assume we are locked, the LED will be disabled if lock fails.
//
//#ifdef RAMP_DAC
//	/* Zero the DAC output */
//	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
//#endif
//
//	//Converts the requested start and stop frequencies into
//	//Requires that the step size is a multiple of 2.98Hz
//	static const double start_freq = ((long)(sweep_settings.req_start_freq/sweep_settings.step_size)) * sweep_settings.step_size;
//	static const double stop_freq = ((long)((sweep_settings.req_stop_freq/sweep_settings.step_size) + 0.5)) * sweep_settings.step_size;
//	static const uint32_t num_points = ((stop_freq - start_freq)/sweep_settings.step_size) + 1;
//
//#ifdef RAMP_DAC
//	double dac_val = 0;
//#endif
//
//	//__disable_irq(); //Simon's code had IRQs disabled
//
//	/* Output used for triggering external scope */
//	HAL_GPIO_WritePin(SCOPE_TRIG_OUT_GPIO_Port, SCOPE_TRIG_OUT_Pin, GPIO_PIN_RESET); // Sets trigger output low
//#ifdef MW_VERBOSE
//	printf("Setting trigger output low \r\n");
//#endif
//
//	for (uint32_t i = 0; i < num_points; i++) {
//
//		double fo = start_freq + (i * sweep_settings.step_size);
//		set_frequency_hz(fo);
//		//printf("Point: %lu, ", i);
//
//
//#ifdef RAMP_DAC
//		dac_val = dac_val + (4096.0/num_points);
//		if(HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)dac_val) != HAL_OK){
//			printf("Failure to program value to DAC \r\n");
//			Error_Handler();
//		}
//#endif
//
//		timer_delay(MW_TIMER, DWELL_TIME_US);
//
//		blue_button_status = HAL_GPIO_ReadPin(BLUE_BUTTON_GPIO_Port, BLUE_BUTTON_Pin);
//		if (blue_button_status) {// If blue button is pressed
//			printf("Terminating sweep early as blue button pressed \r\n");
//			HAL_GPIO_WritePin(SCOPE_TRIG_OUT_GPIO_Port, SCOPE_TRIG_OUT_Pin, GPIO_PIN_SET); // Sets trigger output high
//			break;
//		}
//	}
//
//	//__enable_irq(); //Simon's code had IRQs disabled
//
//	HAL_GPIO_WritePin(SCOPE_TRIG_OUT_GPIO_Port, SCOPE_TRIG_OUT_Pin, GPIO_PIN_SET); // Sets trigger output high
//	printf("Sweep complete: %lu points\r\n", num_points);
//
//#ifdef RAMP_DAC
//	/* Zero and stop the DAC */
//	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
//	//HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
//#endif
//
//}

/**
  * @brief  Calculate MW sweep parameters based on steps and dwell time
  * @param  Centre frequency in Hz
  * @param  Span in Hz
  * @param  Overall sweep period in s
  * @retval Success/failure or early termination
  */
//static bool calc_period_based_MW_sweep(const double centre_freq, const double span, const double period) {
//
//}

void test_call(void) {
//	calc_defined_step_MW_sweep(HYPERFINE, 10000, 2, 1000); //10kHz sweep, 5 POP cycles per step, 1000 points
	calc_fixed_time_MW_sweep(HYPERFINE, 10000, 50, true); //10kHz sweep, 50s
}

/**
  * @brief  Print out the contents of the mw_sweep_settings structure
  * @param  None
  * @retval None
  */
  static void print_mw_sweep_settings (void) {
  	// Check that I've populated everything
  	printf("state: %u \r\n", mw_sweep_settings.state);
  	printf("k: %u \r\n", mw_sweep_settings.k);
  	printf("NINT: %lu \r\n", mw_sweep_settings.NINT);
  	printf("NFRAC_start: %lu \r\n", mw_sweep_settings.NFRAC_start);
  	printf("num_steps: %lu \r\n", mw_sweep_settings.num_steps);
  	printf("step_size: %lu \r\n", mw_sweep_settings.step_size);
  	printf("stabilise_time: %lu us\r\n", mw_sweep_settings.stabilise_time);
  	printf("dwell_time: %lu us\r\n", mw_sweep_settings.dwell_time);
  	printf("current_point: %lu us\r\n", mw_sweep_settings.current_point);
}

/**
  * @brief  Calculate MW sweep parameters based on number of points and POP cycles per step
  * @param  Centre frequency in Hz
  * @param  Span in Hz
  * @param  POP cycles per step
  * @param  Number of points
  * @retval Success/failure or early termination
  */
bool calc_defined_step_MW_sweep(const double centre_freq, const double span, const uint32_t pop_cycles_per_step, const uint32_t num_points_req) {
	printf("MW sweep will have %.9g GHz centre frequency with %.5g Hz span\r\n", centre_freq/1000000000, span);
	printf("and %ld POP cycles per step\r\n", pop_cycles_per_step);

	/* Calculate start frequency */
	double start_freq = centre_freq - 0.5* span;
	mw_sweep_settings.k = calculate_k(start_freq);
//
//	/* Calculate k */
//	uint8_t local_k = VCO_MAX_FREQ / start_freq;
//
//	if (local_k != 1) {
//		while (local_k > 62 || local_k % 2) {
//			local_k --;
//		}
//	}
//	mw_sweep_settings.k = local_k;

	/* Extrapolate step size requested versus achievable  */
	const double step_size_Hz = span / (num_points_req - 1);
	printf("Requested %ld steps, therefore step size of %.3g Hz\r\n", num_points_req, step_size_Hz);
	const double unit_step_size_Hz = REF_FREQ / (double) (mw_sweep_settings.k * (1 << 24)); //minimum step size possible
	//printf("Unit step size: %.3g Hz\r\n", unit_step_size_Hz);
	mw_sweep_settings.step_size = (step_size_Hz / unit_step_size_Hz + 0.5);
	if (!mw_sweep_settings.step_size) { //step_size must be a positive non-zero integer
		mw_sweep_settings.step_size++;
	}
	const double achieved_step_size = (double) (mw_sweep_settings.step_size * unit_step_size_Hz);
	printf("Step size achieved: %.3g Hz\r\n", achieved_step_size);
	mw_sweep_settings.num_steps = span / achieved_step_size;

	/* Can avoid spurs if frequency requested can be encoded exactly  */
	start_freq = ((long)(start_freq/unit_step_size_Hz)) * unit_step_size_Hz;

	/* Calculate the N division ratio, extracting the fractional and integer parts */
	const double N = ((start_freq * mw_sweep_settings.k) / REF_FREQ);
	mw_sweep_settings.NINT = N;
	mw_sweep_settings.NFRAC_start = ((N - mw_sweep_settings.NINT) * (1 << 24)) + 0.5;

	/* Calculate dwell time at each MW frequency */
	mw_sweep_settings.stabilise_time = MW_STABILISE_TIME_US; //Global MW stabilisation time
	mw_sweep_settings.dwell_time = pop_cycles_per_step * POP_CYCLE_TIME_US;

	/* Calculate the period of a sweep */
	const double calc_sweep_time = (double)(MW_STABILISE_TIME_US + MW_PROCESSING_TIME + mw_sweep_settings.dwell_time) * (double)(mw_sweep_settings.num_steps)/1000000;
	printf("Sweep period: %.3g s\r\n", calc_sweep_time);
	printf("%ld steps, %.3g ms each\r\n", mw_sweep_settings.num_steps, 1000* calc_sweep_time / mw_sweep_settings.num_steps);

	mw_sweep_settings.current_point = 0;

	//print_mw_sweep_settings();
	return(true);
}

/**
  * @brief  Calculate MW sweep parameters based on number of points and POP cycles per step
  * @param  Centre frequency in Hz
  * @param  Span in Hz
  * @param  Sweep period in s
  * @param  Additional time for scope sync
  * @retval Success/failure or early termination
  */
bool calc_fixed_time_MW_sweep(const double centre_freq, const double span, const double requested_sweep_period, const bool scope_sync_time) {
	//Dwell time must be a minimum of one POP cycle
	//Dwell time should be at least 50% of sweep time
	//Number of points shall be maximised within the available time

	//uint32_t pop_cycles_per_step, const uint32_t num_points_req) {
	printf("MW sweep will have %.9g GHz centre frequency with %.5g Hz span, over %.3g s\r\n", centre_freq/1000000000, span, requested_sweep_period);

	mw_sweep_settings.dwell_time = POP_CYCLE_TIME_US; //minimum possible value of dwell_time in us
	uint32_t step_time = MW_STABILISE_TIME_US + MW_PROCESSING_TIME + mw_sweep_settings.dwell_time; //minimum possible value in us
	uint32_t steps_in_sweep = requested_sweep_period * (double)(1000000 / step_time); //maximum possible number of steps in sweep, rounded down to an integer
	//printf("%lu steps in sweep, maximum\r\n", steps_in_sweep);

	/* now figure out the unit_step_size and how many steps will be taken in the span */

	/* Calculate start frequency */
	double start_freq = centre_freq - 0.5* span;
	mw_sweep_settings.k = calculate_k(start_freq);

//	/* Calculate k */
//	uint8_t local_k = VCO_MAX_FREQ / start_freq;
//
//	if (local_k != 1) {
//		while (local_k > 62 || local_k % 2) {
//			local_k --;
//		}
//	}
//	mw_sweep_settings.k = local_k;

	//steps should be evenly sized
	//selected step size should be an integer multiple of the unit step size
	//increase the step_size value until the sweep fits into the available period
	const double unit_step_size_Hz = REF_FREQ / (double) (mw_sweep_settings.k * (1 << 24)); //minimum step size possible
	//printf("Unit step size: %.3g Hz\r\n", unit_step_size_Hz);
	mw_sweep_settings.step_size = 1;
	while ((mw_sweep_settings.step_size * steps_in_sweep) < (span / unit_step_size_Hz)) {
		mw_sweep_settings.step_size++;
	}
	const double achieved_step_size = (double) (mw_sweep_settings.step_size * unit_step_size_Hz);
	printf("Step size: %lu x unit step i.e. %.3g Hz\r\n", mw_sweep_settings.step_size, achieved_step_size);

	//calculate number of steps in sweep and round off to integer
	mw_sweep_settings.num_steps = (span / achieved_step_size) + 0.5;
	printf("%lu steps in sweep, %.3g ms each\r\n", mw_sweep_settings.num_steps, (1000 * requested_sweep_period / mw_sweep_settings.num_steps));

	/* Can avoid spurs if frequency requested can be encoded exactly  */
	start_freq = ((long)(start_freq/unit_step_size_Hz)) * unit_step_size_Hz;

	/* Calculate the N division ratio, extracting the fractional and integer parts */
	const double N = ((start_freq * mw_sweep_settings.k) / REF_FREQ);
	mw_sweep_settings.NINT = N;
	mw_sweep_settings.NFRAC_start = ((N - mw_sweep_settings.NINT) * (1 << 24)) + 0.5;

	/* Calculate the period of a sweep */
	double step_period = (double)(MW_STABILISE_TIME_US + MW_PROCESSING_TIME + mw_sweep_settings.dwell_time)/1000000;
	double calc_sweep_time = step_period * (double)(mw_sweep_settings.num_steps);
//	double calc_sweep_time = (double)(MW_STABILISE_TIME_US + MW_PROCESSING_TIME + mw_sweep_settings.dwell_time) * (double)(mw_sweep_settings.num_steps)/1000000;
	printf("Sweep period could be reduced to %.3g s with same number of POP samples\r\n", calc_sweep_time);
	/* Increase dwell time until the sweep is just slightly longer than the sweep period requested */
//	while ((double)(mw_sweep_settings.num_steps * (MW_STABILISE_TIME_US + MW_PROCESSING_TIME + mw_sweep_settings.dwell_time)/1000000) < requested_sweep_period) {
	while ((double)(mw_sweep_settings.num_steps * step_period) < requested_sweep_period) {
		mw_sweep_settings.dwell_time++;
		step_period = (MW_STABILISE_TIME_US + MW_PROCESSING_TIME + mw_sweep_settings.dwell_time)/1000000; //recalculate for new dwell time
	}

	//Period of MW sweep isn't precise as it's based on measured averages processing time
	//Additional 10% (max of 1s) added to sweep period to guarantee horizontal scope sync
	if (scope_sync_time) {
		mw_sweep_settings.num_steps = mw_sweep_settings.num_steps * 1.1 + 0.5;
		calc_sweep_time = step_period * (double)(mw_sweep_settings.num_steps);

		/* Decrease number of steps if additional 10% is >1s */
		while (calc_sweep_time - requested_sweep_period > 1) {
			mw_sweep_settings.num_steps--;
			calc_sweep_time = step_period * (double)(mw_sweep_settings.num_steps);
		}

		//Double check of the sweep period selected
		printf("Final calculated sweep period, including scope sync: %.3g s\r\n", calc_sweep_time);
	}

	mw_sweep_settings.current_point = 0;
	//print_mw_sweep_settings();
	return(true);
}

/**
  * @brief  Calculates k value
  * @retval k
  */
static const uint32_t calculate_k(const double frequency) {
	uint32_t k = VCO_MAX_FREQ / frequency;

	if (k != 1) {
		while (k > 62 || k % 2) {
			k --;
		}
	}
	return (k);
}

/**
  * @brief  Starts a MW sweep
  * @retval Success/failure
  */
const bool start_MW_sweep(void) {
	//uses settings from the mw_sweep_settings structure
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // Assume MW lock, the LED will be disabled if lock fails.

	#ifdef RAMP_DAC
		/* Zero the DAC output */
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
		double dac_val = 0;
	#endif //RAMP_DAC

	#ifdef MW_VERBOSE
		printf("Setting trigger output low \r\n");
	#endif //MW_VERBOSE

	HAL_GPIO_WritePin(MW_INVALID_GPIO_Port, MW_INVALID_Pin, GPIO_PIN_SET); //Sets MW_invalid pin high
//	set_frequency(mw_sweep_settings.NINT, mw_sweep_settings.NFRAC_start, mw_sweep_settings.k, MANUAL_MUTE); //program initial MW frequency
	set_freq_regs(mw_sweep_settings.NINT, mw_sweep_settings.NFRAC_start, mw_sweep_settings.k); //program initial MW frequency
	mw_sweep_settings.state = MW_STABILISING; //waiting for MW output to stabilise
	mw_sweep_settings.current_point = 0; //currently on at start of ramp i.e. point 0
	/* Output used for triggering external scope */
	HAL_GPIO_WritePin(SCOPE_TRIG_OUT_GPIO_Port, SCOPE_TRIG_OUT_Pin, GPIO_PIN_RESET); // Sets trigger output low
	start_timer(MW_TIMER); //reset MW_timer and start counting
	return(true);
}

/**
  * @brief  Checks MW status to see if a timer has elapsed and if frequency needs changing.
  * @retval True if an action was taken
  */
const bool MW_update(void) {
	uint8_t local_copy_of_MW_state = mw_sweep_settings.state; //hack to make switch statement behave
	//switch (mw_sweep_settings.state)
	bool action_taken = false;
	switch (local_copy_of_MW_state)
	{
		case MW_STOPPED:
		case MW_FIXED_FREQ:
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
			if (mw_sweep_settings.current_point == mw_sweep_settings.num_steps) {
				/* All steps completed, tidy up and restart next sweep */
				HAL_GPIO_WritePin(SCOPE_TRIG_OUT_GPIO_Port, SCOPE_TRIG_OUT_Pin, GPIO_PIN_SET); // Sets trigger output high
				printf("Sweep complete\r\n");
				start_MW_sweep(); //restart the MW sweep
			} else {
				/* next MW step */
				mw_sweep_settings.current_point++; //increment point counter
				//calculate the new MW frequency and program Hittite with new NFRAC
				uint32_t local_NFRAC = mw_sweep_settings.NFRAC_start + mw_sweep_settings.step_size * mw_sweep_settings.current_point;
//				set_frequency(mw_sweep_settings.NINT, local_NFRAC, mw_sweep_settings.k, MANUAL_MUTE); //program new MW frequency
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

		default: // Other state
	       printf("MW_update has detected illegal state: %u \r\n", mw_sweep_settings.state);
	       printf("local version: %u \r\n", local_copy_of_MW_state);
	}
    return(action_taken);
}

//
////		struct MW_struct
////		{
////		  uint8_t state;             /* current MW state */
////		  uint8_t k;
////		  uint32_t NINT;
////		  uint32_t NFRAC_start; //ramp starting value of fractional register
////		  uint32_t num_steps;
////		  uint32_t step_size; //in multiples of minimum step
////		  uint32_t stabilise_time;
////		  uint32_t dwell_time;
////		  uint32_t current_point;
////		};
//
//
//
//	for (uint32_t i = 0; i < points; i++) {
//		fo = start_freq + (i * step_size);
//		set_frequency_hz(fo);
//		//printf("Point: %lu, ", i);
//

//
//		timer_delay(MW_TIMER, dwell_time_us);
//
//		blue_button_status = HAL_GPIO_ReadPin(BLUE_BUTTON_GPIO_Port, BLUE_BUTTON_Pin);
//		if (blue_button_status) {// If blue button is pressed
//			printf("Terminating sweep early as blue button pressed \r\n");
//			HAL_GPIO_WritePin(SCOPE_TRIG_OUT_GPIO_Port, SCOPE_TRIG_OUT_Pin, GPIO_PIN_SET); // Sets trigger output high
//			sweep_status = false;
//			break;
//		}
//	}
//
//	HAL_GPIO_WritePin(SCOPE_TRIG_OUT_GPIO_Port, SCOPE_TRIG_OUT_Pin, GPIO_PIN_SET); // Sets trigger output high
//	printf("Sweep complete\r\n");
//
//#ifdef RAMP_DAC
//	/* Zero and stop the DAC */
//	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
//	//HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
//#endif
//
//	return sweep_status;
//}


/* Function to check MW settling time
 * Toggles between two MW frequencies
 */
void MW_frequency_toggle (const double f_one, const double f_two) {
	printf("MW frequency toggling experiment\r\n");
	printf("Toggling between %.9g and %.9g GHz\r\n", f_one/1000000000, f_two/1000000000);

	/* For the k divider we need to find the smallest even integer or use a max of 62*/
	uint32_t k_one = VCO_MAX_FREQ / f_one;
	if (k_one != 1) {
		while (k_one > 62 || k_one % 2) {
			k_one--;
		}
	}
	uint32_t k_two = VCO_MAX_FREQ / f_two;
	if (k_two != 1) {
		while (k_two > 62 || k_two % 2) {
			k_two--;
		}
	}

	const double N_one = ((f_one * k_one) / REF_FREQ);
	const double N_two = ((f_two * k_two) / REF_FREQ);

	/* Extract the fractional and integer parts */
	const uint32_t N_one_INT = N_one;
	const uint32_t N_one_FRAC = ((N_one - N_one_INT) * (1 << 24)) + 0.5;
	const uint32_t N_two_INT = N_two;
	const uint32_t N_two_FRAC = ((N_two - N_two_INT) * (1 << 24)) + 0.5;

	while (1) {
//	set_frequency(N_one_INT, N_one_FRAC, k_one, MANUAL_MUTE); //Program necessary values for f_one
	set_freq_regs(N_one_INT, N_one_FRAC, k_one); //Program necessary values for f_one
	HAL_GPIO_WritePin(SCOPE_TRIG_OUT_GPIO_Port, SCOPE_TRIG_OUT_Pin, GPIO_PIN_RESET); // Sets trigger output low
	timer_delay(SLOW_TIMER, 1000); //100ms delay
//	set_frequency(N_two_INT, N_two_FRAC, k_two, MANUAL_MUTE); //Program necessary values for f_two
	set_freq_regs(N_two_INT, N_two_FRAC, k_two); //Program necessary values for f_two
	HAL_GPIO_WritePin(SCOPE_TRIG_OUT_GPIO_Port, SCOPE_TRIG_OUT_Pin, GPIO_PIN_SET); // Sets trigger output high
	timer_delay(SLOW_TIMER, 1000); //100ms delay
	}
}

/* Selects SDO pin connectivity/functionality
 * By default, the SDO pin will output 'Lock detect' but can be connected
 * to other internal signals. See table 2.15 of HMC835 datasheet (v04.1113)
 * for more details all options
 */
 void set_SDO_output(const uint32_t GPO_setting) {
	//Default output on SDO pin is 'Lock detect output', value 0x01
	//VCO divider is 0x0A
	//See table 2.15 of HMC835 datasheet for more details (v04.1113)
	uint32_t read_data = 0x0;

	if (GPO_setting > 31) {
		printf("SDO pin value must be less that 32\r\n");
		Error_Handler();
	}
	read_data = synth_readreg(GPO_REGISTER); // Get the current value.
	read_data &= 0xFFFFFFE0; // Zero the first 5 LSBs.
	//read_data |= 0x0A; //Select VCO divider output
	read_data |= GPO_setting; //Select GPO output dependent on function input value
	synth_writereg(read_data, GPO_REGISTER, 0x0, VERIFY); // Update the GPO register.
}
