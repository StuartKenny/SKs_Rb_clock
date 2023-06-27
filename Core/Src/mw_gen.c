/* Includes ------------------------------------------------------------------*/
#include "mw_gen.h"
#include "main.h" //needed for port definitions
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
#define MW_VERBOSE
//#define RAMP_DAC // Use a DAC output to represent increasing MW frequency
#define HALT_ON_LOSS_OF_LOCK
#define VERIFY 1 //constant, don't touch
#define DONT_VERIFY 0 //constant, don't touch

/* HMC835LP6GE Register addresses */
#define ID_REGISTER 0x00 //read-only
#define OPEN_MODE_READ_ADDRESS 0x00 //write-only
#define REFDIV_REGISTER 0x02
#define INTEGER_FREQUENCY_REGISTER 0x03
#define FRACTIONAL_FREQUENCY_REGISTER 0x04
#define LOCK_DETECT_REGISTER 0x07
#define ANALOG_EN_REGISTER 0x08
#define GPOLD_REGISTER 0x12
#define GAIN_DIVIDER_REGISTER 0x16
#define MODES_REGISTER 0x17

/* Private variables ---------------------------------------------------------*/
static TIM_TypeDef * FAST_TIMER = TIM3; // Clocked at 100 kHz
static const uint32_t SYNTH_SPI_BITS = 32;
static const uint32_t SYNTH_ID = 0xC7701A;
//static const uint32_t LOCK_WAIT_US = 10; // 10 us SB original definition
static const uint32_t LOCK_WAIT_US = 1000; // 1ms (10us was regularly timing out)
//static const uint32_t DWELL_TIME_US = 100; // 100us
//static const uint32_t DWELL_TIME_US = 5960; // 5.96ms x 1679 steps for 10s ramp (12.7s in practice)
static const uint32_t DWELL_TIME_US = 4360; // (4.36ms + measured 1.6ms processing) x 1679 steps for 10s ramp
//static const uint32_t DWELL_TIME_US = 59600; // 59.6ms x 1679 steps for 100s ramp (102s in practice)
//static const uint32_t DWELL_TIME_US = 1000000; // 1s for accurate spectrum analyser readings
//static const uint32_t ERROR_LED_DELAY = 1000; // 100 ms
static const double VCO_MAX_FREQ = 4100E6;
//static const double VCO_MIN_FREQ = 2050E6;
static const double REF_FREQ = 50E6;
static const uint8_t LO2GAIN = 0x3; // 3 is max, 0 is min, log scale with 2dB between points
//max is +5dBm output, min is -1dBm out.
//NOTE - these values are measured and not consistent with datasheet
static const double HYPERFINE = 3035736939; //Rb85 hyperfine frequency

/* Mute MW generation whilst changing frequency */
static const bool AUTO_MUTE = true; //0 is disabled, 1 is enabled ***DO NOT ENABLE UNTIL CODE DEBUGGED***
static const bool MANUAL_MUTE = true; //0 is disabled, 1 is enabled

//MW sweep settings have been selected so that all values can be represented exactly as binary fractions
//For 5kHz sweep, 2.98Hz x 1679 steps, centred around 3.035736939GHz
//SweepSettings const sweep_settings = {3.0357344390E9, 3.0357394390e9, (50e6 / (1 << 24)) }; //SB original settings
//For 10kHz sweep, 5.96Hz x 1679 steps, centred around 3.035736939GHz
SweepSettings const sweep_settings = {3.0357319390E9, 3.0357419390e9, (50e6 / (1 << 23)) };
//For 10kHz sweep, 190.7Hz step, centred around 3.035736939GHz
//SweepSettings const sweep_settings = {3.0357319390E9, 3.0357419390e9, (50e6 / (1 << 18)) };

/* Private function prototypes -----------------------------------------------*/
__attribute__((section(".itcm"))) static uint32_t synth_writereg(const uint32_t data, const uint32_t reg_address, const uint32_t chip_address, const bool verify);
__attribute__((section(".itcm"))) static uint32_t synth_readreg(const uint32_t reg_address);
__attribute__((section(".itcm"))) uint32_t init_synthesiser();
__attribute__((section(".itcm"))) static const bool check_lock(uint32_t timeout);
__attribute__((section(".itcm"))) static void mute_mw_outputs();
__attribute__((section(".itcm"))) static void set_frequency(const uint32_t integer, const uint32_t fraction, const uint32_t vco_divider, bool mute);
__attribute__((section(".itcm"))) void set_frequency_hz(const double fo);
__attribute__((section(".itcm"))) void run_sweep();
extern uint32_t start_timer(TIM_TypeDef * timer);
extern uint32_t stop_timer(TIM_TypeDef * timer);
extern void timer_delay(TIM_TypeDef *timer, uint32_t delay_us);
extern void Error_Handler(void);

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

static uint32_t synth_readreg(const uint32_t reg_address){

    synth_writereg(reg_address, 0x0, 0x0, DONT_VERIFY); // First cycle to send the read address
    const uint32_t read_data = synth_writereg(reg_address, 0x0, 0x0, DONT_VERIFY);  // Data returned on the second cycle

    return (read_data >> 8); // We only care about the first 24 bits returned.

}

uint32_t init_synthesiser() {

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // Turn off the amber lock LED

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
	read_data  &= ~(AUTO_MUTE << 7); //can disable auto_mute - see variable declarations
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
	read_data = synth_readreg(GAIN_DIVIDER_REGISTER); // Get the current value.
	read_data &= 0xFFFFFCFF; 		// Zero bits 8:9.
	read_data |= (LO2GAIN << 8);	// Set LO2GAIN value.
	synth_writereg(read_data, GAIN_DIVIDER_REGISTER, 0x0, VERIFY); // Update the VCO divide register.
#ifdef MW_VERBOSE
	printf("PROGRAMMED GAIN DIVIDER REGISTER: 0x%lX \r\n", read_data);
	printf("LO2 gain setting: %u \r\n", LO2GAIN);
#endif

	/* Sets output frequency to the hyperfine value */
	set_frequency_hz(HYPERFINE);
	printf("Single frequency output: %f Hz \r\n", HYPERFINE);
	return SUCCESS;
}

static const bool check_lock(uint32_t timeout) {

	bool locked = false;

	/* Check for lock */
	uint32_t start = start_timer(FAST_TIMER);

	while ((FAST_TIMER->CNT - start) < timeout) {
		//printf("Debug lock while condition\r\n");
		//locked = synth_readreg(0x12) & (1UL << 1);
		locked = synth_readreg(GPOLD_REGISTER) & (1UL << 1);
		if (locked) {
			stop_timer(FAST_TIMER);
			return true;
		}
	}

	stop_timer(FAST_TIMER);
	return false;
}

static void mute_mw_outputs() {
	uint32_t read_data = 0x0;

	/* Mute the outputs by setting k value to zero */
	read_data = synth_readreg(GAIN_DIVIDER_REGISTER); // Get the current value.
	read_data &= 0xFFFFFFC0; // Zero the first 6 LSBs (VCO division value - mute).
	synth_writereg(read_data, GAIN_DIVIDER_REGISTER, 0x0, VERIFY); // Update the VCO divide register.
}

static void set_frequency(const uint32_t integer, const uint32_t fraction, const uint32_t vco_divider, bool mute) {

	static uint32_t last_integer = -1, last_fraction = -1, last_vcodiv = -1;

	uint32_t read_data = 0x0;

	if (mute) {
		mute_mw_outputs();
	}

	if (last_integer == -1 || (last_integer != integer)) {
		synth_writereg(integer, INTEGER_FREQUENCY_REGISTER, 0x0, VERIFY);   // Integer register.
		last_integer = integer;
	}

	if (last_fraction == -1 || (last_fraction != fraction)) {
		synth_writereg(fraction, FRACTIONAL_FREQUENCY_REGISTER, 0x0, VERIFY);  // Fractional register.
		last_fraction = fraction;
	}

	if (last_vcodiv == -1 || (last_vcodiv != vco_divider) || mute) {
		read_data = synth_readreg(GAIN_DIVIDER_REGISTER); // Get the current value.
		read_data &= 0xFFFFFFC0; // Zero the first 6 LSBs (VCO division value - mute).
		read_data |= vco_divider; // This will set k which will un-mute the outputs */
		synth_writereg(read_data, GAIN_DIVIDER_REGISTER, 0x0, VERIFY); // Update the VCO divide register.
		last_vcodiv = vco_divider;
	}

	if (!check_lock(LOCK_WAIT_US)) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); //turn off amber LED
		printf("Lock failed within set_frequency!\r\n");
#ifdef HALT_ON_LOSS_OF_LOCK
		Error_Handler();
#endif //HALT_ON_LOSS_OF_LOCK
	}

}

void set_frequency_hz(const double fo) {

	/* For the k divider we need to find the smallest even integer or use a max of 62*/
	uint32_t k = VCO_MAX_FREQ / fo;

	if (k != 1) {
		while (k > 62 || k % 2) {
			k = k - 1;
		}
	}

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

	set_frequency(NINT, NFRAC, k, MANUAL_MUTE);

}

void run_sweep() {

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // Assume we are locked, the LED will be disabled if lock fails.

#ifdef RAMP_DAC
	/* Zero the DAC output */
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
#endif

	static const double start_freq = ((long)(sweep_settings.req_start_freq/sweep_settings.step_size)) * sweep_settings.step_size;
	static const double stop_freq = ((long)((sweep_settings.req_stop_freq/sweep_settings.step_size) + 0.5)) * sweep_settings.step_size;
	static const uint32_t num_points = ((stop_freq - start_freq)/sweep_settings.step_size) + 1;

#ifdef RAMP_DAC
	double dac_val = 0;
#endif

	__disable_irq();

	for (uint32_t i = 0; i < num_points; i++) {

		double fo = start_freq + (i * sweep_settings.step_size);
		set_frequency_hz(fo);


#ifdef RAMP_DAC
		dac_val = dac_val + (4096.0/num_points);
		if(HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)dac_val) != HAL_OK){
			printf("DAC setup failed!\r\n");
			Error_Handler();
		}
#endif

		timer_delay(FAST_TIMER, DWELL_TIME_US);

	}

	__enable_irq();

	printf("Total Points: %lu\r\n", num_points);

#ifdef RAMP_DAC
	/* Zero and stop the DAC */
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
#endif

}
