
/*
VESC firmware designed for G2 aircraft of Vayu.co
Firmware has been developed based on VESC project with hardware configuration hw_410.c, hw_410.h, drv8301.c, drv8301.h

Developer: sheng-yu.lo@vayu.us
Date: July, 15, 2019
*/

#include "hw.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils.h"
#include "drv8353.h"

// Variables
static volatile bool i2c_running = false;

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};

void hw_init_gpio(void) {
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);        // Don't needed

	// LEDs - Check
	palSetPadMode(GPIOC, 4,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);                              // LED-Green
	palSetPadMode(GPIOC, 5,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);                              // LED-RED

	// ENABLE_GATE - Check
	palSetPadMode(GPIOC, 10,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	DISABLE_GATE();

	// GPIOA Configuration: Channel 1 to 3 as alternate function push-pull - For DRV8353 L1, L2, L3, H1, H2, H3 - Check
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |      // H3
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |      // H2
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |     // H1
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |     // L3
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |     // L2
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |     // L1
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	// Hall sensors - Check
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);

	// Fault pin - Check
	palSetPadMode(GPIOC, 12, PAL_MODE_INPUT_PULLUP);

	// ADC Pins - Check half
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);       // SENS3
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);       // SENS2
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);       // SENS1
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);       // ADC_TEMP
	// palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG);       // Test
    palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);       // ADC_EXT
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);       // ADC_EXT2
	palSetPadMode(GPIOA, 7, PAL_MODE_INPUT_ANALOG);       // ADC_EXT3

	palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);       // BR_SO2
	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);       // BR_SO1
	//palSetPadMode(GPIOB, 2, PAL_MODE_INPUT_ANALOG);       // BR_SO3 - PB2 is not an ADC

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);       // TEMP_MOTOR
	// palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);       // Test
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);       // AN_IN
	//palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);       // Test

	// drv8353_init(); // Use hardware interface for setting current gain, July, 17
}

void hw_setup_adc_channels(void) {
	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_15Cycles);           // SENS1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_15Cycles);           // CURR1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 3, ADC_SampleTime_15Cycles);     // Vrefint
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_15Cycles);           // ADC_EXT
	
	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_15Cycles);           // SENS2
	ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 2, ADC_SampleTime_15Cycles);           // CURR2
	ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 3, ADC_SampleTime_15Cycles);          // TEMP_MOTOR
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 4, ADC_SampleTime_15Cycles);      	 // ADC_EXT2

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);           // SENS3
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);          // AN_IN
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 3, ADC_SampleTime_15Cycles);           // ADC_TEMP
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 4, ADC_SampleTime_15Cycles);     		 // ADC_EXT3 (temp, virtual)

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_15Cycles);          // CURR1
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_15Cycles);          // CURR2
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_15Cycles);          // CURR1
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_8, 2, ADC_SampleTime_15Cycles);          // CURR2

}

void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;

	}

	i2cReleaseBus(&HW_I2C_DEV);
}

/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void) {
	if (i2c_running) {
		i2cAcquireBus(&HW_I2C_DEV);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for(int i = 0;i < 16;i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);

		i2cReleaseBus(&HW_I2C_DEV);
	}
}
