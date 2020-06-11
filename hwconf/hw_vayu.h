
/*
VESC firmware designed for G2 aircraft of Vayu.co
Firmware has been developed based on VESC project with hardware configuration hw_410.c, hw_410.h, drv8301.c, drv8301.h

Developer: sheng-yu.lo@vayu.us
Date: July, 15, 2019
*/


#ifndef HW_VAYU_H_
#define HW_VAYU_H_

#define HW_NAME					"VAYU"

// HW properties
// #define HW_HAS_DRV8353       Use hardware interface for setting current gain, July, 17
// #define HW_HAS_3_SHUNTS      Only two shunts sensor
// #define HW_HAS_PHASE_SHUNTS

// Macros
#define ENABLE_GATE()			palSetPad(GPIOC, 10)
#define DISABLE_GATE()			palClearPad(GPIOC, 10)
#define IS_DRV_FAULT()			(!palReadPad(GPIOC, 12))

#define LED_GREEN_ON()			palSetPad(GPIOC, 4)
#define LED_GREEN_OFF()			palClearPad(GPIOC, 4)
#define LED_RED_ON()			palSetPad(GPIOC, 5)
#define LED_RED_OFF()			palClearPad(GPIOC, 5)


/*
 * ADC Vector
 *
 * 0:	ADC1      IN2     SENS1  (Phase-1 Voltage)
 * 1:	ADC2      IN1     SENS2  (Phase-2 Voltage)
 * 2:	ADC3      IN0     SENS3  (Phase-1 Voltage)
 * 3:	ADC1      IN9     CURR1  (Phase-1 Current - BR_SO1)
 * 4:	ADC2      IN8     CURR1  (Phase-2 Current - BR_SO2)
 * 5:   ADC3      IN12    AN_IN  (Vss)
 * 6:   ADC1      Vrefint
 * 7:   ADC2      IN10    TEMP_MOTOR
 * 8:   ADC3      IN3     TEMP_MOS  (= TEMP_PCB)
 * 9:   ADC1      IN5     ADC_EXT1
 * 10:  ADC2      IN6     ADC_EXT2
 * 11:  ADCc      IN7     ADC_EXT3  (temp, virtual)
 */



#define HW_ADC_CHANNELS			12 // Number of used channel, should be same number as vector
#define HW_ADC_INJ_CHANNELS		2  // Number of injected channel
#define HW_ADC_NBR_CONV			4  // Number of channel in for each ADC. For example, each of ADC has three channels

// ADC Indexes
#define ADC_IND_SENS1			0  // SENS1
#define ADC_IND_SENS2			1  // SENS2
#define ADC_IND_SENS3			2  // SENS3
#define ADC_IND_CURR1			3  // CURR1
#define ADC_IND_CURR2			4  // CURR2
#define ADC_IND_VIN_SENS        5  // AN_IN
#define ADC_IND_VREFINT         6  // Vrefint
#define ADC_IND_TEMP_MOTOR      7  // TEMP_MOTOR
#define ADC_IND_TEMP_MOS        8  // TEMP_PCB
#define ADC_IND_EXT			    9  // External ADC sensor, not using
#define ADC_IND_EXT2			10 // External ADC sensor, not using


// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG					3.3     // VCC (?)
#endif
#ifndef VIN_R1                          // AN_IN: R12 = 39kOhm
#define VIN_R1					39000.0
#endif
#ifndef VIN_R2                          // AN_IN: R13 = 2.2kOhm
#define VIN_R2					2200.0
#endif
// This can be configured via SPI or hardware interface. For now, I will use hardware interface first which connect gain to ground.
// Note that ther is a 47kOhm between ground.
#ifndef CURRENT_AMP_GAIN        
#define CURRENT_AMP_GAIN		10.0
#endif
// SHUNT resistor, CSS2H-3920R-1L00F, has 0.001 OHM
#ifndef CURRENT_SHUNT_RES       
#define CURRENT_SHUNT_RES		0.001
//#define CURRENT_SHUNT_RES		0.001 // hw410.h
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC Thermistors
#define NTC_B_CONSTANT          3380.0  // B Constant of thermistor, NCP18XH103F03RB
#define NTC_RES(adc_val)		((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / NTC_B_CONSTANT) + (1.0 / 298.15)) - 273.15)

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE		0
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE		0
#endif

// UART Peripheral (USART6) COM J1 - Check
#define HW_UART_DEV				SD6                   // serial_lld.c
#define HW_UART_GPIO_AF			GPIO_AF_USART6        // stm32f4_gpio_af.h
#define HW_UART_TX_PORT			GPIOC
#define HW_UART_TX_PIN			6
#define HW_UART_RX_PORT			GPIOC
#define HW_UART_RX_PIN			7

// ICU Peripheral for servo decoding (SERVO) - Check
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOB
#define HW_ICU_PIN				5

// I2C Peripheral (I2C2) COM J1 - Check
#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11

// Hall/encoder pins - Check
#define HW_HALL_ENC_GPIO1		GPIOB
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOB
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		11
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

// SPI pins (SPI1) COM J1 - Check
#define HW_SPI_DEV				SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS			GPIOA
#define HW_SPI_PIN_NSS			4
#define HW_SPI_PORT_SCK			GPIOA
#define HW_SPI_PIN_SCK			5
#define HW_SPI_PORT_MISO        GPIOA
#define HW_SPI_PIN_MISO         6
#define HW_SPI_PORT_MOSI		GPIOA
#define HW_SPI_PIN_MOSI			7

// SPI for DRV8353 - No internal SPI from MCU to DRV8353. DRV8353's SPI is pinout from J7 and MCU's SPI is pinout from J1. Connect them together.
// Use hardware interface for setting current gain, July, 17
// #define DRV8353_MOSI_GPIO       GPIOA
// #define DRV8353_MOSI_PIN        7
// #define DRV8353_MISO_GPIO       GPIOA
// #define DRV8353_MISO_PIN        6
// #define DRV8353_SCK_GPIO        GPIOA
// #define DRV8353_SCK_PIN         5
// #define DRV8353_CS_GPIO         GPIOA
// #define DRV8353_CS_PIN          4


// CAN is set in hw.h

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Default setting overrides
// #ifndef MCCONF_DEFAULT_MOTOR_TYPE
// #define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
// #endif
// #ifndef MCCONF_L_MAX_ABS_CURRENT
// #define MCCONF_L_MAX_ABS_CURRENT		150.0	// The maximum absolute current above which a fault is generated
// #endif
// #ifndef MCCONF_FOC_SAMPLE_V0_V7
// #define MCCONF_FOC_SAMPLE_V0_V7			false	// Run control loop in both v0 and v7 (requires phase shunts)
// #endif

// Setting limits
// #define HW_LIM_CURRENT			-120.0, 120.0
// #define HW_LIM_CURRENT_IN		-120.0, 120.0
// #define HW_LIM_CURRENT_ABS		0.0, 160.0
// #define HW_LIM_VIN				6.0, 57.0
// #define HW_LIM_ERPM				-200e3, 200e3
// #define HW_LIM_DUTY_MIN			0.0, 0.1
// #define HW_LIM_DUTY_MAX			0.0, 0.99
// #define HW_LIM_TEMP_FET			-40.0, 110.0


// Setting limits - from hw410.h
#define HW_LIM_CURRENT			-100.0, 100.0
#define HW_LIM_CURRENT_IN		-100.0, 100.0
#define HW_LIM_CURRENT_ABS		0.0, 150.0
#define HW_LIM_VIN				6.0, 57.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.95
#define HW_LIM_TEMP_FET			-40.0, 110.0


#endif /* HW_VAYU_H_ */
