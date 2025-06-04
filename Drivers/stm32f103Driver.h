#ifndef STM32F103DRIVER_H
#define STM32F103DRIVER_H
#include <stdint.h>
#include "lcd_image.h"



/**********************************************************/
/*************************__GPIO__*************************/
/**********************************************************/

// Base addresses of GPIO ports
#define GPIOA_BASE (0x40010800U)
#define GPIOB_BASE (0x40010C00U)
#define GPIOC_BASE (0x40011000U)
#define GPIOD_BASE (0x40011400U)
#define GPIOE_BASE (0x40011800U)

// Define the structure for GPIO registers
typedef struct
{
    volatile uint32_t CRL;   // Configuration Register Low (GPIOx_CRL)
    volatile uint32_t CRH;   // Configuration Register High (GPIOx_CRH)
    volatile uint32_t IDR;   // Input Data Register (GPIOx_IDR)
    volatile uint32_t ODR;   // Output Data Register (GPIOx_ODR)
    volatile uint32_t BSRR;  // Bit Set/Reset Register (GPIOx_BSRR)
    volatile uint32_t BRR;   // Bit Reset Register (GPIOx_BRR)
    volatile uint32_t LCKR;  // Lock Register (GPIOx_LCKR)
} GPIO_TypeDef;

// Define GPIO port pointers
#define GPIOA ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE ((GPIO_TypeDef *) GPIOE_BASE)


// GPIO Mode and Configuration Constants
//defining the ports:
#define portA 							1
#define portB 							2
#define portC 							3
#define portD 							4
#define portE 							5

#define input               0
#define output_10Mhz        1
#define output_2Mhz         2
#define output_50Mhz        3

#define analog_in           0
#define floating_in         1
#define pp_in               2

#define gp_output           0
#define od_output           1
#define af_pp_output        2
#define af_od_output        3

#define HIGH                1
#define LOW                 0








/*******************************************************************/
/*************************__RCC_Registers__*************************/
/*******************************************************************/
// Define the structure for RCC registers
typedef struct
{
    volatile uint32_t CR;       // Clock Control Register (RCC_CR)
    volatile uint32_t CFGR;     // Clock Configuration Register (RCC_CFGR)
    volatile uint32_t CIR;      // Clock Interrupt Register (RCC_CIR)
    volatile uint32_t APB2RSTR; // APB2 Peripheral Reset Register (RCC_APB2RSTR)
    volatile uint32_t APB1RSTR; // APB1 Peripheral Reset Register (RCC_APB1RSTR)
    volatile uint32_t AHBENR;   // AHB Peripheral Clock Enable Register (RCC_AHBENR)
    volatile uint32_t APB2ENR;  // APB2 Peripheral Clock Enable Register (RCC_APB2ENR)
    volatile uint32_t APB1ENR;  // APB1 Peripheral Clock Enable Register (RCC_APB1ENR)
    volatile uint32_t BDCR;     // Backup Domain Control Register (RCC_BDCR)
    volatile uint32_t CSR;      // Control/Status Register (RCC_CSR)
} RCC_TypeDef;

// Base address of RCC
#define RCC_BASE (0x40021000U)

// Define RCC pointer
#define RCC ((RCC_TypeDef *) RCC_BASE)

#define HSI_CLOCK  0
#define HSE_CLOCK  1


/*******************************************************************/
/*************************__Flash_Memory_Registers__*************************/
/*******************************************************************/

typedef struct
{
    volatile uint32_t ACR;      // Flash Access Control Register (FLASH_ACR)        Offset: 0x00
    volatile uint32_t KEYR;     // Flash Key Register (FLASH_KEYR)                  Offset: 0x04
    volatile uint32_t OPTKEYR;  // Flash Option Key Register (FLASH_OPTKEYR)        Offset: 0x08
    volatile uint32_t SR;       // Flash Status Register (FLASH_SR)                 Offset: 0x0C
    volatile uint32_t CR;       // Flash Control Register (FLASH_CR)                Offset: 0x10
    volatile uint32_t AR;       // Flash Address Register (FLASH_AR)                Offset: 0x14
    uint32_t RESERVED;          // Reserved (not used)                              Offset: 0x18
    volatile uint32_t OBR;      // Option Byte Register (FLASH_OBR)                 Offset: 0x1C
    volatile uint32_t WRPR;     // Write Protection Register (FLASH_WRPR)           Offset: 0x20
} FLASH_TypeDef;

// Base address of FLASH
#define FLASH_BASE (0x40022000U)

// Define FLASH pointer
#define FLASH ((FLASH_TypeDef *) FLASH_BASE)






/********************************************************************************/
/*************************__Handling Global interrupt__*************************/
/********************************************************************************/

#define __enableinterrupt()  __asm("CPSIE I")  // ; Enable interrupts and configurable fault handlers (clear PRIMASK)
#define __disableinterrupt() __asm("CPSID I")  // Disable interrupts and configurable fault handlers (set PRIMASK)








/********************************************************/
/*************************__EXTI__*************************/
/********************************************************/
#define AFIO_BASE (0x40010000UL) // Base address of AFIO peripheral

//define the structure of AFIO registers
typedef struct
{
    volatile uint32_t EVCR;      // Event Control Register
    volatile uint32_t MAPR;      // Remap and Debug I/O Configuration Register
    volatile uint32_t EXTI_1;    // EXTI Line 0/1 configuration register (EXTICR1)
    volatile uint32_t EXTI_2;    // EXTI Line 2/3 configuration register (EXTICR2)
    volatile uint32_t EXTI_3;    // EXTI Line 4/5/6/7 configuration register (EXTICR3)
    volatile uint32_t EXTI_4;    // EXTI Line 8/9/10/11/12/13/14/15 configuration register (EXTICR4)
    volatile uint32_t MAPR2;     // Remap and Debug I/O Configuration Register 2 (on some high-density variants)
} AFIO_TypeDef;

#define AFIO ((AFIO_TypeDef *) AFIO_BASE)


// Define the structure for EXTI registers 
#define EXTI_BASE (0x40010400UL) // Base address of EXTI peripheral
typedef struct
{
    volatile uint32_t IMR;   // Interrupt Mask Register
    volatile uint32_t EMR;   // Event Mask Register
    volatile uint32_t RTSR;  // Rising Trigger Selection Register
    volatile uint32_t FTSR;  // Falling Trigger Selection Register
    volatile uint32_t SWIER; // Software Interrupt Event Register
    volatile uint32_t PR;    // Pending Register
} EXTI_TypeDef;

#define EXTI ((EXTI_TypeDef *) EXTI_BASE)

//Difining the triggering mode: 
#define RISING 0
#define FALLING 1
#define BOTH 2

//Define interrupt ports:

#define INT_PORT_A 0
#define INT_PORT_B 1
#define INT_PORT_C 2
#define INT_PORT_D 3
#define INT_PORT_E 4

/*************************************************************************************************/
/*************************__ARM cortex M3 core registers__*************************/
/*************************************************************************************************/

//Interrupt set-enable registers (NVIC_ISERx)
#define NVIC_ISER0 *(volatile uint32_t *) 0xE000E100  //offset is 0x00
#define NVIC_ISER1 *(volatile uint32_t *) 0xE000E104  //offset is 0x04

//System control block:
//system control block base register address
#define SCB_BASE (0xE000ED00UL)

typedef struct {
    volatile uint32_t CPUID;   // 0x00 - CPU ID Base Register
    volatile uint32_t ICSR;    // 0x04 - Interrupt Control and State Register
    volatile uint32_t VTOR;    // 0x08 - Vector Table Offset Register
    volatile uint32_t AIRCR;   // 0x0C - Application Interrupt and Reset Control Register
    volatile uint32_t SCR;     // 0x10 - System Control Register
    volatile uint32_t CCR;     // 0x14 - Configuration and Control Register
    volatile uint8_t  SHP[12]; // 0x18 - System Handler Priority Registers (SHP1–SHP3)
    volatile uint32_t SHCSR;   // 0x24 - System Handler Control and State Register
    volatile uint32_t CFSR;    // 0x28 - Configurable Fault Status Register
    volatile uint32_t HFSR;    // 0x2C - HardFault Status Register
    volatile uint32_t DFSR;    // 0x30 - Debug Fault Status Register
    volatile uint32_t MMFAR;   // 0x34 - MemManage Fault Address Register
    volatile uint32_t BFAR;    // 0x38 - BusFault Address Register
    volatile uint32_t AFSR;    // 0x3C - Auxiliary Fault Status Register
} SCB_Type;


#define SCB ((SCB_Type *) SCB_BASE)


	

/*************************************************************************************************/
/*************************__Systic__*************************/
/*************************************************************************************************/

// Define SysTick base address
#define SYSTICK_BASE (0xE000E010)

// Define the SysTick structure
typedef struct {
    volatile uint32_t CTRL;   // SysTick Control and Status Register
    volatile uint32_t LOAD;   // SysTick Reload Value Register
    volatile uint32_t VAL;    // SysTick Current Value Register
    volatile uint32_t CALIB;  // SysTick Calibration Value Register
} SysTick_TypeDef;

// Define a pointer to the SysTick structure at the base address
#define SysTick ((SysTick_TypeDef *) SYSTICK_BASE)


/*************************************************************************************************/
/*************************__Timers__*************************/
/*************************************************************************************************/
// Define Timer base addresses
#define TIM1_BASE (0x40012C00)
#define TIM2_BASE (0x40000000)
#define TIM3_BASE (0x40000400)
#define TIM4_BASE (0x40000800)

// Define the Timer structure
typedef struct {
    volatile uint32_t CR1;    // Control Register 1 - basic timer control (enable, direction, etc.)
    volatile uint32_t CR2;    // Control Register 2 - additional control features
    volatile uint32_t SMCR;   // Slave Mode Control Register - for synchronization and triggering
    volatile uint32_t DIER;   // DMA/Interrupt Enable Register - enables update/comparison interrupts or DMA
    volatile uint32_t SR;     // Status Register - holds status flags (e.g., update flag)
    volatile uint32_t EGR;    // Event Generation Register - manually trigger events like update
    volatile uint32_t CCMR1;  // Capture/Compare Mode Register 1 - config for channels 1 and 2
    volatile uint32_t CCMR2;  // Capture/Compare Mode Register 2 - config for channels 3 and 4
    volatile uint32_t CCER;   // Capture/Compare Enable Register - enable outputs for each channel
    volatile uint32_t CNT;    // Counter - holds the current counter value
    volatile uint32_t PSC;    // Prescaler - divides the input clock to slow down the timer
    volatile uint32_t ARR;    // Auto-Reload Register - sets the timer's period
    volatile uint32_t RCR;    // Repetition Counter Register - used in advanced timers for PWM
    volatile uint32_t CCR1;   // Capture/Compare Register 1 - value for channel 1
    volatile uint32_t CCR2;   // Capture/Compare Register 2 - value for channel 2
    volatile uint32_t CCR3;   // Capture/Compare Register 3 - value for channel 3
    volatile uint32_t CCR4;   // Capture/Compare Register 4 - value for channel 4
    volatile uint32_t BDTR;   // Break and Dead-Time Register - used in advanced timers for safety features
    volatile uint32_t DCR;    // DMA Control Register - configures how timer works with DMA
    volatile uint32_t DMAR;   // DMA Address for Full Transfer - holds memory address for DMA
} Timer_TypeDef;


// Define Timer base pointers
#define TIM1 ((Timer_TypeDef *)TIM1_BASE)
#define TIM2 ((Timer_TypeDef *)TIM2_BASE)
#define TIM3 ((Timer_TypeDef *)TIM3_BASE)
#define TIM4 ((Timer_TypeDef *)TIM4_BASE)


#define timer1   1
#define timer2   2
#define timer3   3
#define timer4   4


/*************************************************************************************************/
/*************************__USART__*************************/
/*************************************************************************************************/
// Define USART base addresses
#define USART1_BASE ( 0x40013800)
#define USART2_BASE (0x40004400)
#define USART3_BASE (0x40004800)


// Define the structure for USART registers
typedef struct
{
    volatile uint32_t SR;    // Status Register (USART_SR)
    volatile uint32_t DR;    // Data Register (USART_DR)
    volatile uint32_t BRR;   // Baud Rate Register (USART_BRR)
    volatile uint32_t CR1;   // Control Register 1 (USART_CR1)
    volatile uint32_t CR2;   // Control Register 2 (USART_CR2)
    volatile uint32_t CR3;   // Control Register 3 (USART_CR3)
    volatile uint32_t GTPR;  // Guard Time and Prescaler Register (USART_GTPR)
} USART_TypeDef;

//define usart based pointers: 
#define USART1 ((USART_TypeDef *)USART1_BASE)
#define USART2 ((USART_TypeDef *)USART2_BASE)
#define USART3 ((USART_TypeDef *)USART3_BASE)

#define USART_1   1  //Usart 1
#define USART_2   2  //Usart 2
#define USART_3   3  //Usart 3



/*************************************************************************************************/
/*************************__SPI__*************************/
/*************************************************************************************************/
// Define SPI base addresses
#define SPI1_BASE (0x40013000)
#define SPI2_BASE (0x40003800)

// Define the structure for SPI registers
typedef struct
{
    volatile uint32_t CR1;    // Control Register 1 (SPI_CR1)
    volatile uint32_t CR2;    // Control Register 2 (SPI_CR2)
    volatile uint32_t SR;     // Status Register (SPI_SR)
    volatile uint32_t DR;     // Data Register (SPI_DR)
    volatile uint32_t CRCPR;  // CRC Polynomial Register (SPI_CRCPR)
    volatile uint32_t RXCRCR; // RX CRC Register (SPI_RXCRCR)
    volatile uint32_t TXCRCR; // TX CRC Register (SPI_TXCRCR)
    volatile uint32_t I2SCFGR;// I2S Configuration Register (SPI_I2SCFGR)
    volatile uint32_t I2SPR;  // I2S Prescaler Register (SPI_I2SPR)
} SPI_TypeDef;

// Define SPI peripheral pointers
#define SPI1 ((SPI_TypeDef *)SPI1_BASE)
#define SPI2 ((SPI_TypeDef *)SPI2_BASE)

// Define SPI identifiers
#define SPI_1 1   // SPI1
#define SPI_2 2   // SPI2


/*************************************************************************************************/
/*************************__I2C__*************************/
/*************************************************************************************************/
// Define I2C base addresses
#define I2C1_BASE (0x40005400)
#define I2C2_BASE (0x40005800)

// Define the structure for I2C registers
typedef struct
{
    volatile uint32_t CR1;       // Control Register 1 (I2C_CR1)
    volatile uint32_t CR2;       // Control Register 2 (I2C_CR2)
    volatile uint32_t OAR1;      // Own Address Register 1 (I2C_OAR1)
    volatile uint32_t OAR2;      // Own Address Register 2 (I2C_OAR2)
    volatile uint32_t DR;        // Data Register (I2C_DR)
    volatile uint32_t SR1;       // Status Register 1 (I2C_SR1)
    volatile uint32_t SR2;       // Status Register 2 (I2C_SR2)
    volatile uint32_t CCR;       // Clock Control Register (I2C_CCR)
    volatile uint32_t TRISE;     // TRISE Register (I2C_TRISE)
} I2C_TypeDef;

// Define I2C peripheral pointers
#define I2C1 ((I2C_TypeDef *)I2C1_BASE)
#define I2C2 ((I2C_TypeDef *)I2C2_BASE)

// Define I2C identifiers
#define I2C_1 1   // I2C1
#define I2C_2 2   // I2C2


//I2c mode
#define i2c_FastMode 0x2d
#define i2c_StandardMode 0xB4


/*************************************************************************************************/
/*************************__I2C_OLED__*************************/
/*************************************************************************************************/
// Define inversion modes
#define INVERTED     1
#define NON_INVERTED   0




/*************************************************************************************************/
/*************************__DWT Cycle Counter_Registers__*************************/
/*************************************************************************************************/
#define DEMCR       (*(volatile unsigned int*)0xE000EDFC)
#define DWT_CTRL    (*(volatile unsigned int*)0xE0001000)
#define DWT_CYCCNT  (*(volatile unsigned int*)0xE0001004)


/*************************************************************************************************/
/*************************__ADC__*************************/
/*************************************************************************************************/
// Define ADC base addresses
#define ADC1_BASE (0x40012400)
#define ADC2_BASE (0x40012800)

// Define the structure for ADC registers
typedef struct
{
    volatile uint32_t SR;        // Status Register (ADC_SR)
    volatile uint32_t CR1;       // Control Register 1 (ADC_CR1)
    volatile uint32_t CR2;       // Control Register 2 (ADC_CR2)
    volatile uint32_t SMPR1;     // Sample Time Register 1 (ADC_SMPR1)
    volatile uint32_t SMPR2;     // Sample Time Register 2 (ADC_SMPR2)
    volatile uint32_t JOFR1;     // Injected Channel Data Offset Register 1 (ADC_JOFR1)
    volatile uint32_t JOFR2;     // Injected Channel Data Offset Register 2 (ADC_JOFR2)
    volatile uint32_t JOFR3;     // Injected Channel Data Offset Register 3 (ADC_JOFR3)
    volatile uint32_t JOFR4;     // Injected Channel Data Offset Register 4 (ADC_JOFR4)
    volatile uint32_t HTR;       // Watchdog Higher Threshold Register (ADC_HTR)
    volatile uint32_t LTR;       // Watchdog Lower Threshold Register (ADC_LTR)
    volatile uint32_t SQR1;      // Regular Sequence Register 1 (ADC_SQR1)
    volatile uint32_t SQR2;      // Regular Sequence Register 2 (ADC_SQR2)
    volatile uint32_t SQR3;      // Regular Sequence Register 3 (ADC_SQR3)
    volatile uint32_t JSQR;      // Injected Sequence Register (ADC_JSQR)
    volatile uint32_t JDR1;      // Injected Data Register 1 (ADC_JDR1)
    volatile uint32_t JDR2;      // Injected Data Register 2 (ADC_JDR2)
    volatile uint32_t JDR3;      // Injected Data Register 3 (ADC_JDR3)
    volatile uint32_t JDR4;      // Injected Data Register 4 (ADC_JDR4)
    volatile uint32_t DR;        // Regular Data Register (ADC_DR)
} ADC_TypeDef;

// Define ADC peripheral pointers
#define ADC1 ((ADC_TypeDef *)ADC1_BASE)
#define ADC2 ((ADC_TypeDef *)ADC2_BASE)

// Define ADC identifiers
#define ADC_1 1   // ADC1
#define ADC_2 2   // ADC2



/*************************************************************************************************/
/*************************__RTC__*************************/
/*************************************************************************************************/
// Define RTC base addresses
#define RTC_BASE (0x40002800UL)

//define the structure of rtc registers
typedef struct {
    volatile uint32_t CRH;   // 0x00 - Control Register High (interrupt enables)
    volatile uint32_t CRL;   // 0x04 - Control Register Low (status flags)
    volatile uint32_t PRLH;  // 0x08 - Prescaler Load Register High
    volatile uint32_t PRLL;  // 0x0C - Prescaler Load Register Low
    volatile uint32_t DIVH;  // 0x10 - Divider Register High (read-only)
    volatile uint32_t DIVL;  // 0x14 - Divider Register Low (read-only)
    volatile uint32_t CNTH;  // 0x18 - Counter Register High
    volatile uint32_t CNTL;  // 0x1C - Counter Register Low
    volatile uint32_t ALRH;  // 0x20 - Alarm Register High
    volatile uint32_t ALRL;  // 0x24 - Alarm Register Low
} RTC_TypeDef;




/************************************************************************/
/*************************__Priority configuration __*************************/
/************************************************************************/

/* SCB base address and AIRCR offset */
#define SCB_BASE        (0xE000ED00UL)
#define SCB_AIRCR       (*(volatile uint32_t *)(SCB_BASE + 0x0C))

/* AIRCR Register fields */
#define SCB_AIRCR_VECTKEY_MASK     (0xFFFFUL << 16)
#define SCB_AIRCR_VECTKEY          (0x5FAUL << 16)
#define SCB_AIRCR_PRIGROUP_MASK    (0x7UL << 8)

/* Priority grouping values (only pre-emption bits, no subpriority) */
#define NVIC_PRIORITY_GROUP_0      (0x0UL << 8)  // 0 bits for subpriority, 4 bits for preemption
#define NVIC_PRIORITY_GROUP_1      (0x1UL << 8)
#define NVIC_PRIORITY_GROUP_2      (0x2UL << 8)
#define NVIC_PRIORITY_GROUP_3      (0x3UL << 8)
#define NVIC_PRIORITY_GROUP_4      (0x4UL << 8)  // 4 subpriority bits, 0 preemption (usually not




/************************************************************************/
/*************************__Function Prototypes__*************************/
/************************************************************************/

//GPIO:
void Config_GPIO(uint8_t port, uint8_t pin, uint8_t mode, uint8_t config);  //configuring GPIO pin
void Write_GPIO (uint8_t port, uint8_t pin, uint8_t state);	//write into gpio pin
uint32_t Read_GPIO_Pin (uint8_t port, uint8_t pin);   //read a single gpio pin in a specific port
uint32_t Read_GPIO_Port (uint8_t port); //read the entire gpio input data register
void toggle_gpio (uint8_t port, uint8_t pin);   //toggle a single gpio pin
void configure_gpio_interrupt(uint8_t pin, uint8_t port, uint8_t trigger_type,  uint8_t priority_level); //enabling external interrupt


//systic timer
void systic_init(void);          //SysTick iniialization
void Delay_Sys_US (uint16_t t,uint32_t clock_freq);  //SysTick delay in Microsecond
void Delay_Sys_MS (uint16_t t,uint32_t clock_freq);  //SysTick delay in Milisecond
void Systick_interrupt(uint32_t clock_freq, uint32_t interval_ms);				//SysTick interrupt


//General Purpose Timers

void delay_microSecond(uint8_t timer, uint16_t delay,uint32_t clock_freq); // Delays execution for a specified number of microseconds using the selected timer.
void delay_miliSecond(uint8_t timer, uint16_t delay,uint32_t clock_freq); // Delays execution for a specified number of milliseconds using the selected timer.
void timer_irq_microSecond_start (uint8_t timer, uint16_t delay,uint32_t clock_freq); // Starts the selected timer to generate an interrupt after a specified number of microseconds.
void timer_irq_milisecond_start (uint8_t timer, uint16_t delay,uint32_t clock_freq); // Starts the selected timer to generate an interrupt after a specified number of milliseconds.
void stop_timer_irq (uint8_t timer); // Stops the interrupt generation for the selected timer.
void stop_timer (uint8_t timer); // Completely stops the selected timer, including disabling its clock and interrupt.
void timer_compare_MhZ(uint32_t port, uint8_t pin, uint16_t Load_value, uint16_t compare_value,uint32_t clock_freq ); // Generate output compare signal in MHz range using specific GPIO pin and timer
void timer_compare_khZ(uint32_t port, uint8_t pin, uint16_t Load_value, uint16_t compare_value,uint32_t clock_freq ); // Generate output compare signal in kHz range using specific GPIO pin and timer
void timer_PWM_Microsecond(uint32_t port, uint8_t pin, uint16_t period, uint16_t duty_cycle_percentage,uint32_t clock_freq ); // Generate PWM signal in microsecond resolution using specific GPIO pin and timer
void timer_PWM_Milisecond(uint32_t port, uint8_t pin, uint16_t period, uint16_t duty_cycle_percentage,uint32_t clock_freq ); // Generate PWM signal in millisecond resolution using specific GPIO pin and timer
void update_period(uint32_t port, uint8_t pin, uint16_t new_period, uint16_t duty_percent); //Dynamic Period Update Function 
void update_duty(uint32_t port, uint8_t pin, uint16_t duty_percent); //Dynamic Duty Cycle Update Function 


//USART
void init_USART ( uint8_t usart, uint32_t baud_rate );
void USART_transmit(uint8_t usart, char c);
char USART_receive (uint8_t usart);
void init_usart_transmit_interrupt(uint8_t usart, uint32_t baud_rate,uint8_t priority_level);
void init_usart_receive_interrupt(uint8_t usart, uint32_t baud_rate,uint8_t priority_level);
void USART_send_string(uint8_t usart, const char *str);


//SPI
void init_SPI(uint8_t spi, uint32_t  prescaler);
void spi_transmit(uint8_t spi, char tx_char);
void spi_send_message (uint8_t spi, char message[]);
uint8_t spi_receive_simplex(uint8_t spi);
uint8_t spi_transmit_receive_duplex(uint8_t spi, uint8_t tx_data) ;


//I2C
void i2c_init ( uint8_t i2c, uint8_t speed );
void i2c_write (uint8_t i2c, uint8_t address, char data []);
void i2c_start (uint8_t i2c);
void i2c_send_address(uint8_t i2c, char address, uint8_t R_W );
void i2c_send_data (uint8_t i2c, char data);
uint8_t i2c_receive_data(uint8_t i2c, uint8_t ACK_NACK);
void i2c_stop (uint8_t i2c);



//ADC
void adc_init (uint8_t adc, uint8_t port, uint8_t pin); //adc init
uint8_t adc_data_ready(uint8_t adc); //function to check the data conversion is ready or not
uint16_t adc_read (uint8_t adc); //reading adc value


//RTC


//16X02 LCD 
void LCD1602_init (void);
void LCD1602_print (uint8_t line, uint16_t position, char message[]);
void LCD1602_scroll_from_right(char message[], uint8_t line, uint16_t delay_ms);
void LCD1602_scroll_from_left(char message[], uint8_t line, uint16_t delay_ms);

//I2C OLED
void oled_configure_64 (uint8_t i2c);
void oled_configure_32 (uint8_t i2c);
void oled_blank(uint8_t i2c);
void oled_send_message_8X8_font(uint8_t i2c, const char *str, uint8_t scale, uint8_t start_Y, uint16_t start_X,uint8_t inverted );
void oled_send_message_5X8_font(uint8_t i2c, const char *str, uint8_t scale, uint8_t start_Y, uint16_t start_X,uint8_t inverted );
void clear_oled_buffer (unsigned char oled_buffer[][128]);
void update_oled_buffer (lcd_image_typedef *img, uint8_t img_num, unsigned char oled_buffer[][128] );
void print_buffer (uint8_t i2c, unsigned char oled_buffer [][128]);


//Helper function:
void int_to_str(int8_t num, char buffer[]);


//System clock configuration
int ConfigureSystemClock_HSE(uint32_t hse_freq, uint32_t target_sysclk);
int ConfigureSystemClock_HSI(uint32_t target_sysclk);

//DWT cycle counter enable:
void dwt_init(void);

//priority grouping
void NVIC_SetPriorityGrouping(uint32_t priority_group);

//Set nvic priority:
void NVIC_SetPriority(uint8_t irq_number, uint8_t priority_level);

//low power: 
void sleep_on_exit(void);
 void enable_wfi(void);
 void enable_wfe(void);


 //DS3231 RTC

 void update_second (uint8_t i2c, uint8_t sec);
 void update_minute (uint8_t i2c, uint8_t minute);
 void update_hour (uint8_t i2c, uint8_t hour);
 void update_day(uint8_t i2c, uint8_t day);
 void update_month(uint8_t i2c, uint8_t month);
 void update_year(uint8_t i2c, uint8_t year);
 void update_time(uint8_t i2c, uint8_t hour, uint8_t minute, uint8_t second);
 void update_date(uint8_t i2c, uint8_t date, uint8_t month, uint8_t year);
 void get_time_and_date(uint8_t i2c, uint8_t *rtc_data);
 void format_date_string(uint8_t day, uint8_t month, uint8_t year, char *date_str);
 void format_time_string(uint8_t hour, uint8_t minute, char *time_str);


#endif // STM32F103DRIVER_H
