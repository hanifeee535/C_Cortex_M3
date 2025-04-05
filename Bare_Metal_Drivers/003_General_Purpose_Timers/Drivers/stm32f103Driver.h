#ifndef STM32F103DRIVER_H
#define STM32F103DRIVER_H
#include <stdint.h>



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
/*************************__Interrupt set-enable registers (NVIC_ISERx)__*************************/
/*************************************************************************************************/


#define NVIC_ISER0 *(volatile uint32_t *) 0xE000E100  //offset is 0x00
#define NVIC_ISER1 *(volatile uint32_t *) 0xE000E104  //offset is 0x04
	

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




/************************************************************************/
/*************************__Function Prototypes__*************************/
/************************************************************************/

//GPIO:
void Config_GPIO(uint8_t port, uint8_t pin, uint8_t mode, uint8_t config);  //configuring GPIO pin
void Write_GPIO (uint8_t port, uint8_t pin, uint8_t state);	//write into gpio pin
uint32_t Read_GPIO_Pin (uint8_t port, uint8_t pin);   //read a single gpio pin in a specific port
uint32_t Read_GPIO_Port (uint8_t port); //read the entire gpio input data register
void toggle_gpio (uint8_t port, uint8_t pin);   //toggle a single gpio pin
void configure_gpio_interrupt(uint8_t pin, uint8_t port, uint8_t trigger_type); //enabling external interrupt

void Delay_Sys_US (uint16_t t);  //SysTick delay in Microsecond
void Delay_Sys_MS (uint16_t t);  //SysTick delay in Milisecond
void systic_init(void);          //SysTick iniialization
void Systic_interrupt ();				//SysTick interrupt



void delay_microSecond(uint8_t timer, uint16_t delay); // Delays execution for a specified number of microseconds using the selected timer.
void delay_miliSecond(uint8_t timer, uint16_t delay); // Delays execution for a specified number of milliseconds using the selected timer.
void timer_irq_microSecond_start (uint8_t timer, uint16_t delay); // Starts the selected timer to generate an interrupt after a specified number of microseconds.
void timer_irq_milisecond_start (uint8_t timer, uint16_t delay); // Starts the selected timer to generate an interrupt after a specified number of milliseconds.
void stop_timer_irq (uint8_t timer); // Stops the interrupt generation for the selected timer.
void stop_timer (uint8_t timer); // Completely stops the selected timer, including disabling its clock and interrupt.
void timer_compare_MhZ(uint32_t port, uint8_t pin, uint16_t Load_value, uint16_t compare_value ); // Generate output compare signal in MHz range using specific GPIO pin and timer
void timer_compare_khZ(uint32_t port, uint8_t pin, uint16_t Load_value, uint16_t compare_value ); // Generate output compare signal in kHz range using specific GPIO pin and timer
void timer_PWM_Microsecond(uint32_t port, uint8_t pin, uint16_t period, uint16_t duty_cycle_percentage ); // Generate PWM signal in microsecond resolution using specific GPIO pin and timer
void timer_PWM_Milisecond(uint32_t port, uint8_t pin, uint16_t period, uint16_t duty_cycle_percentage ); // Generate PWM signal in millisecond resolution using specific GPIO pin and timer
void update_period(uint32_t port, uint8_t pin, uint16_t new_period, uint16_t duty_percent); //Dynamic Period Update Function 
void update_duty(uint32_t port, uint8_t pin, uint16_t duty_percent); //Dynamic Duty Cycle Update Function 

#endif // STM32F103DRIVER_H