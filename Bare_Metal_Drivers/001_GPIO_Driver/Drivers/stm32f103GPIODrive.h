/* ####################################
Author: MD. Soyabbir Abu Hanif
 MSc in Embedded Systems
Tampere University, Finland
Email: hanifseceee535@gmail.com 
######################################
*/


#ifndef STM32F103GPIODRIVE_H
#define STM32F103GPIODRIVE_H
#include <stdint.h>

/*************************__GPIO__*************************/

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




/*************************__RCC_Registers__*************************/
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






/*************************__Function Prototypes__*************************/

//GPIO:
void Config_GPIO(uint8_t port, uint8_t pin, uint8_t mode, uint8_t config);  //configuring GPIO pin
void Write_GPIO (uint8_t port, uint8_t pin, uint8_t state);	//write into gpio pin
uint32_t Read_GPIO_Pin (uint8_t port, uint8_t pin);   //read a single gpio pin in a specific port
uint32_t Read_GPIO_Port (uint8_t port); //read the entire gpio input data register
void toggle_gpio (uint8_t port, uint8_t pin);   //toggle a single gpio pin





#endif // STM32F103GPIODRIVE_H