/* ####################################
Author: MD. Soyabbir Abu Hanif
 MSc in Embedded Systems
Tampere University, Finland
Email: hanifseceee535@gmail.com 
######################################
*/





#ifndef HEADER_H
#define HEADER_H 
#include <stdint.h>



//*************************__SysTick__*************************

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


//*************************__IRQ__*************************

#define __enableirq()  __asm("CPSIE I")  // Enable interrupts globally
#define __disableirq() __asm("CPSID I")  // Disable interrupts globally


//*************************__Timers__*************************

// Define Timer base addresses
#define TIM1_BASE (0x40012C00)
#define TIM2_BASE (0x40000000)
#define TIM3_BASE (0x40000400)
#define TIM4_BASE (0x40000800)

// Define the Timer structure
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t BDTR;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
} Timer_TypeDef;

// Define Timer base pointers
#define TIM1 ((Timer_TypeDef *)TIM1_BASE)
#define TIM2 ((Timer_TypeDef *)TIM2_BASE)
#define TIM3 ((Timer_TypeDef *)TIM3_BASE)
#define TIM4 ((Timer_TypeDef *)TIM4_BASE)


#define t1   1  //timer 1
#define t2   2  //timer 2
#define t3   3  //timer 3
#define t4   4  //timer 4


/*************************__USART__*************************/

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




/*************************__SPI__*************************/

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


/*************************__I2C__*************************/

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



/*************************__RCC__*************************/
// Base Addresses
#define RCCAPB2       *(volatile uint32_t *) 0x40021018
#define RCCAPB1       *(volatile uint32_t *) 0x4002101C


/*************************__GPIO__*************************/
// Define the structure for GPIO registers
typedef struct
{
    volatile uint32_t CRL;        // Configuration Register Low (GPIOx_CR[0-7])
    volatile uint32_t CRH;        // Configuration Register High (GPIOx_CR[8-15])
    volatile uint32_t IDR;        // Input Data Register (GPIOx_IDR)
    volatile uint32_t ODR;        // Output Data Register (GPIOx_ODR)
    volatile uint32_t BSRR;       // Bit Set/Reset Register (GPIOx_BSRR)
    volatile uint32_t BRR;        // Bit Reset Register (GPIOx_BRR)
    volatile uint32_t LCKR;       // Lock Register (GPIOx_LCKR)
} GPIO_TypeDef;

// Define GPIO peripheral pointers
#define GPIOA ((GPIO_TypeDef *)0x40010800)
#define GPIOB ((GPIO_TypeDef *)0x40010C00)
#define GPIOC ((GPIO_TypeDef *)0x40011000)
#define GPIOD ((GPIO_TypeDef *)0x40011400)
#define GPIOE ((GPIO_TypeDef *)0x40011800)






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
#define open_drain_output   1
#define af_pp_output        2
#define af_od_output        3

#define HIGH                1
#define LOW                 0

#define SET                 1
#define CLEAR               0


/*************************__External interrupt__*************************/

// Interrupt Configuration
#define INT_PORT_A          0
#define INT_PORT_B          1
#define INT_PORT_C          2
#define INT_PORT_D          3
#define INT_PORT_E          4

#define RISING              0
#define FALLING             1
#define BOTH                2

#define AFIO_EXTR1          *(volatile uint32_t *) 0x40010008
#define AFIO_EXTR2          *(volatile uint32_t *) 0x4001000C
#define AFIO_EXTR3          *(volatile uint32_t *) 0x40010010
#define AFIO_EXTR4          *(volatile uint32_t *) 0x40010014

#define EXTI_IMR            *(volatile uint32_t *) 0x40010400
#define EXTI_PR             *(volatile uint32_t *) 0x40010414
#define EXTI_RTSR           *(volatile uint32_t *) 0x40010408
#define EXTI_FTSR           *(volatile uint32_t *) 0x4001040C

#define NVIC_ISER0          *(volatile uint32_t *) 0xE000E100
#define NVIC_ISER1          *(volatile uint32_t *) 0xE000E104






//####################################################################################################
// Function Prototypes
void config_gpio(uint32_t port, uint32_t pin, uint32_t mode, uint32_t config);
void write_gpio(uint32_t port, uint32_t pin, uint32_t state);
uint32_t read_gpio(uint32_t port, uint32_t pin);
void toggle_gpio(uint32_t port, uint32_t pin);
void set_gpio_odr (uint32_t port, uint32_t value, uint8_t state );


void configure_gpio_interrupt(uint8_t pin, uint8_t port, uint8_t trigger_type);
void delay_microSecond(uint8_t timer, uint16_t delay);
void delay_miliSecond(uint8_t timer, uint16_t delay);
void timer_irq_microSecond_start (uint8_t timer, uint16_t delay);
void timer_irq_milisecond_start (uint8_t timer, uint16_t delay);
void stop_timer_irq (uint8_t timer);
void stop_timer (uint8_t timer);
void timer_compare_MhZ(uint32_t port, uint8_t pin, uint16_t Load_value, uint16_t compare_value );
void timer_compare_khZ(uint32_t port, uint8_t pin, uint16_t Load_value, uint16_t compare_value );
void timer_PWM_Microsecond(uint32_t port, uint8_t pin, uint16_t period, uint16_t duty_cycle_percentage );
void timer_PWM_Milisecond(uint32_t port, uint8_t pin, uint16_t period, uint16_t duty_cycle_percentage );
void Delay_Sys_US (uint16_t t);
void Delay_Sys_MS (uint16_t t);
void systic_init(void);
void Systic_interrupt ();
void init_USART ( uint8_t usart, uint32_t baud_rate );
void USART_transmit(uint8_t usart, char c);
char USART_receive (uint8_t usart);
void init_usart_transmit_interrupt(uint8_t usart, uint32_t baud_rate);
void init_usart_receive_interrupt(uint8_t usart, uint32_t baud_rate);
void init_SPI(uint8_t spi, uint32_t  prescaler);
void spi_transmit(uint8_t spi, char tx_char);
void spi_send_message (uint8_t spi, char message[]);


void i2c_init ( uint8_t i2c, uint8_t speed );
void i2c_write (uint8_t i2c, uint8_t address, char data []);
void i2c_start (uint8_t i2c);
void i2c_send_address(uint8_t i2c, char address, uint8_t R_W );
void i2c_send_data (uint8_t i2c, char data);
void i2c_stop (uint8_t i2c);
void oled_configure_64 (uint8_t i2c);
void oled_configure_32 (uint8_t i2c);
void oled_send_message (uint8_t i2c, uint8_t Y_position, uint8_t X_position, char message[]);
void oled_blank(uint8_t i2c);


//void LCD1602_Send_data ( unsigned char data );
void LCD1602_init (void);
//void lcd1602_send_string (char text[]);
void LCD1602_print (uint8_t line, uint16_t position, char message[]);
void LCD1602_scroll_from_right(char message[], uint8_t line, uint16_t delay_ms);
void LCD1602_scroll_from_left(char message[], uint8_t line, uint16_t delay_ms);


#endif // HEADER_H
