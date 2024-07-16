#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>


#define GPIO_MEM_ADDR		0x48000000

#define GPIOA_OFFSET		0x0000
#define GPIOB_OFFSET		0x0400

#define GPIOA               ( ( struct gpio * ) ( GPIO_MEM_ADDR + GPIOA_OFFSET ) )
#define GPIOB               ( ( struct gpio * ) ( GPIO_MEM_ADDR + GPIOB_OFFSET ) )


struct gpio {
    // configuration registers
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;

    // data registers
    volatile uint32_t IDR;
    volatile uint32_t ODR;

    // set/reset register
    volatile uint32_t BSRR;

    // locking register
    volatile uint32_t LCKR;

    // alternate function registers
    volatile uint32_t AFR[2];
    volatile uint32_t BRR;
};

enum {
    GPIO_MODE_INPUT,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_ALTERNATE_FUNCTION,
    GPIO_MODE_ANALOG
};

enum {
    // PA2 alternate functions
    GPIO_PA2_AF_LSCO                = 0,
    GPIO_PA2_AF_TIM2_CH3            = 1,
    GPIO_PA2_AF_USART2_TX           = 7,
    GPIO_PA2_AF_LPUART1_TX          = 8,
    GPIO_PA2_AF_COMP2_OUT           = 12,
    GPIO_PA2_AF_DEBUG_PWR_LDORDY    = 13,
    GPIO_PA2_AF_CM4_EVENTOUT        = 15,

    // PA3 alternate functions
    GPIO_PA3_AF_TIM2_CH4            = 1,
    GPIO_PA3_AF_I2S2_MCK            = 5,
    GPIO_PA3_AF_USART2_RX           = 7,
    GPIO_PA3_AF_LPUART1_RX          = 8,
    GPIO_PA3_AF_CM4_EVENTOUT        = 15,

    // PA9 alternate functions
    GPIO_PA9_AF_TIM1_CH2            = 1,
    GPIO_PA9_AF_SPI2_NSS            = 3,
    GPIO_PA9_AF_I2S2_WS             = 3,
    GPIO_PA9_AF_I2C1_SCL            = 4,
    GPIO_PA9_AF_SPI2_SCK            = 5,
    GPIO_PA9_AF_I2S2_CK             = 5,
    GPIO_PA9_AF_USART1_TX           = 7,
    GPIO_PA9_AF_CM4_EVENTOUT        = 15,

    // PA10 alternate functions
    GPIO_PA10_AF_RTC_REFIN           = 0,
    GPIO_PA10_AF_TIM1_CH3            = 1,
    GPIO_PA10_AF_I2C1_SDA            = 4,
    GPIO_PA10_AF_SPI2_MOSI           = 5,
    GPIO_PA10_AF_I2S2_SD             = 5,
    GPIO_PA10_AF_USART1_RX           = 7,
    GPIO_PA10_AF_DEBUG_RF_HSE32RDY   = 13,
    GPIO_PA10_AF_TIM17_BKIN          = 14,
    GPIO_PA10_AF_CM4_EVENTOUT        = 15,
};

enum {
	GPIO_PIN_0,
	GPIO_PIN_1,
	GPIO_PIN_2,
	GPIO_PIN_3,
	GPIO_PIN_4,
	GPIO_PIN_5,
	GPIO_PIN_6,
	GPIO_PIN_7,
	GPIO_PIN_8,
	GPIO_PIN_9,
	GPIO_PIN_10,
	GPIO_PIN_11,
	GPIO_PIN_12,
	GPIO_PIN_13,
	GPIO_PIN_14,
	GPIO_PIN_15
};

enum {
    LOW,
    HIGH
};

void gpio_set_mode(struct gpio *gpio, uint8_t pin, uint8_t mode);

int gpio_set_af(struct gpio *gpio, uint8_t pin, uint8_t af);

int gpio_write(struct gpio *gpio, uint8_t pin, uint8_t val);
int8_t gpio_read(struct  gpio *gpio, uint8_t pin);

#endif
