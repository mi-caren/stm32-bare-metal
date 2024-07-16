#ifndef RCC_H
#define RCC_H

#include <stdint.h>


#define RCC                 ( ( struct rcc * ) 0x58000000 )


struct rcc {
    volatile uint32_t CR;
    volatile uint32_t ICSCR;
    volatile uint32_t CFGR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t _RESERVED1[2];
    volatile uint32_t CIER;
    volatile uint32_t CIFR;
    volatile uint32_t CICR;
    volatile uint32_t _RESERVED2;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    volatile uint32_t _RESERVED3;
    volatile uint32_t APB1RSTR1;
    volatile uint32_t APB1RSTR2;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB3RSTR;
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    volatile uint32_t _RESERVED4;
    volatile uint32_t APB1ENR1;
    volatile uint32_t APB1ENR2;
    volatile uint32_t APB2ENR;
    volatile uint32_t APB3ENR;
    volatile uint32_t AHB1SMENR;
    volatile uint32_t AHB2SMENR;
    volatile uint32_t AHB3SMENR;
    volatile uint32_t APB1SMENR1;
    volatile uint32_t APB1SMENR2;
    volatile uint32_t APB2SMENR;
    volatile uint32_t APB3SMENR;
    volatile uint32_t CCIPR;
    volatile uint32_t _RESERVED5;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
};


enum {
    // AHB2ENR bits
    RCC_AHB2ENR_BIT_GPIOAEN         = 1 << 0,
    RCC_AHB2ENR_BIT_GPIOBEN         = 1 << 1,
    RCC_AHB2ENR_BIT_GPIOCEN         = 1 << 2,
    RCC_AHB2ENR_BIT_GPIOHEN         = 1 << 7,

    // APB1ENR2 bits
    RCC_APB1ENR2_BIT_LPUART1EN      = 1 << 0,
    RCC_APB1ENR2_BIT_LPTIM2EN       = 1 << 5,
    RCC_APB1ENR2_BIT_LPTIM3EN       = 1 << 6,

    // APB2ENR bits
    RCC_APB2ENR2_BIT_ADCEN          = 1 << 9,
    RCC_APB2ENR2_BIT_TIM1EN         = 1 << 11,
    RCC_APB2ENR2_BIT_SPI1EN         = 1 << 12,
    RCC_APB2ENR2_BIT_USART1EN       = 1 << 14,
    RCC_APB2ENR2_BIT_TIM16EN        = 1 << 17,
    RCC_APB2ENR2_BIT_TIM17EN        = 1 << 18,
};

void rcc_gpio_ck_enable(uint32_t bits);
void rcc_lp_peripheral_ck_enable(uint32_t bits);

#endif
