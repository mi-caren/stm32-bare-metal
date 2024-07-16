#include "rcc.h"

void rcc_gpio_ck_enable(uint32_t bits) {
    RCC->AHB2ENR |= bits &
        ( RCC_AHB2ENR_BIT_GPIOAEN | RCC_AHB2ENR_BIT_GPIOBEN
        | RCC_AHB2ENR_BIT_GPIOCEN | RCC_AHB2ENR_BIT_GPIOHEN );
}

void rcc_lp_peripheral_ck_enable(uint32_t bits) {
    RCC->APB1ENR2 |= bits &
        ( RCC_APB1ENR2_BIT_LPUART1EN | RCC_APB1ENR2_BIT_LPTIM2EN
        | RCC_APB1ENR2_BIT_LPTIM3EN );
}
