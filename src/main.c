#include <stdint.h>
#include <stdbool.h>

#define BIT(x)            (1UL << (x))
#define PIN(bank, num)    ((((bank) - 'A') << 8) | (num))
#define PINNO(pin)        ((pin) & 0xf)
#define PINBANK(pin)      ((pin) >> 8)


#define GPIO(bank)        (( struct gpio * ) ( 0x48000000 + (0x0400 * (bank))))
#define RCC               (( struct rcc * )  ( 0x58000000 ))


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
    GPIO_MODE_INPUT,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_ALTERNATE_FUNCTION,
    GPIO_MODE_ANALOG
};


static void gpio_set_mode(uint16_t pin, uint8_t mode) {
    struct gpio* gpio = GPIO(PINBANK(pin));
    uint8_t n = PINNO(pin);
    gpio->MODER &= ~( 0b11 << ( n * 2 ) );
    gpio->MODER |= ( mode << ( n * 2 ) );
}

static inline void gpio_write(uint16_t pin, bool val) {
    struct gpio* gpio = GPIO(PINBANK(pin));
    gpio->BSRR = ( 1U << PINNO(pin) ) << ( val ? 0 : 16 );
}

static inline void spin(volatile uint32_t count) {
    while (count--) (void) 0;
}






int main(void) {
    uint16_t led = PIN('B', 15);
    RCC->AHB2ENR |= BIT(PINBANK(led));
    gpio_set_mode(led, GPIO_MODE_OUTPUT);

    for (;;) {
        gpio_write(led, true);
        spin(999999);
        gpio_write(led, false);
        spin(999999);
    };
    return 0;
}







__attribute__((naked, noreturn)) void _reset(void) {
	extern long _data_start, _data_end, _data_loadaddr, _bss_start, _bss_end;

    for (long *bss_p = &_bss_start; bss_p < &_bss_end; bss_p++) {
        *bss_p = 0;
    }

    long *data_lvm_p = &_data_loadaddr;
    long *data_sram1_p = &_data_start;
    while (data_sram1_p < &_data_end) {
        *data_sram1_p = *data_lvm_p;
        data_sram1_p++;
        data_lvm_p++;
    }

    main();

    for(;;) (void) 0;
}

extern void _estack(void);

// vector table. 16 cortex-m4 specific interrupts + 62 stm32wl55 interrupts
__attribute__((section(".vectors"))) void (*tab[16 + 62]) (void) = {
	_estack,
	_reset,
	// 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	// systick_handler
};
