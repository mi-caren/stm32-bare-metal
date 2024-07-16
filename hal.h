#pragma once

#include <stdint.h>
#include <stdbool.h>


/* ---------------------------------------------------- */
/* --------------- DEFINES ---------------------------- */
/* ---------------------------------------------------- */

#define BIT(x)            (1UL << (x))
#define PIN(bank, num)    ((((bank) - 'A') << 8) | (num))
#define PINNO(pin)        ((pin) & 0xf)
#define PINBANK(pin)      ((pin) >> 8)

#define GPIO(bank)        (( struct gpio * )    ( 0x48000000 + (0x0400 * (bank))))
#define RCC               (( struct rcc * )       0x58000000 )
#define USART2            (( struct uart * )      0x40004400 )
#define LPUART1           (( struct uart * )      0x40008000 )
#define USART1            (( struct uart * )      0x40013800 )
#define SYSTICK           (( struct systick * )   0xe000e010 )


#define FREQ              4000000    // Cpu frequency, 4Mhz

/* ---------------------------------------------------- */
/* --------------- STRUCTS ---------------------------- */
/* ---------------------------------------------------- */



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

struct uart {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t BRR;
    volatile uint32_t GTPR;
    volatile uint32_t RTOR;
    volatile uint32_t RQR;
    volatile uint32_t ISR;
    volatile uint32_t ICR;
    volatile uint32_t RDR;
    volatile uint32_t TDR;
    volatile uint32_t PRESC;
};

struct systick {
	volatile uint32_t CTRL;
	volatile uint32_t LOAD;
	volatile uint32_t VAL;
	volatile uint32_t CALIB;
};




/* ---------------------------------------------------- */
/* --------------- ENUMS ------------------------------ */
/* ---------------------------------------------------- */



enum {
    GPIO_MODE_INPUT,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_ALTERNATE_FUNCTION,
    GPIO_MODE_ANALOG
};


/* ---------------------------------------------------- */
/* --------------- FUNCTIONS -------------------------- */
/* ---------------------------------------------------- */


static inline void spin(volatile uint32_t count) {
    while (count--) (void) 0;
}


static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
    struct gpio* gpio = GPIO(PINBANK(pin));
    uint8_t n = PINNO(pin);
    RCC->AHB2ENR |= BIT(PINBANK(pin));
    gpio->MODER &= ~( 0b11U << ( n * 2 ) );
    gpio->MODER |= ( mode << ( n * 2 ) );
}

static inline void gpio_set_af(uint16_t pin, uint8_t af_num) {
    struct gpio* gpio = GPIO(PINBANK(pin));
    uint8_t n = PINNO(pin);
    gpio->AFR[ pin / 8 ] &= ~( 0b1111U << ( ( n % 8 ) * 4 ) );
    gpio->AFR[ pin / 8 ] |= af_num << ( ( n % 8 ) * 4 );
}

static inline void gpio_write(uint16_t pin, bool val) {
    struct gpio* gpio = GPIO(PINBANK(pin));
    gpio->BSRR = ( 1U << PINNO(pin) ) << ( val ? 0 : 16 );
}



static inline void uart_init(struct uart* uart, unsigned long baud) {
    uint8_t af = 8;
    uint16_t rx = 0, tx = 0;

    if (uart == LPUART1) {
        RCC->APB1ENR2 |= BIT(0);
        tx = PIN('A', 2);
        rx = PIN('A', 3);
    }

    gpio_set_mode(tx, GPIO_MODE_ALTERNATE_FUNCTION);
    gpio_set_af(tx, af);

    gpio_set_mode(rx, GPIO_MODE_ALTERNATE_FUNCTION);
    gpio_set_af(rx, af);

    uart->CR1 = 0;    // reset this uart
    uart->BRR = ( ( 256 * FREQ ) ) / baud;
    uart->CR1 = BIT(3) | BIT(2) | BIT(0);    // set TE, RE, UE
}

static inline int uart_read_ready(struct uart* uart) {
    return uart->ISR & BIT(5);    // If RXNE bit is set, data is ready
}

static inline uint8_t uart_read_byte(struct uart* uart) {
    return (uint8_t) uart->RDR & 0xff;
}

static inline void uart_write_byte(struct uart *uart, uint8_t byte) {
  uart->TDR = byte;
  while ((uart->ISR & BIT(7)) == 0) spin(1);
}

static inline void uart_write_buf(struct uart *uart, char *buf, size_t len) {
  while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}



static inline void systick_init(uint32_t ticks) {
    if ((ticks - 1) > 0xffffff) return;
    SYSTICK->LOAD = ticks - 1;
    SYSTICK->VAL = 0;
    SYSTICK->CTRL |= BIT(0) | BIT(1) | BIT(2);
}


// t: expiration time, prd: period, now: current time. Return true if expired
static inline bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // Firt poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}

