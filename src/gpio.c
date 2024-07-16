#include <stdint.h>

#include "gpio.h"


void gpio_set_mode(struct gpio *gpio, uint8_t pin, uint8_t mode) {
    gpio->MODER &= ~( 0b11 << ( pin * 2 ) );
    gpio->MODER |= ( mode << ( pin * 2 ) );
}

int gpio_set_af(struct gpio *gpio, uint8_t pin, uint8_t af) {
    if (pin > 15) {
        return -1;
    }
    if (af > 15) {
        return -2;
    }

    gpio->AFR[ pin / 8 ] &= ~( 0b1111 << ( ( pin % 8 ) * 4 ) );
    gpio->AFR[ pin / 8 ] |= ( af << ( ( pin % 8 ) * 4 ) );

    return 0;
}

int gpio_write(struct gpio *gpio, uint8_t pin, uint8_t val) {
    if (pin > 15) {
        return -1;
    }
    if (val > 1) {
    	return -2;
    }

    gpio->BSRR = ( ( 0b1 << pin ) << ( val == 1 ? 0 : 16 ) );

	return 0;
}

int8_t gpio_read(struct  gpio *gpio, uint8_t pin) {
    if (pin > 15) {
        return -1;
    }

    return (gpio->IDR & (1 << pin)) >> pin;
}
