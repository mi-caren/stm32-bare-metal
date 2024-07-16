#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "hal.h"

static volatile uint32_t s_ticks;

void SysTick_Handler(void) {
    s_ticks++;
}



/* ---------------------------------------------------- */
/* --------------- MAIN ---------------------------- */
/* ---------------------------------------------------- */


int main(void) {
    uint16_t led = PIN('B', 15);
    gpio_set_mode(led, GPIO_MODE_OUTPUT);
    systick_init(4000000 / 1000);

    uint32_t timer = 0, period = 500;    // Declare timer and 500ms period

    uart_init(LPUART1, 115200);

    for (;;) {
        if (timer_expired(&timer, period, s_ticks)) {
            static bool on = false;
            gpio_write(led, on);
            on = !on;

            printf("LED: %d, tick: %lu\r\n", on, (unsigned long) s_ticks);
            // uart_write_buf(LPUART1, "hi\r\n", 4);
        }
    };

    return 0;
}




