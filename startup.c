
/* ---------------------------------------------------- */
/* --------------- STARTUP ---------------------------- */
/* ---------------------------------------------------- */


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

    extern int main(void);
    main();

    for(;;) (void) 0;
}

extern void SysTick_Handler(void);
extern void _estack(void);

// vector table. 16 cortex-m4 specific interrupts + 62 stm32wl55 interrupts
__attribute__((section(".vectors"))) void (*tab[16 + 62]) (void) = {
	_estack,
	_reset,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	SysTick_Handler,
};
