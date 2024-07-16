// end of stack, provided by linker
extern void _estack(void);

extern int main(void);

// reset handler
__attribute__((naked, noreturn)) void _reset(void) {
	extern unsigned int _data_start, _data_end, _data_loadaddr, _bss_start, _bss_end;

    for (unsigned int *bss_p = &_bss_start; bss_p < &_bss_end; bss_p++) {
        *bss_p = 0;
    }

    unsigned int *data_lvm_p = &_data_loadaddr;
    unsigned int *data_sram1_p = &_data_start;
    while (data_sram1_p < &_data_end) {
        *data_sram1_p = *data_lvm_p;
        data_sram1_p++;
        data_lvm_p++;
    }

    main();

    while (1) (void) 0;
}

extern void systick_handler(void);

// vector table. 16 cortex-m4 specific interrupts + 62 stm32wl55 interrupts
__attribute__((section(".vectors"))) void (*tab[16 + 62]) (void) = {
	_estack,
	_reset,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	systick_handler
};
