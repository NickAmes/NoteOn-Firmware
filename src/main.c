#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

int main(void){
	int i;
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
	
	while (1) {
		/* Toggle LED. */
		gpio_toggle(GPIOA, GPIO13);
		for (i = 0; i < 3000000; i++) /* Wait a bit. */
			__asm__("nop");
	}
	
}