/* High-clock system control functions.
 * 
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 project.                                 */
#include "clock.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>

/* WARNING: Some timer setup routines were written with the assumption
 * that the ABP1/ABP2 division factors are either 1 or 2. Don't choose different
 * factors without checking all timer setup routines. */

/* Default system clock is 8Mhz. */
volatile uint32_t SystemClock = 8000000;

/* Set the system clock for 72Mhz derived from an external 16Mhz crystal.
 * The APB high-speed clock will be set to 72Mhz, the APB low-speed clock
 *  to 36Mhz, and the USB clock to 48Mhz. */
void clock_72MHz_hse(void){
	rcc_osc_on(HSE);
	rcc_wait_for_osc_ready(HSE);

	rcc_osc_off(PLL);
	rcc_wait_for_osc_not_ready(PLL);
	rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_PREDIV);
	rcc_set_main_pll_hsi(RCC_CFGR_PLLMUL_PLL_IN_CLK_X9);
	rcc_usb_prescale_1_5();
	RCC_CFGR |= RCC_CFGR_PLLXTPRE; /* Divide HSE by 2 before PLL. */
	rcc_set_ppre2(RCC_CFGR_PPRE2_DIV_NONE);
	rcc_set_ppre1(RCC_CFGR_PPRE1_DIV_2);
	rcc_set_hpre(RCC_CFGR_HPRE_DIV_NONE);
	rcc_osc_on(PLL);
	rcc_wait_for_osc_ready(PLL);

	flash_set_ws(FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2WS);
	rcc_set_sysclk_source(RCC_CFGR_SW_PLL);
	rcc_wait_for_sysclk_status(PLL);

	rcc_ppre1_frequency = 36000000;
	rcc_ppre2_frequency = 72000000;
	SystemClock = 72000000;
}

/* Set the system clock for 64Mhz derived from the internal 8MHz oscillator.
 * The APB high-speed clock will be set to 64Mhz and the APB low-speed clock
 * to 32Mhz. */
void clock_64MHz_hsi(void){
	clock_scale_t clock =
	{
	.pll = RCC_CFGR_PLLMUL_PLL_IN_CLK_X16,
	.pllsrc = RCC_CFGR_PLLSRC_HSI_DIV2,
	.hpre = RCC_CFGR_HPRE_DIV_NONE,
	.ppre1 = RCC_CFGR_PPRE1_DIV_2,
	.ppre2 = RCC_CFGR_PPRE2_DIV_NONE,
	.power_save = 1,
	.flash_config = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2WS,
	.apb1_frequency = 32000000,
	.apb2_frequency = 64000000,
	};
	rcc_clock_setup_hsi(&clock);
	SystemClock = 64000000;
}