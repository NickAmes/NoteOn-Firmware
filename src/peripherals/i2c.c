/* I2C Setup and Control Functions
 * This file provides a high-level interface to the I2C1 peripheral.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 project.                              */
#include "i2c.h"
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>
#include <stdbool.h>
#include <string.h>
#include "systick.h"
#include "basepri.h"
//TODO: remove when done.
#include "usart.h"

/* If !0, the I2C peripheral is enabled. */
static volatile bool I2CEnabled;

/* Conveyor. */
static volatile i2c_ticket_t Conveyor[I2C_CONVEYOR_SIZE];

/* Current ticket. This variable stores the index of the current ticket.
 * If it is negative, no tickets are currently being processed. */
static volatile int8_t CurrentTicket = -1;

/* Number of tickets on conveyor. This includes the ticket currently being
 * processed; when NumTickets == I2C_CONVEYOR_SIZE the conveyor is full. */
static volatile int8_t NumTickets;

/* Evaluates to the index of the ticket after the given one. This handles
 * the circular wrap-around at the end of the conveyor. */
#define next_ticket(ticket) ((ticket + 1) % I2C_CONVEYOR_SIZE)

/* Evaluates to the index of the next free slot in the conveyor. */
#define open_ticket() (( ((-1 == CurrentTicket) ? 0:CurrentTicket) + NumTickets) % I2C_CONVEYOR_SIZE)

/* I2C transaction sequence:
 *   -Wait for I2C not busy.
 *   -Set transfer properties, send start, and setup DMA of the address
 *    and register.
 *   -If reading, generate another start condition and re-send the slave address.
 *   -When address and register have been sent (DMA IRQ) setup DMA of
 *    ticket data.
 *   -When DMA is complete, set ticket done flag and move on to the next ticket.
 *   -If an error occurs, stop any transmission, send a stop condition, set the
 *    error flag on the ticket, and move on to the next ticket.
 */

/* Start a DMA transfer from the I2C peripheral to memory. */
static void dma_read(uint8_t *data, uint8_t num_data);

/* Start a DMA transfer from memory to the I2C peripheral. */
static void dma_write(uint8_t *data, uint8_t num_data);

static volatile enum {SENT_ADDRESS,     /* The address has been sent. */
                      SENT_REGISTER,    /* The register has been sent.
                                         * A data write or repeated start for
				         * reading can now occur. */
		      SENT_2ND_ADDRESS, /* The repeated start and address have
		                         * been sent. A data read can now occur. */
		      TRANSFER_DONE     /* The transfer is complete. */
} TicketState;


/* Start an i2c transaction with the current ticket.
 * CurrentTicket and NumTickets must be correctly set before calling this
 * function.
 * The function does not check if the I2C peripheral is busy, and should
 * only be called when the bus is idle. */
static void start_conveyor(void){
	/* No busyness check is performed, as we can't block here. */
	//TODO
	if(I2C_WRITE == Conveyor[CurrentTicket].rw){
		write_i2c(I2C1, Conveyor[CurrentTicket].addr,
		          Conveyor[CurrentTicket].reg,
		          Conveyor[CurrentTicket].size, Conveyor[CurrentTicket].data);
	} else {
		read_i2c(I2C1, Conveyor[CurrentTicket].addr,
		          Conveyor[CurrentTicket].reg,
		          Conveyor[CurrentTicket].size, Conveyor[CurrentTicket].data);
	}
	--NumTickets;
	if(0 == NumTickets){
		CurrentTicket = -1;
	} else {
		CurrentTicket = next_ticket(CurrentTicket);
	}
	
}

static void i2c_error_cleanup(void){
	/* On error: set the error flag on the current ticket, reset
	 * the I2C peripheral, and proceed to the next ticket if there is one. */
	init_i2c();
	//TODO: reset DMA.
	if(CurrentTicket >= 0){
		if(Conveyor[CurrentTicket].done_flag != NULL){
			*(Conveyor[CurrentTicket].done_flag) = I2C_ERROR;
		}
		--NumTickets;
		if(NumTickets > 0){
			CurrentTicket = next_ticket();
			start_conveyor();
		} else {
			CurrentTicket = -1;
		}
	} else {
		/* Looks like we got into an invalid state somehow. */
		NumTickets = 0;
		CurrentTicket = -1;
	}
}

/* NACK error interrupt. Conveyor management is handled through
 * the DMA interrupts. */
void i2c1_ev_exti23_isr(){
	//TODO: remove when done.
	write_str("NACK\r\n");
	I2C1_ICR |= I2C_ICR_NACKCF; /* Clear the NACK flag so the ISR will exit. */
	i2c_error_cleanup();
}


/* I2C error interrupt. */
void i2c1_er_isr(){
	//TODO: remove when done.
	write_str("I2C Error\r\n");
	I2C1_ICR |= 0x00003F00; /* Clear the error flag(s) so the ISR will exit. */
	i2c_error_cleanup();
}

/* Add a ticket to the conveyor. The ticket will be copied (and therefore
 * doesn't need to exist after the function call) but the data will not.
 * This function may be called from an interrupt.
 * Returns:
 *   0 - Success.
 *  -1 - NULL data field or ticket pointer.
 *  -2 - Conveyor is full. Please try again later. */
int add_ticket_i2c(i2c_ticket_t *ticket){
	if(NULL == ticket)return -1;
	if(NULL == ticket->data)return -1;
	if(!I2CEnabled)init_i2c();
	uint32_t save_basepri = __get_BASEPRI();
	__set_BASEPRI(I2C_IRQ_PRIORITY);
	if(I2C_CONVEYOR_SIZE == NumTickets){
		__set_BASEPRI(save_basepri);
		return -2; /* Conveyor is full. */
	}
	uint8_t ticket_index = open_ticket();
	memcpy((i2c_ticket_t *) &Conveyor[ticket_index], ticket, sizeof(i2c_ticket_t));
	NumTickets++;
	if(-1 == CurrentTicket){
		CurrentTicket = open_ticket();
		start_conveyor();
	}
	__set_BASEPRI(save_basepri);
	return 0;
}


/* Setup the I2C1 peripheral. */
void init_i2c(void){
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_set_i2c_clock_hsi(I2C1);
	
	i2c_reset(I2C1);
	/* Setup GPIO pins */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO6 | GPIO7);
	i2c_peripheral_disable(I2C1);
	i2c_enable_analog_filter(I2C1);
	i2c_set_digital_filter(I2C1, I2C_CR1_DNF_DISABLED);
	/* Setup I2C Fast Mode (400MHz) */
	i2c_set_prescaler(I2C1, 0);
	i2c_set_scl_low_period(I2C1, 0x9);
	i2c_set_scl_high_period(I2C1, 0x3);
	i2c_set_data_hold_time(I2C1, 0x1);
	i2c_set_data_setup_time(I2C1, 0x3);

	i2c_peripheral_enable(I2C1);

	nvic_set_priority(NVIC_I2C1_EV_EXTI23_IRQ, I2C_IRQ_PRIORITY << 4);
	nvic_set_priority(NVIC_I2C1_ER_IRQ, I2C_IRQ_PRIORITY << 4);
	nvic_enable_irq(NVIC_I2C1_EV_EXTI23_IRQ);
	nvic_enable_irq(NVIC_I2C1_ER_IRQ);
	i2c_enable_interrupt(I2C1, I2C_CR1_NACKIE);
	i2c_enable_interrupt(I2C1, I2C_CR1_ERRIE);
	//TODO: DMA interrupts and setup

	I2CEnabled = true;
}

/* Shutdown the I2C1 peripheral to save power. */
void shutdown_i2c(void){
	nvic_disable_irq(NVIC_I2C1_EV_EXTI23_IRQ);
	i2c_peripheral_disable(I2C1);
	rcc_periph_clock_disable(RCC_I2C1);
	I2CEnabled = false;
}