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
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>
#include <stdbool.h>
#include <string.h>
#include "systick.h"
#include "basepri.h"
//TODO: remove when done.
#include "usart.h"
#include <stdio.h>

/* If true, the I2C peripheral is enabled. */
static volatile bool I2CEnabled;

/* Conveyor. */
static volatile i2c_ticket_t Conveyor[I2C_CONVEYOR_SIZE];

/* Current ticket. This variable stores the index of the current ticket.
 * If it is negative, no tickets are currently being processed. */
static volatile int8_t CurrentTicket = -1;

/* Number of tickets on conveyor. This includes the ticket currently being
 * processed; when NumTickets == I2C_CONVEYOR_SIZE the conveyor is full. */
static volatile int8_t NumTickets = 0;

/* Evaluates to the index of the ticket after the given one. This handles
 * the circular wrap-around at the end of the conveyor. */
#define next_ticket(ticket) ((ticket + 1) % I2C_CONVEYOR_SIZE)

/* Evaluates to the index of the next free slot in the conveyor. */
#define open_ticket() (( ((-1 == CurrentTicket) ? 0:CurrentTicket) + NumTickets) % I2C_CONVEYOR_SIZE)

/*
 *
 *   -Use STOP interrupt for end of transmission cleanup and next ticket start.
 *    (STOP is instantaneous.)
 *   -Use TXIS done interrupt for end of register send.
 *   -No DMA interrupts.
 */

/* Start a DMA transfer from the I2C peripheral to memory.
 * This function handles configuration of the DMA and I2C peripherals
 * for the transfer. */
static void dma_read(uint8_t *data, uint8_t size){
	dma_disable_channel(DMA1, DMA_CHANNEL7);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL7, (uint32_t) &(I2C_RXDR(I2C1)));
	dma_set_memory_address(DMA1, DMA_CHANNEL7, (uint32_t) data);
	dma_set_number_of_data(DMA1, DMA_CHANNEL7, size);
	dma_set_priority(DMA1, DMA_CHANNEL7, DMA_CCR_PL_MEDIUM);
	dma_set_memory_size(DMA1, DMA_CHANNEL7, DMA_CCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL7, DMA_CCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL7);
	dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL7);
	dma_set_read_from_peripheral(DMA1, DMA_CHANNEL7);
	dma_enable_channel(DMA1, DMA_CHANNEL7);
	
	i2c_enable_rxdma(I2C1);
}

/* Start a DMA transfer from memory to the I2C peripheral.
 * This function handles configuration of the DMA and I2C peripherals
 * for the transfer. */
static void dma_write(uint8_t *data, uint8_t size){
	dma_disable_channel(DMA1, DMA_CHANNEL6);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL6,  (uint32_t) &(I2C_TXDR(I2C1)));
	dma_set_memory_address(DMA1, DMA_CHANNEL6, (uint32_t) data);
	dma_set_number_of_data(DMA1, DMA_CHANNEL6, size);
	dma_set_priority(DMA1, DMA_CHANNEL6, DMA_CCR_PL_MEDIUM);
	dma_set_memory_size(DMA1, DMA_CHANNEL6, DMA_CCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL6, DMA_CCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL6);
	dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL6);
	dma_set_read_from_memory(DMA1, DMA_CHANNEL6);
	dma_enable_channel(DMA1, DMA_CHANNEL6);

	i2c_enable_txdma(I2C1);
}

/* Start an i2c transaction with the current ticket.
 * CurrentTicket and NumTickets must be correctly set before calling this
 * function.
 * The function does not check if the I2C peripheral is busy, and should
 * only be called when the bus is idle. */
static void start_conveyor(void){
	/* No busyness check is performed, as we can't block here. */
	if(I2C_WRITE == Conveyor[CurrentTicket].rw){
		//TODO
		iprintf("Writing I2C.\r\n");
		write_i2c(I2C1, Conveyor[CurrentTicket].addr,
		          Conveyor[CurrentTicket].reg,
		          Conveyor[CurrentTicket].size, Conveyor[CurrentTicket].data);
		//TODO:
		iprintf("Setting flag. CurrentTicket=%d NumTickets=%d\n\r", CurrentTicket, NumTickets);
		if(CurrentTicket >= 0){
			if(NULL != Conveyor[CurrentTicket].done_flag){
				*Conveyor[CurrentTicket].done_flag = 1;
			}
			--NumTickets;
			if(0 == NumTickets){
				CurrentTicket = -1;
			} else {
				CurrentTicket = next_ticket(CurrentTicket);
			}
		}
	} else {
		//TODO
		iprintf("Reading I2C.\r\n");
		int wait;
		int i;
		int size = Conveyor[CurrentTicket].size;
		while (i2c_busy(I2C1) == 1);
		while (i2c_is_start(I2C1) == 1);
		/*Setting transfer properties*/
		i2c_set_bytes_to_transfer(I2C1, 1);
		i2c_set_7bit_address(I2C1, Conveyor[CurrentTicket].addr);
		i2c_set_write_transfer_dir(I2C1);
		i2c_disable_autoend(I2C1);
		/*start transfer*/
		i2c_send_start(I2C1);
		
		wait = true;
		while (wait) {
			if (i2c_transmit_int_status(I2C1)) {
				wait = false;
			}
			while (i2c_nack(I2C1)); /* Some error */
		}
		i2c_send_data(I2C1, Conveyor[CurrentTicket].reg);
		
		while (i2c_is_start(I2C1) == 1);
		/*Setting transfer properties*/
		i2c_set_bytes_to_transfer(I2C1, size);
		i2c_set_7bit_address(I2C1, Conveyor[CurrentTicket].addr);
		i2c_set_read_transfer_dir(I2C1);
		i2c_enable_autoend(I2C1);
		/*start transfer*/
		i2c_send_start(I2C1);
		
		for (i = 0; i < size; i++) {
			while (i2c_received_data(I2C1) == 0);
			Conveyor[CurrentTicket].data[i] = i2c_get_data(I2C1);
		}

		//TODO:
		iprintf("Setting flag. CurrentTicket=%d NumTickets=%d\n\r", CurrentTicket, NumTickets);
		if(CurrentTicket >= 0){
			if(NULL != Conveyor[CurrentTicket].done_flag){
				*Conveyor[CurrentTicket].done_flag = 1;
			}
			--NumTickets;
			if(0 == NumTickets){
				CurrentTicket = -1;
			} else {
				CurrentTicket = next_ticket(CurrentTicket);
			}
		}
	}
}

/* State of current ticket. This variable is mainly used to tell
 * when the 2nd address has been sent when reading from a device,
 * but the extra states are provided for completeness. */
static volatile enum {SENT_ADDRESS,     /* The address has been sent. */
                      SENT_REGISTER,    /* The register has been sent.
                                         * A data write or repeated start for
				         * reading can now occur. */
		      SENT_2ND_ADDRESS, /* The repeated start and address have
		                         * been sent. A data read can now occur. */
		      TRANSFER_DONE     /* The transfer is complete. */
} TicketState = TRANSFER_DONE;

static void i2c_error_cleanup(void){
	/* On error: set the error flag on the current ticket, reset
	 * the I2C peripheral, and proceed to the next ticket if there is one. */
	init_i2c();
	dma_disable_channel(DMA1, DMA_CHANNEL6);
	dma_disable_channel(DMA1, DMA_CHANNEL7);
	TicketState = TRANSFER_DONE;
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

/* I2C event interrupt. Handles STOP, TXIS, and NACK events. */
void i2c1_ev_exti23_isr(){
	if(I2C1_ISR & I2C_ISR_NACKF){ /* NACK error */
		I2C1_ICR |= I2C_ICR_NACKCF; /* Clear the NACK flag so the ISR will exit. */
		i2c_error_cleanup();
	} else if(I2C1_ISR & I2C_ISR_TXIS){
		/* TXIS event - time to transmit register address
		 * or do a repeated start for reading. */
	} else if(I2C1_ISR & I2C_ISR_STOPF){
		//TODO
		iprintf("Stop condition interrupt CurrentTicket=%d  NumTickets=%d.\r\n", CurrentTicket, NumTickets);
		/* Stop condition detected, indicating that the transfer is
		 * complete. Wrap up the current ticket and start on the next
		 * one. */
		I2C1_ICR |= I2C_ICR_STOPCF;
// 		TicketState = TRANSFER_DONE;
// 		if(CurrentTicket >= 0){
// 			if(NULL != Conveyor[CurrentTicket].done_flag){
// 				*Conveyor[CurrentTicket].done_flag = 1;
// 			}
// 			--NumTickets;
// 			if(0 == NumTickets){
// 				CurrentTicket = -1;
// 			} else {
// 				CurrentTicket = next_ticket(CurrentTicket);
// 			}
// 		}
	}
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
	uint32_t save_basepri = get_basepri();
	set_basepri(I2C_IRQ_PRIORITY);
	if(I2C_CONVEYOR_SIZE == NumTickets){
		set_basepri(save_basepri);
		return -2; /* Conveyor is full. */
	}
	int ticket_index = open_ticket();
	memmove((i2c_ticket_t *) Conveyor + ticket_index, ticket, sizeof(i2c_ticket_t));
	NumTickets++;
	if(-1 == CurrentTicket){
		CurrentTicket = ticket_index;
		start_conveyor();
	}
	set_basepri(save_basepri);
	return 0;
}


/* Setup the I2C1 peripheral. */
void init_i2c(void){
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_set_i2c_clock_hsi(I2C1);
	
	i2c_reset(I2C1);
	i2c_disable_txdma(I2C1);
	i2c_disable_rxdma(I2C1);
	i2c_disable_autoend(I2C1);
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
	i2c_enable_interrupt(I2C1, I2C_CR1_STOPIE);
	i2c_enable_interrupt(I2C1, I2C_CR1_ERRIE);

	I2CEnabled = true;
}

/* Shutdown the I2C1 peripheral to save power. */
void shutdown_i2c(void){
	nvic_disable_irq(NVIC_I2C1_EV_EXTI23_IRQ);
	i2c_peripheral_disable(I2C1);
	rcc_periph_clock_disable(RCC_I2C1);
	I2CEnabled = false;
}