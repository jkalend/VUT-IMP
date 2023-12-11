/*
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    MKL27Z644_Project.c
 * @brief   Application entry point.
 */

// IMP project 2023
// Jan Kalenda xkalen07
// keeping the NPX copyright as required by the clause
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL27Z644.h"
#include "fsl_debug_console.h"
#include "fsl_lpuart.h"
#include "fsl_tpm.h"
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_pit.h"


#define SIDELIGHTS(ORDER) do { \
	GPIO_PortToggle(BOARD_S##ORDER##_G2_GPIO, BOARD_S##ORDER##_G2_GPIO_PIN_MASK); \
} while (0)



#define SELECT_OPTION(PIN, ORDER) do { \
    switch (PIN) { \
        case 0: \
            GPIO_PortClear(BOARD_##PIN##_G1_GPIO, BOARD_##PIN##_G1_GPIO_PIN_MASK); \
			GPIO_PortSet(BOARD_##PIN##_R_GPIO, BOARD_##PIN##_R_GPIO_PIN_MASK); \
			SIDELIGHTS(ORDER); \
			PIN = 5; \
            break; \
        case 1: \
            GPIO_PortClear(BOARD_##PIN##_O_GPIO, BOARD_##PIN##_O_GPIO_PIN_MASK); \
			GPIO_PortSet(BOARD_##PIN##_G1_GPIO, BOARD_##PIN##_G1_GPIO_PIN_MASK); \
			SIDELIGHTS(ORDER); \
			PIN--; \
            break; \
        case 2: \
            GPIO_PortClear(BOARD_##PIN##_R_GPIO, BOARD_##PIN##_R_GPIO_PIN_MASK); \
			GPIO_PortSet(BOARD_##PIN##_O_GPIO, BOARD_##PIN##_O_GPIO_PIN_MASK); \
			PIN--; \
            break; \
        default: \
			PIN--; \
            break; \
    } \
} while (0)

#define ACTIVATE_LIGHT(PIN) do { \
    GPIO_PortToggle(BOARD_##PIN##_PG_GPIO, BOARD_##PIN##_PG_GPIO_PIN_MASK); \
	GPIO_PortToggle(BOARD_##PIN##_PR_GPIO, BOARD_##PIN##_PR_GPIO_PIN_MASK); \
} while (0)


volatile int S1 = 2;
volatile int S2 = 4;
volatile int S3 = 6;
volatile int option = 1;
volatile bool side1 = false;
volatile bool side2 = false;
volatile bool side3 = false;


void initPins()
{
	gpio_pin_config_t led_config_off = {
		kGPIO_DigitalOutput,
		0,
	};
	gpio_pin_config_t led_config_on = {
		kGPIO_DigitalOutput,
		1,
	};
	gpio_pin_config_t button_config = {
		kGPIO_DigitalInput,
		1,
	};

	GPIO_PinInit(BOARD_S2_PS_GPIO, BOARD_S2_PS_PIN, &button_config); //S2_PS
	GPIO_PinInit(BOARD_S2_PG_GPIO, BOARD_S2_PG_PIN, &led_config_off); //S2_PG
	GPIO_PinInit(BOARD_S2_PR_GPIO, BOARD_S2_PR_PIN, &led_config_on); //S2_PR

	GPIO_PinInit(BOARD_S1_PS_GPIO, BOARD_S1_PS_PIN, &button_config); //S1_PS
	GPIO_PinInit(BOARD_S1_PG_GPIO, BOARD_S1_PG_PIN, &led_config_off); //S1_PG
	GPIO_PinInit(BOARD_S1_PR_GPIO, BOARD_S1_PR_PIN, &led_config_on); //S1_PR

	GPIO_PinInit(BOARD_S3_PS_GPIO, BOARD_S3_PS_PIN, &button_config); //S3_PS
	GPIO_PinInit(BOARD_S3_PG_GPIO, BOARD_S3_PG_PIN, &led_config_off); //S3_PG
	GPIO_PinInit(BOARD_S3_PR_GPIO, BOARD_S3_PR_PIN, &led_config_on); //S3_PR


	GPIO_PinInit(BOARD_S3_G1_GPIO, BOARD_S3_G1_PIN, &led_config_off);
	GPIO_PinInit(BOARD_S3_G2_GPIO, BOARD_S3_G2_PIN, &led_config_off);
	GPIO_PinInit(BOARD_S3_O_GPIO, BOARD_S3_O_PIN, &led_config_off);
	GPIO_PinInit(BOARD_S3_R_GPIO, BOARD_S3_R_PIN, &led_config_on);

	GPIO_PinInit(BOARD_S2_G1_GPIO, BOARD_S2_G1_PIN, &led_config_off);
	GPIO_PinInit(BOARD_S2_G2_GPIO, BOARD_S2_G2_PIN, &led_config_off);
	GPIO_PinInit(BOARD_S2_O_GPIO, BOARD_S2_O_PIN, &led_config_off);
	GPIO_PinInit(BOARD_S2_R_GPIO, BOARD_S2_R_PIN, &led_config_on);

	GPIO_PinInit(BOARD_S1_G1_GPIO, BOARD_S1_G1_PIN, &led_config_off);
	GPIO_PinInit(BOARD_S1_G2_GPIO, BOARD_S1_G2_PIN, &led_config_off);
	GPIO_PinInit(BOARD_S1_O_GPIO, BOARD_S1_O_PIN, &led_config_off);
	GPIO_PinInit(BOARD_S1_R_GPIO, BOARD_S1_R_PIN, &led_config_on);
}


int main(void) {
	pit_config_t pitConfig;

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    PIT_GetDefaultConfig(&pitConfig);

	/* Init pit module */
	PIT_Init(PIT, &pitConfig);

	/* Set timer period for channel 0 */
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, MSEC_TO_COUNT(3000U, CLOCK_GetFreq(kCLOCK_BusClk)));

	/* Enable timer interrupts for channel 0 */
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

	/* Enable at the NVIC */
	EnableIRQ(PIT_IRQn);

	/* Start channel 0 */
	PIT_StartTimer(PIT, kPIT_Chnl_0);
/////////////////////////////////////


    PORT_SetPinInterruptConfig(BOARD_S3_PS_PORT, BOARD_S3_PS_PIN, kPORT_InterruptRisingEdge);
    PORT_SetPinInterruptConfig(BOARD_S2_PS_PORT, BOARD_S2_PS_PIN, kPORT_InterruptRisingEdge);
    PORT_SetPinInterruptConfig(BOARD_S1_PS_PORT, BOARD_S1_PS_PIN, kPORT_InterruptRisingEdge);

	EnableIRQ(PORTB_PORTC_PORTD_PORTE_IRQn);

    initPins();



    while(1) {
    	 __WFI();
    }
    return 0 ;
}

void PIT_IRQHandler(void)
{
	/* Clear interrupt flag.*/
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
	if (option == 1) { //ideal for handling side3
		S1 = 2;
		SELECT_OPTION(S1, 1);
		option++;
	} else if (option == 2){
		SELECT_OPTION(S1, 1);
		if (side3) {
			ACTIVATE_LIGHT(S3);
			SIDELIGHTS(1);
		}
		SIDELIGHTS(2);
		option++;
	} else if (option == 3) {
		S2 = 3;
		SELECT_OPTION(S1, 1);
		if (side3) {
			ACTIVATE_LIGHT(S3);
			SIDELIGHTS(1);
			side3 = false;
		}
		SIDELIGHTS(2);
		SELECT_OPTION(S2, 2);
		option++;
	} else if (option == 4) { //ideal for handling side1
		SELECT_OPTION(S2, 2);
		option++;
	} else if (option == 5) {
		SELECT_OPTION(S2, 2);
		if (side1) {
			ACTIVATE_LIGHT(S1);
			SIDELIGHTS(2);
		}
		SIDELIGHTS(3);
		option++;
	} else if (option == 6) {
		if (side1) {
			ACTIVATE_LIGHT(S1);
			SIDELIGHTS(2);
			side1 = false;
		}
		S3 = 3;
		SELECT_OPTION(S2, 2);
		SIDELIGHTS(3);
		SELECT_OPTION(S3, 3);
		option++;
	} else if (option == 7) { //ideal for handling side2
		SELECT_OPTION(S3, 3);
		option++;
	} else if (option == 8) {
		SELECT_OPTION(S3, 3);
		if (side2) {
			ACTIVATE_LIGHT(S2);
			SIDELIGHTS(3);
		}
		SIDELIGHTS(1);
		option++;
	} else {
		SELECT_OPTION(S3, 3);
		if (side2) {
			ACTIVATE_LIGHT(S2);
			SIDELIGHTS(3);
			side2 = false;
		}
		SIDELIGHTS(1);
		option = 1;
	}
	__DSB();
}

void PORTB_PORTC_PORTD_PORTE_IRQHandler(void)
{
	volatile int i = 0;
	while(i < 30000) {
		__asm("NOP");
		i++;
	}
	if (PORTE->ISFR & BOARD_S3_PS_PIN_MASK) {
			side3 = true;
	    }
	if (PORTE->ISFR & BOARD_S2_PS_PIN_MASK) {
			side2 = true;
		}
	if (PORTE->ISFR & BOARD_S1_PS_PIN_MASK) {
			side1 = true;
		}
	//taken from SDK example for GPIO interrupts
#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
    /* Clear external interrupt flag. */
    GPIO_GpioClearInterruptFlags(BOARD_S3_PS_GPIO, BOARD_S3_PS_PIN_MASK);
#else
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_S3_PS_GPIO, BOARD_S3_PS_PIN_MASK);
    GPIO_PortClearInterruptFlags(BOARD_S2_PS_GPIO, BOARD_S2_PS_PIN_MASK);
    GPIO_PortClearInterruptFlags(BOARD_S1_PS_GPIO, BOARD_S1_PS_PIN_MASK);
#endif
    SDK_ISR_EXIT_BARRIER;
}
