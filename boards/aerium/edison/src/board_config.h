/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * board internal definitions
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/*
 * Aerium Edison - DroneCAN power board (Edison.kicad_sch, sheet /CAN_MCU/).
 *
 * MCU U4: STM32F412CGUx (UFQFPN48), 8 MHz HSE crystal Y1 on PH0/PH1.
 * CAN1 (PB8 RX / PB9 TX) -> TCAN332G (U7) -> J5 (HOST_5V, CAN_P, CAN_N, GND).
 * Debug console USART6 on PA11 (TX, TP1) / PA12 (RX, TP2). SWD on PA13/PA14.
 * No PWM outputs, SPI devices or on-board sensor ICs. I2C1 (PB6/PB7) is only
 * net-labelled on the schematic.
 */

/* LM5143 dual buck (U2) enables EN1/EN2. Push-pull, high = rail on.
 * Initialised high so the rails stay on until edison_power applies EDSN_EN_A/B. */
#define GPIO_EN_VCC_A         /* PB12 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN12)
#define GPIO_EN_VCC_B         /* PB14 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN14)

/* LM5143 power-good PG1/PG2 (open drain, R15/R14 100K pull-ups to 3.3V). High = rail in regulation. */
#define GPIO_PG_VCC_A         /* PB13 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTB|GPIO_PIN13)
#define GPIO_PG_VCC_B         /* PB15 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTB|GPIO_PIN15)

/* HOST_5V from CAN connector J5 pin 1 (R29 1K series, R36 100K pull-down). High = host 5V present. */
#define GPIO_HOST_5V_VALID    /* PA8  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTA|GPIO_PIN8)

/* ADC1 channels */
#define ADC_TEMP2_CHANNEL               3  /* PA3: TEMP2, NTC TH1 to AGND, R30 12K pull-up to 3.3V */
#define ADC_TEMP1_CHANNEL               4  /* PA4: TEMP1, NTC TH2 to AGND, R45 12K pull-up to 3.3V */
#define ADC_VCC_A_SENSE_CHANNEL         5  /* PA5: VCCA_SENSE, VCC_A via R46 180K / R49 10K */
#define ADC_VCC_B_SENSE_CHANNEL         6  /* PA6: VCCB_SENSE, VCC_B via R50 180K / R51 10K */
#define ADC_BATTERY_VOLTAGE_CHANNEL     8  /* PB0: V_SENSE, input (IP-) via R2 180K / R5 10K */
#define ADC_BATTERY_CURRENT_CHANNEL     9  /* PB1: C_SENSE, INA241A3 (50 V/V) across R20 (0.2 mOhm) */

#define ADC_CHANNELS \
	((1 << ADC_TEMP2_CHANNEL)           | \
	 (1 << ADC_TEMP1_CHANNEL)           | \
	 (1 << ADC_VCC_A_SENSE_CHANNEL)     | \
	 (1 << ADC_VCC_B_SENSE_CHANNEL)     | \
	 (1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL))

#define PX4_ADC_GPIO  \
	/* PA3 */  GPIO_ADC1_IN3,  \
	/* PA4 */  GPIO_ADC1_IN4,  \
	/* PA5 */  GPIO_ADC1_IN5,  \
	/* PA6 */  GPIO_ADC1_IN6,  \
	/* PB0 */  GPIO_ADC1_IN8,  \
	/* PB1 */  GPIO_ADC1_IN9

#define GPIO_USART6_RX_GPIO   /* PA12 */  (GPIO_INPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_PORTA|GPIO_PIN12)
#define GPIO_USART6_TX_GPIO   /* PA11 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_PORTA|GPIO_PIN11)

#define FLASH_BASED_PARAMS

/* High-resolution timer */
#define HRT_TIMER                    8  /* use timer 8 for the HRT */
#define HRT_TIMER_CHANNEL            1  /* use capture/compare channel 1 */

#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_EN_VCC_A,                    \
		GPIO_EN_VCC_B,                    \
		GPIO_PG_VCC_A,                    \
		GPIO_PG_VCC_B,                    \
		GPIO_HOST_5V_VALID,               \
	}

__BEGIN_DECLS

#define BOARD_HAS_N_S_RGB_LED       1
#define BOARD_MAX_LEDS              BOARD_HAS_N_S_RGB_LED

#ifndef __ASSEMBLY__

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
