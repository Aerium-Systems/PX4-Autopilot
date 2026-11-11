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
 * Aerium NEO-M9N CAN GPS - STM32F412CGU (CAN_MCU sheet)
 *
 *   PH0/PH1    8 MHz HSE crystal (Y1)
 *   PB8/PB9    CAN1 RX/TX -> TCAN332G (U5). The TCAN332G has no silent/standby
 *              pin, and bus termination is the JP5/R30 solder jumper (no GPIO),
 *              so there is no GPIO_CAN1_SILENT_S0 and no GPIO_CAN_TERM.
 *   PB3/PA15   USART1 RX/TX  <- NEO-M9N TXD / -> NEO-M9N RXD  (/dev/ttyS0)
 *   PA11/PA12  USART6 TX/RX  debug console, test pads TP1/TP2  (/dev/ttyS1)
 *   PB6/PB7    I2C1 SCL/SDA  LIS3MDL magnetometer (0x1E)
 *   PA13/PA14  SWDIO/SWCLK
 *
 * Not connected to the MCU: NEO-M9N TIMEPULSE (PPS) and EXTINT (they only go to
 * J3/J2 and the blue LED), NEO-M9N RESET_N/SAFEBOOT, and both LEDs (D1 is on PPS,
 * D2 is a 3.3V power LED). The PWMx / SPI1_DRDY / LED_GREEN / UART2 labels on the
 * CAN_MCU sheet are local labels that are not routed off the MCU.
 */

#define GPIO_USART1_RX_GPIO   /* PB3  */  (GPIO_INPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_PORTB|GPIO_PIN3)
#define GPIO_USART1_TX_GPIO   /* PA15 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_PORTA|GPIO_PIN15)

#define GPIO_USART6_RX_GPIO   /* PA12 */  (GPIO_INPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_PORTA|GPIO_PIN12)
#define GPIO_USART6_TX_GPIO   /* PA11 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_PORTA|GPIO_PIN11)

#define FLASH_BASED_PARAMS

/* High-resolution timer */
#define HRT_TIMER                    8  /* use timer 8 for the HRT */
#define HRT_TIMER_CHANNEL            1  /* use capture/compare channel 1 */

#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_GPIO_INIT_LIST { \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
	}

__BEGIN_DECLS

#ifndef __ASSEMBLY__

extern void stm32_spiinitialize(void);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
