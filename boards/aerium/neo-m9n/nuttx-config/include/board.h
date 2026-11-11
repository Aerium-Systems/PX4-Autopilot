/************************************************************************************
 * configs/px4fmu/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ************************************************************************************/
#include "board_dma_map.h"

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include <stm32.h>

/* HSI - 8 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - 8  MHz Crystal
 * LSE - not installed
 */
#define STM32_BOARD_USEHSE      1
#define STM32_BOARD_XTAL        8000000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000

/* Main PLL Configuration */
#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(384)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_4
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(8)
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(2)

#define STM32_RCC_PLLI2SCFGR_PLLI2SM RCC_PLLI2SCFGR_PLLI2SM(16)
#define STM32_RCC_PLLI2SCFGR_PLLI2SN RCC_PLLI2SCFGR_PLLI2SN(192)
#define STM32_RCC_PLLI2SCFGR_PLLI2SQ RCC_PLLI2SCFGR_PLLI2SQ(2)
#define STM32_RCC_PLLI2SCFGR_PLLI2SR RCC_PLLI2SCFGR_PLLI2SR(2)
#define STM32_RCC_PLLI2SCFGR_PLLI2SSRC RCC_PLLI2SCFGR_PLLI2SSRC(0) /* HSE or HSI depending on PLLSRC of PLLCFGR*/

#define STM32_RCC_DCKCFGR2_CK48MSEL RCC_DCKCFGR2_CK48MSEL_PLL
#define STM32_RCC_DCKCFGR2_FMPI2C1SEL RCC_DCKCFGR2_FMPI2C1SEL_APB
#define STM32_RCC_DCKCFGR2_SDIOSEL RCC_DCKCFGR2_SDIOSEL_48MHZ

#define STM32_SYSCLK_FREQUENCY  96000000ul

/* AHB clock (HCLK) is SYSCLK (96MHz) */
#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY      /* Same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/2 (48MHz) */
#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2     /* PCLK1 = HCLK / 2 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB1 will be twice PCLK1 (see page 112 of reference manual) */
#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK (96MHz) */
#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK       /* PCLK2 = HCLK */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY)

/* Timers driven from APB2 will be PCLK2 since no prescale division */
#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx otherwise frequency is 2xAPBx. */
#define BOARD_TIM2_FREQUENCY    (2 * STM32_PCLK1_FREQUENCY)
#define BOARD_TIM3_FREQUENCY    (2 * STM32_PCLK1_FREQUENCY)
#define BOARD_TIM4_FREQUENCY    (2 * STM32_PCLK1_FREQUENCY)
#define BOARD_TIM5_FREQUENCY    (2 * STM32_PCLK1_FREQUENCY)
#define BOARD_TIM6_FREQUENCY    (2 * STM32_PCLK1_FREQUENCY)
#define BOARD_TIM7_FREQUENCY    (2 * STM32_PCLK1_FREQUENCY)
#define BOARD_TIM8_FREQUENCY    (2 * STM32_PCLK2_FREQUENCY)

/* Alternate function pin selections ************************************************/

/* UARTs */
/* USART1: u-blox NEO-M9N (/dev/ttyS0).
 *   GPS_TXD -> USART1_RX (PB3), USART1_TX (PA15) -> GPS_RXD.
 * USART6: NSH debug console on test pads TP1/TP2 (/dev/ttyS1).
 *   UART6_TX = PA11, UART6_RX = PA12.
 * USART2 (PA2/PA3) is labelled on the schematic but not routed anywhere. */

#define GPIO_USART1_RX	GPIO_USART1_RX_2	/* PB3  */
#define GPIO_USART1_TX	GPIO_USART1_TX_3	/* PA15 */

#define GPIO_USART6_RX	GPIO_USART6_RX_3	/* PA12 */
#define GPIO_USART6_TX	GPIO_USART6_TX_3	/* PA11 */

/* CAN */
/* CAN1 -> TCAN332G transceiver (U5) -> J1. RX = PB8, TX = PB9. */
#define GPIO_CAN1_RX	GPIO_CAN1_RX_2	/* PB8 */
#define GPIO_CAN1_TX	GPIO_CAN1_TX_2	/* PB9 */

/* SPI */
/* SPI1 is not connected on this board (PA4-PA7 are no-connect).
 * SPI2 pins are labelled on the schematic (CS1 = PB12, CS2 = PC13) but no
 * device is populated; configured as an external bus only. */
#define GPIO_SPI2_MISO  GPIO_SPI2_MISO_1 /* PB14 */
#define GPIO_SPI2_MOSI  GPIO_SPI2_MOSI_1 /* PB15 */
#define GPIO_SPI2_SCK   GPIO_SPI2_SCK_2  /* PB13 */

/* I2C */
/* I2C1: LIS3MDL magnetometer (U7, addr 0x1E), 4.7K pull-ups R20/R21.
 * SCL = PB6, SDA = PB7. The NEO-M9N's own I2C pins are not connected. */
#define GPIO_I2C1_SCL	GPIO_I2C1_SCL_1	/* PB6 */
#define GPIO_I2C1_SDA	GPIO_I2C1_SDA_1	/* PB7 */

#endif /* __ARCH_BOARD_BOARD_H */
