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
 * Board internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

#define FLASH_BASED_PARAMS

/* The board has a single FMU status LED (active-low) on PA4. */

#define GPIO_nLED_BLUE          /* PA4 nFMU_LED */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

#define BOARD_HAS_CONTROL_STATUS_LEDS   1
/*
 * Only one LED on this board, so it is used for the arming status:
 *   not-ready blink @10Hz, ready-to-arm blink @1Hz, armed solid, failsafe blink.
 * Both arming macros point at the same (only) LED. No BOARD_OVERLOAD_LED is
 * defined, otherwise the CPU-overload indicator would force the LED off every
 * cycle and stomp the arming blink.
 */
#define BOARD_ARMED_LED                 0 // nFMU_LED (PA4): solid ON when armed
#define BOARD_ARMED_STATE_LED           0 // nFMU_LED (PA4): arming-state blink

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */
#define ADC1_CH(n)                  (n)

/* Single power input (J4 Power_A). MAIN_V_SENSE is selected by JP1 between
 * the external power-module voltage pin and the onboard 18k/1k divider on
 * MAIN_POWER; MAIN_C_SENSE comes from the external power module only.
 *
 * Not used, per the schematic:
 *   PC2_C/PC3_C (AUX_V_SENSE/AUX_C_SENSE) are not routed off the FCU sheet,
 *   so there is no second battery input.
 *   PC4 (BOARD_5V_SENSE): divider R18/R19 is DNP, so the pin only sees C63.
 *   PC5 (BOARD_3.3V_SENSE) is tied straight to the 3.3V rail, which is also
 *   the ADC reference (VDDA), so it always reads full scale.
 */
#define BOARD_NUMBER_BRICKS               1
#define BOARD_ADC_BRICK_VALID             1

/* Define GPIO pins used as ADC N.B. Channel numbers must match below  */
#define PX4_ADC_GPIO  \
	/* PC0 MAIN_V_SENSE */  GPIO_ADC123_INP10, \
	/* PC1 MAIN_C_SENSE */  GPIO_ADC123_INP11

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_BATTERY_VOLTAGE_CHANNEL      	/* PC0  */  ADC1_CH(10)
#define ADC_BATTERY_CURRENT_CHANNEL      	/* PC1  */  ADC1_CH(11)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL)	| \
	(1 << ADC_BATTERY_CURRENT_CHANNEL))

/* Define Battery 1 Voltage Divider and A per V */

#define BOARD_BATTERY1_V_DIV         (19.1f)     /* measured with the provided PM board (onboard divider is 18k/1k = 19.0) */
#define BOARD_BATTERY1_A_PER_V       (0.0f)      /* current sense comes from the external power module only */

/* USB VBUS
 *
 * VBUS_CONN is not routed to the MCU (R20 DNP) and PA9 is the HEATER output,
 * so there is no VBUS sensing on this board. The OTG core runs with VBUS
 * detection disabled (CONFIG_USBDEV_VBUSSENSING not set). Report USB as always
 * connected rather than always disconnected: cdcacm_autostart (see
 * CONFIG_DRIVERS_CDCACM_AUTOSTART) gates bringing up the USB CDC-ACM
 * console/MAVLink link on board_read_VBUS_state(), so reporting "never
 * connected" would silently prevent the USB link from ever starting.
 */
#define BOARD_ADC_USB_CONNECTED 1

/* The bootloader (which shares this header) still references GPIO_OTGFS_VBUS
 * in board_deinit(), so it must resolve to a harmless pin. PB12 carries the
 * SPI4_DRDY net, which is not routed off the FCU sheet.
 */
#define BOARD_USB_VBUS_SENSE_DISABLED
#define GPIO_OTGFS_VBUS		/* PB12 (unrouted) */ (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_50MHz|GPIO_PORTB|GPIO_PIN12)

/* CAN
 *
 * Both CAN ports use TCAN332G transceivers, which have no silent/standby
 * control pin, so there are no CAN silent GPIOs.
 */

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   16
#define DIRECT_INPUT_TIMER_CHANNELS  6

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

/* Spare GPIO
 *
 * PB12 (SPI4_DRDY net) and PB2 (SPI4_CS net) are not routed off the FCU sheet.
 * PC13 (BMP388_CS) and PE12 (SPI6_CS2) only go to the DNP J15 isolated-sensors
 * header.
 */

/* I2C2 is external (it is also routed to the GPS2 connector), but it carries
 * the onboard LIS3MDL. px4_i2c_device_external() in i2c.cpp reports that one
 * device as internal so it gets the board rotation and internal priority.
 */
#define BOARD_OVERRIDE_I2C_DEVICE_EXTERNAL

/* Tone‑alarm output on PB9 with TIM17_CH1 ---------------------------------- */
#define TONE_ALARM_TIMER             17      /* use TIM17               */
#define TONE_ALARM_CHANNEL           1       /* channel 1               */

/* Idle (GPIO) state: low, push‑pull, 2 MHz */
#define GPIO_TONE_ALARM_IDLE \
        (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|\
         GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN9)

#define GPIO_TONE_ALARM \
        (GPIO_ALT|GPIO_AF1|GPIO_SPEED_2MHz|GPIO_PUSHPULL|\
         GPIO_PORTB|GPIO_PIN9)

/* HEATER (simple on/off GPIO) ---------------------------------------------
 * PA9 drives the gate of Q2 (low-side N-FET switching R27, 25R from 5V).
 * Active-high; held low from boot via PX4_GPIO_INIT_LIST because the gate
 * has no pull-down.
 */
#define GPIO_HEATER_OUTPUT   /* PA9 */ \
        (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|\
         GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN9)

#define HEATER_OUTPUT_EN(on_true)  px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, (on_true))

/* High-resolution timer */
/* High‑resolution timer (HRT) --------------------------------------------- */
#define HRT_TIMER               2   /* use TIM2 for the HRT               */
#define HRT_TIMER_CHANNEL       3   /* use capture/compare channel 3      */

/* PPM input on TIM2_CH1 (PA15) -------------------------------------------- */
#define HRT_PPM_CHANNEL		1
#define GPIO_PPM_IN             (GPIO_TIM2_CH1IN_2|GPIO_SPEED_50MHz)

/* Safety button -----------------------------------------------------------
 * J16 is a 2-pin switch between SAFETY_SWITCH_BTN (PC15, via 220R ESD) and
 * GND, with no external pull-up: use the MCU pull-up, pressed = low.
 *
 * The stock safety_button driver assumes an active-high button and an
 * active-low LED, so this board builds its own copy (src/safety_button) that
 * honours BOARD_SAFETY_BUTTON_ACTIVE_LOW / BOARD_SAFETY_LED_ACTIVE_HIGH.
 * Keep CONFIG_DRIVERS_SAFETY_BUTTON disabled in default.px4board.
 */
#define GPIO_BTN_SAFETY         /* PC15 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN15)
#define BOARD_SAFETY_BUTTON_ACTIVE_LOW

/* Safety LED ---------------------------------------------------------------
 * D9 (red, on the board - J16 has no LED pin) is wired PB1 -> LED -> R70 330R
 * -> GND, so it is active-high even though the net is named
 * nSAFETY_SWITCH_LED. Starts off.
 */
#define GPIO_LED_SAFETY         /* PB1 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)
#define BOARD_SAFETY_LED_ACTIVE_HIGH

/* RC input: FMU_PPM_INPUT reaches both PA15 (RC_IN, TIM2_CH1 PPM capture,
 * via R28 0R) and PA10 (USART1_RX, via R39 0R). USART1 is RX-only - no TX
 * telemetry (CRSF/GHST) back to the receiver is possible on this port.
 */
#define RC_SERIAL_PORT          "/dev/ttyS0"

/* microSD is wired in SPI mode on SPI4 (CS PE3, no card detect); see init.c
 * and CONFIG_NSH_MMCSDSPIPORTNO. SDMMC is not used.
 */

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO, \
		GPIO_CAN1_TX, \
		GPIO_CAN1_RX, \
		GPIO_CAN2_RX, \
		GPIO_CAN2_TX, \
		GPIO_nLED_BLUE, \
		GPIO_TONE_ALARM_IDLE, \
		GPIO_HEATER_OUTPUT, \
		GPIO_BTN_SAFETY, \
		GPIO_LED_SAFETY, \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 4

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/


/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
