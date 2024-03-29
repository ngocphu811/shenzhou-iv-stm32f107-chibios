/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for the STMicroelectronics STM3210C-EVAL evaluation board.
 */

/*
 * Board identifier.
 */
//#define BOARD_ST_STM3210C_EVAL
#define BOARD_SHENZHOU_IV_STM32F107VCT6
#define BOARD_NAME              "ShenZhou IV STM32F107 v1.1 Deveploment Board"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            32768
#define STM32_HSECLK            25000000

#define GPIOD_LED1_STATUS		2
#define GPIOD_LED2_STATUS		3
#define GPIOD_LED3_STATUS		4
#define GPIOD_LED4_STATUS		7


/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F10X_CL

/*
 * Ethernet PHY type.
 */
#define BOARD_PHY_ID            MII_DM9161_ID
#define BOARD_PHY_RMII

/*
 * IO pins assignments.
 * *********************TO BE COMPLETED*********************
 */

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 *
 * The digits have the following meaning:
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 * Please refer to the STM32 Reference Manual for details.
 */

/*
 * Port A setup.
 * Everything input except:
 * PA1  - Normal input      (ETH_RMII_REF_CLK).
 * PA2  - Alternate output  (ETH_RMII_MDIO).
 */
#define VAL_GPIOACRL            0x44444B44      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x444444B4      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFFFFF

/*
 * Port B setup.
 * Everything input except:
 * PB11 - Alternate output  (ETH_RMII_TX_EN).
 * PB12 - Alternate output  (ETH_RMII_TXD0).
 * PB13 - Alternate output  (ETH_RMII_TXD1).
 */
#define VAL_GPIOBCRL            0x44444444      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0x44BBB444      /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFFFFFF

/*
 * Port C setup.
 * Everything input except:
 * PC1  - Alternate output  (ETH_MDC).
 * PC5  - Normal input      (ETH_RMII_MDINT).

 */
#define VAL_GPIOCCRL            0x444444B4      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x44444444      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFFFFF

/*
 * Port D setup.
 * Everything input except:
 * PD02 Led1
 * PD03 Led2
 * PD04 Led3
 * PD07 Led4
 * PD08  - Normal input      (ETH_RMII_CRSDV).
 * PD09  - Normal input      (ETH_RMII_RXD0).
 * PD10  - Normal input      (ETH_RMII_RXD1).
 */
#define VAL_GPIODCRL            0x34B33344      /*  PD7...PD0 */
#define VAL_GPIODCRH            0x44444444      /* PD15...PD8 */
#define VAL_GPIODODR            0xFFFFFFFF

/*
 * Port E setup.
 * Everything input except:
 */
#define VAL_GPIOECRL            0x44444444      /*  PE7...PE0 */
#define VAL_GPIOECRH            0x44444444      /* PE15...PE8 */
#define VAL_GPIOEODR            0xFFFFFFFF

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
