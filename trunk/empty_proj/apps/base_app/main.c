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

#include "ch.h"
#include "hal.h"
#include "test.h"

/*
 * Green LED blinker thread, times are in milliseconds.
 */

void led1_on()
{
   palClearPad(GPIOD, GPIOD_LED1_STATUS);
}

void led1_off()
{
   palSetPad(GPIOD, GPIOD_LED1_STATUS);
}

void led2_on()
{
   palClearPad(GPIOD, GPIOD_LED2_STATUS);
}

void led2_off()
{
   palSetPad(GPIOD, GPIOD_LED2_STATUS);
}

void led3_on()
{
   palClearPad(GPIOD, GPIOD_LED3_STATUS);
}

void led3_off()
{
   palSetPad(GPIOD, GPIOD_LED3_STATUS);
}

void led4_on()
{
   palClearPad(GPIOD, GPIOD_LED4_STATUS);
}

void led4_off()
{
   palSetPad(GPIOD, GPIOD_LED4_STATUS);
}

void led_all_on()
{
   led1_on();
   led2_on();
   led3_on();
   led4_on();
}

void led_all_off()
{
   led1_off();
   led2_off();
   led3_off();
   led4_off();
}

static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (TRUE) {
    led1_on();
    chThdSleepMilliseconds(500);
    led1_off();
    chThdSleepMilliseconds(500);
    led2_on();
    chThdSleepMilliseconds(500);
    led2_off();
    chThdSleepMilliseconds(500);
    led3_on();
    chThdSleepMilliseconds(500);
    led3_off();
    chThdSleepMilliseconds(500);
    led4_on();
    chThdSleepMilliseconds(500);
    led4_off();
    chThdSleepMilliseconds(500);
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the serial driver 3 using the driver default configuration.
   */
  //sdStart(&SD1, NULL);

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (TRUE) {
    //if (palReadPad(GPIOC, GPIOC_SWITCH_TAMPER) == 0)
    //  TestThread(&SD1);
    chThdSleepMilliseconds(500);
  }
}
