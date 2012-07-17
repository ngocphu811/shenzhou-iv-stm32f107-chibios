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

#include "board_led.h"

void led1_on(void)
{
   palClearPad(GPIOD, GPIOD_LED1_STATUS);
}

void led1_off(void)
{
   palSetPad(GPIOD, GPIOD_LED1_STATUS);
}

void led2_on(void)
{
   palClearPad(GPIOD, GPIOD_LED2_STATUS);
}

void led2_off(void)
{
   palSetPad(GPIOD, GPIOD_LED2_STATUS);
}

void led3_on(void)
{
   palClearPad(GPIOD, GPIOD_LED3_STATUS);
}

void led3_off(void)
{
   palSetPad(GPIOD, GPIOD_LED3_STATUS);
}

void led4_on(void)
{
   palClearPad(GPIOD, GPIOD_LED4_STATUS);
}

void led4_off(void)
{
   palSetPad(GPIOD, GPIOD_LED4_STATUS);
}

void led_all_on(void)
{
   led1_on();
   led2_on();
   led3_on();
   led4_on();
}

void led_all_off(void)
{
   led1_off();
   led2_off();
   led3_off();
   led4_off();
}

msg_t thread_led_test(void *arg) 
{
  (void)arg;
  chRegSetThreadName("blinker_led_test");
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
