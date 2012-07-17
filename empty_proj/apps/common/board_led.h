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

#ifndef _BOARD_LED_H_
#define _BOARD_LED_H_

#include "ch.h"
#include "hal.h"

void led1_on(void);
void led2_on(void);
void led3_on(void);
void led4_on(void);
void led1_off(void);
void led2_off(void);
void led3_off(void);
void led4_off(void);
void led_all_on(void);
void led_all_off(void);

msg_t thread_led_test(void *arg);

#endif //_BOARD_LED_H_
