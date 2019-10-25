/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 *ISR implementation for both button.
 */
#include "contiki.h"
#include "lib/sensors.h"
#include "dev/hwconf.h"
#include "dev/button-sensor.h"
#include "isr_compat.h"
#include "button-isr.h"
#include <stdio.h>

#include "net/mac/wurmac/wrimac-broadcast.h"

HWCONF_PIN(BUTTON, 2, 6);
HWCONF_IRQ(BUTTON, 2, 6);
HWCONF_PIN(BUTTON2, 2, 7);
HWCONF_IRQ(BUTTON2, 2, 7);
/*--------------------------------*/
HWCONF_PIN(MR, 2, 3); //MCU Interrupt PIN P2.3 (input signal from WuRx)
HWCONF_IRQ(MR, 2, 3);
/*--------------------------------*/

struct timer debouncetimer, debouncetimer2, debouncetimerWrx;
/*---------------------------------------------------------------------------*/
ISR(PORT2, irq_p2)
{
    ENERGEST_ON(ENERGEST_TYPE_IRQ);

    // /*--------------------------------*/
    // if(MR_CHECK_IRQ())
    // {
    //   if(timer_expired(&debouncetimerWrx))
    //   {
    //     timer_set(&debouncetimerWrx, CLOCK_SECOND / 4);
    //     sensors_changed(&wur_sensor);
    //     LPM4_EXIT;
    //     printf("WUR_SENSOR: INTERRUPT : Trigger signal received by MR!\n");
    //   };
    //   P2IFG &= ~BIT3;   //P2.3 BUTTON_CLEAR_IRQ();
    // };

    /*--------------------------------*/
    if(MR_CHECK_IRQ())
    {
      if(wrimac_interrupt())
      {
        LPM4_EXIT;
      };
      P2IFG &= ~BIT3;   //P2.3 BUTTON_CLEAR_IRQ();
    };
    /*--------------------------------*/

    if(BUTTON_CHECK_IRQ()) {
        if(timer_expired(&debouncetimer)) {
            timer_set(&debouncetimer, CLOCK_SECOND / 8);
            sensors_changed(&button_sensor);
            LPM4_EXIT;
        }
        P2IFG &= ~(BUTTON_CHECK_IRQ()); //// Clear interrupt flag for P2.6 = P2IFG &= (~BIT6);
    }
    if(BUTTON2_CHECK_IRQ()) {
        if(timer_expired(&debouncetimer2)) {
            timer_set(&debouncetimer2, CLOCK_SECOND / 8);
            sensors_changed(&button_sensor2);
            LPM4_EXIT;
              printf("BUTTON: INTERRUPT!\n");
        }
        P2IFG &= ~(BUTTON2_CHECK_IRQ());
    }
    ENERGEST_OFF(ENERGEST_TYPE_IRQ);
};
