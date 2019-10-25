/*
 * Copyright (c) 2019, Swedish Institute of Computer Science.
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
 */

/**
 * \file
 *         Testing the broadcast layer in Rime
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

 /**
   * \Modified file
   *        WuRx firmware a FeuerWhere nodes (FW-node) to work as WuRx
   * \author
   *         Juan Aranda <juan-aranda@javeriana.edu.co>
   *         Date: 27/02/2019
 */


#include "contiki.h"
#include "random.h"
#include "dev/cc2520/cc2520.h"
#include "dev/cc2520/cc2520_const.h"
#include "net/netstack.h"
#include "wur.h"
#include "dev/lpm.h"

#include <stdio.h>

/*---------------------------------------------------------------------------*/
PROCESS(wurx_firm_process, "WuRx firmware no RIME");
AUTOSTART_PROCESSES(&wurx_firm_process);
/*--------------------------------------------------------------------------*/
// static void wur_trigger_tx()
// {
//   printf("WuRx: Triggering Signal to MR!\n");
//   wur_set_tx();
//   clock_delay(100);
//   wur_clear_tx();
// };

/*---------------------------------------------------------------------------*/
static void
on(void)
{
    NETSTACK_RADIO.on();
};


/*---------------------------------------------------------------------------*/
//src: https://stackoverflow.com/questions/33833362/how-does-contiki-os-process-external-interrupts
//This proceeding prevents long lasting calculations in the ISR

/* ContikiOS Wiki
A poll request is a special type of event. A process is polled by calling the function process_poll().
Calling this function on a process causes the process to be scheduled as quickly as possible.
The process is passed a special event that informs the process that it has been polled.
Polling is the way to make a process run from an interrupt.
The process_poll() function is the only function in the process module that is safe to call from preemptive mode.
*/
int
wurx_interrupt(void)
{
  process_poll(&wurx_firm_process);
  printf("INTERRUPT : WuRx - RF signal received the radio!\n");
  return 1;
}

/*--------------------------------------------------------------------------*/
PROCESS_THREAD(wurx_firm_process, ev, data)
{
  static struct etimer timeout;
  PROCESS_BEGIN();

  //lpm_off();
  on();

  while(1)
  {
    /*If a cc2520 interrupt occurrs, i.e., a packet has been received,
      * a wake signal is trigged via GPIO
      */
    //PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    //PROCESS_WAIT_EVENT(); // INTERRUPT AT CC2520 LEVEL
    //PROCESS_WAIT_EVENT_UNTIL(0);
    // {
    //   //wur_trigger_tx(); // trigger the WURPSIG to the MR's MCU
    // };

    // cc2520_off();
    // cc2520_init();
    // cc2520_on();

    #if CONTIKI_TARGET_ZOUL
      // DO nothing
    #else
      etimer_set(&timeout, 15 * CLOCK_SECOND); //60
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timeout));
      printf("Resetting!\n");
      watchdog_reboot(); // to reset the mote by software
    #endif

  };

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
