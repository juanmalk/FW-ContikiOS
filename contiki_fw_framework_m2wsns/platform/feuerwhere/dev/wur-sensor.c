/*
 * Copyright (c) 2018, Bruno Kessler Foundation, Trento, Italy and
 * ETH, IIS, Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * \author
 *         Rajeev Piyare <rajeev.piyare@hotmail.com>
 *         This is the wake-up receiver interrupt handler
 */
/**
  * \Modified file
  *        Add support to FeuerWhere nodes (FW-node)
  * \author
  *         Juan Aranda <juan-aranda@javeriana.edu.co>
  *         Date: 26/02/2019
*/

#include "contiki.h"
#include "lib/sensors.h"
#include "dev/hwconf.h"
#include "dev/button-sensor.h"
#include "isr_compat.h"
#include "button-isr.h"
#include "wur.h"
#include "dev/leds.h"
#include <stdio.h>

/*
 *	Debug
 *
 */
#define LOG_DEBUG 0
#if LOG_DEBUG
#define LOG_DBG(...) printf(__VA_ARGS__)
#else
#define LOG_DBG(...)
#endif

const struct sensors_sensor wur_sensor;

//static struct timer debouncetimerWrxWrx;
static int status(int type);

/* Main Radio (MR) and WuRx ports for FeuerWhere nodes (FW-node) */
HWCONF_PIN(MR, 2, 3); //MCU Interrupt PIN P2.3 (input signal from WuRx)
HWCONF_IRQ(MR, 2, 3);
HWCONF_PIN(WUR_TX, 6, 0); //GPIO P6.0 Triggering signal (output signal)
/*---------------------------------------------------------------------------*/
// ISR(PORT2, irq_p2) // Port 2 is enable for interrupt handler (in button-isr.c)
// {
//   ENERGEST_ON(ENERGEST_TYPE_IRQ);
//   if(MR_CHECK_IRQ())
//   {
//     if(timer_expired(&debouncetimerWrx))
//     {
//       leds_on(LEDS_RED);
//       timer_set(&debouncetimerWrx, CLOCK_SECOND / 4);
//       sensors_changed(&wur_sensor);
//       leds_off(LEDS_RED);
//       LPM4_EXIT;
//       LOG_DBG("WUR_SENSOR: INTERRUPT : Trigger signal received by MR!\n");
//     };
//   };
//
//   P2IFG &= ~BIT3;   //P2.3 MR_CLEAR_IRQ();
//   ENERGEST_OFF(ENERGEST_TYPE_IRQ);
// }
/*---------------------------------------------------------------------------*/
static int
value(int type)
{
  return MR_READ() || !timer_expired(&debouncetimerWrx);
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int c)
{
  switch (type) {
  case SENSORS_ACTIVE:
    if (c) {
      if(!status(SENSORS_ACTIVE)) {

      timer_set(&debouncetimerWrx, 0);

      // MR_SET();
      // MR_MAKE_INPUT();
      // P2REN |= BIT3; //Port P2 resistor enable/  LOG_DBG("WUR_SENSOR: INTERRUPT : Trigger signal received!\n");/MR_ENABLE_PULLUP();
      // MR_IRQ_EDGE_SELECTD();
      // //MR_IRQ_EDGE_SELECTU();
      // MR_ENABLE_IRQ();
      // P2IFG &= ~BIT3; //MR_CLEAR_IRQ();
      MR_IRQ_EDGE_SELECTU();
    	MR_SELECT();
    	MR_MAKE_INPUT();
    	MR_ENABLE_IRQ();
      LOG_DBG("WUR_SENSOR: CONFIG : MR INPUT GPIO!\n");
      }
    } else {
      MR_DISABLE_IRQ();
      LOG_DBG("WUR_SENSOR: CONFIG : MR DISABLE_IRQ!\n");
    }
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
void wur_init() {
  WUR_TX_SELECT();
  WUR_TX_MAKE_OUTPUT();
  WUR_TX_CLEAR();
  LOG_DBG("WUR_SENSOR: INIT : WUR OUPUT GPIO!\n");
  //P2DIR |= BIT3;
}
/*---------------------------------------------------------------------------*/
void wur_set_tx() {
  LOG_DBG("WUR_SENSOR: Set/Send the trigger signal, high GPIO!\n");
  WUR_TX_SET();
}
/*---------------------------------------------------------------------------*/
void wur_clear_tx() {
  LOG_DBG("WUR_SENSOR: Clear the signal, low GPIO!\n");
  WUR_TX_CLEAR();
}
// /*---------------------------------------------------------------------------*/
// void wur_rx_done() {
//   P2OUT |= BIT3;
// }
// /*---------------------------------------------------------------------------*/
// void wur_clear_rx() {
//   P2OUT &= ~BIT3;
// }
/*---------------------------------------------------------------------------*/
void wur_enable_ISR()
{
  MR_ENABLE_IRQ();
  LOG_DBG("WUR_SENSOR: CONFIG : MR ENABLE_IRQ!\n");
};
/*---------------------------------------------------------------------------*/
void wur_disable_ISR()
{
  MR_DISABLE_IRQ();
  LOG_DBG("WUR_SENSOR: CONFIG : MR DISABLE_IRQ!\n");
};
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  switch (type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return MR_IRQ_ENABLED();
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(wur_sensor, WUR_SENSOR, value, configure, status);
