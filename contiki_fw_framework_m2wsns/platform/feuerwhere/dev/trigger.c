/*
 * Copyright (c) 2019 PUJ-USeA.
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
  * \File
  *         Code that triggering signal via GPIO when
  *         a message is received
  * \author
  *         Juan Aranda <juan-aranda@javeriana.edu.co>
  *         Date: 25/02/2019
  */

#include "contiki.h"
#include "dev/hwconf.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "trigger.h"
#include <stdio.h>

/*
 *	Debug
 *
 */
#define LOG_DEBUG 1
#if LOG_DEBUG
#define LOG_DBG(...) printf(__VA_ARGS__)
#else
#define LOG_DBG(...)
#endif

/*---------------------------------------------------------------------------*/
HWCONF_PIN(TRIGGER, 6, 0); // GPIO configuraiton, select P6.0 from Extension header
/*---------------------------------------------------------------------------*/

void trigger_set()
{
	TRIGGER_SET();
  //leds_toggle(LEDS_RED);
  LOG_DBG("Set/Send the trigger signal, high GPIO!\n");
};

void trigger_clear()
{
	TRIGGER_CLEAR();
  //leds_off(LEDS_RED);
  LOG_DBG("Clear the signal, low GPIO!\n");
};

void trigger_init()
{
  LOG_DBG("Configure TRIGGER GPIO as output!\n");
  TRIGGER_SELECT();
	TRIGGER_MAKE_OUTPUT();
	TRIGGER_CLEAR();
};
