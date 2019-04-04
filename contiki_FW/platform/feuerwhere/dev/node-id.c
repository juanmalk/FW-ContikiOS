/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
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
 *         Utility to store a node id in the external flash
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "sys/node-id.h"
#include "contiki-conf.h"
//#include "dev/xmem.h"
#include "dev/flash.h"
#include <string.h>


unsigned short node_id = 0;

uint8_t id[] = {0xab, 0xcd};
#define NODE_ID_MEM_ADDR 0x1980
/*---------------------------------------------------------------------------*/
void
node_id_restore(void)
{
    uint8_t buf[4];
    memcpy(buf, (uint8_t *) NODE_ID_MEM_ADDR, 4);
    if( !(buf[0] == id[0] && buf[1] == id[1])){
        buf[2] = 0x02;
        buf[3] = 0x03;
  }
  node_id = buf[3] | (buf[2] << 8);
}
/*---------------------------------------------------------------------------*/
void
node_id_burn(unsigned short id)
{
    flash_setup();
    flash_clear((unsigned short*)NODE_ID_MEM_ADDR);
    flash_write((unsigned short*)NODE_ID_MEM_ADDR, 0xabcd);
    flash_write(((unsigned short*)NODE_ID_MEM_ADDR + 2), id);
    flash_done();
}
/*---------------------------------------------------------------------------*/
