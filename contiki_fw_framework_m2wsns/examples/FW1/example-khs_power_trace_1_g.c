	/*
 * Copyright (c) 2017 PUJ-USeA.
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
 *         Example of how the Kang's hybrid node scheduling works
 *
 * \ref    Kang, Y., Hu, B., Ding, Y., & Tan, J. (2014). A hybrid node scheduling
 *         approach based on energy efficient chain routing for WSN. Advances in
 *         Mechanical Engineering, 2014, 1–12. https://doi.org/10.1155/2014/254761
 *
 * \author
 *         juanmalk <juan-aranda@javeriana.edu.co>
 *    leds_toggle(LEDS_SINK_NODE);              <juan.aranda@usa.edu.co>
 * \date_last_version
 * 					27/11/2017
 */


//#include "contiki.h"
#include "net/rime/khs.h"
#include "lib/random.h"
#include "powertrace.h"
#include "net/packetbuf.h"
#include "powertrace.h"
#include "dev/leds.h"
#include <stdio.h>
#include <stdlib.h>

#define TIME_SYNCH 0


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

#define CM5000 0

// Parameters
#define GRID_ID 1
#define TIER_ID 1
#define INIT_BAT 100
#define NUM_SN_GRID 5 //w/o sink node

// Channels definition for message exchange
#define BROADCAST_CH 129
#define RUNICAST_CH 144
#define COLLECT_NODE_QUEUE_CH 130

/*
 *	Struct & variables definition
 *
 */

khs_node c_node;
khs_flags c_flags;
khs_node_queue c_node_queue;
khs_working_msg c_working_msg;
khs_broadcast_msg c_broadcast_msg;
khs_join_msg c_join_msg;
khs_report_msg c_report_msg;
khs_neighbor_tier_info c_nti;

static struct broadcast_conn broadcast; // Broadcast connection.
static struct runicast_conn runicast; //Runicast connection.
static struct collect_conn tc; //Collect procedure

/*---------------------------------------------------------------------------*/
/* OPTIONAL: Sender history.
 * Detects duplicate callbacks at receiving nodes.
 * Duplicates appear when ack messages are lost. */
 #define NUM_HISTORY_ENTRIES 4
 struct history_entry {
   struct history_entry *next;
   linkaddr_t addr;
   uint8_t seq;
 };
LIST(history_table);
MEMB(history_mem, struct history_entry, NUM_HISTORY_ENTRIES);

/*
 *	Events definition
 *
 */

extern process_event_t ev_sleeping;
extern process_event_t ev_working;
extern process_event_t ev_assisting;
extern process_event_t ev_initializing;
extern process_event_t ev_init_khs;
process_event_t ev_sink;
process_event_t ev_init_rime;
/*
 *	Processes definition
 *
 */

PROCESS(wait_init, "Wait nodes starting up");
PROCESS(initializing, "Intializing state process");
PROCESS(working, "Working state process");
PROCESS(assisting, "Assisting state process");
PROCESS(sleeping, "Sleeping state process");
PROCESS(sinking, "Sink process");
PROCESS(send_out_msg, "Sending stored packetprocess");


AUTOSTART_PROCESSES(&wait_init);

/*
 *	Functions
 *
 */

static void
recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno)
{
	 /* OPTIONAL: Sender history */
	 // Start: Code taken form example-runicast
	struct history_entry *e = NULL;
	for(e = list_head(history_table); e != NULL; e = e->next)
	{
		if(linkaddr_cmp(&e->addr, from))
	 	{
	 		break;
	 	};
	};

	if(e == NULL)
	{
		/* Create new history entry */
		e = memb_alloc(&history_mem);
		if(e == NULL)
		{
			e = list_chop(history_table); /* Remove oldest at full history */
		};

		linkaddr_copy(&e->addr, from);
		e->seq = seqno;
		list_push(history_table, e);
		//printf("runicast message received from %d.%d, seqno %d\n",
		//	from->u8[0], from->u8[1], seqno);

		/* My code*/
		khs_rucast_recv (&c_node, c, packetbuf_dataptr(), from, seqno);

	}
	else
	{
		/* Detect duplicate callback */
		if(e->seq == seqno)
		{
			printf("runicast message received from %d.%d, seqno %d (DUPLICATE)\n",
			from->u8[0], from->u8[1], seqno);
			return;
		};

		/* Update existing history entry */
		e->seq = seqno;
		//printf("runicast message received from %d.%d, seqno %d\n",
		//	from->u8[0], from->u8[1], seqno);
		// End: Code taken form example-runicast

		/* My code*/
		khs_rucast_recv (&c_node, c, packetbuf_dataptr(), from, seqno);
	};
};

static void
timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions)
{
	khs_rucast_timedout (&c_node, c, to, retransmissions);
};

static const struct runicast_callbacks runicast_call = {recv_runicast,
                                                        timedout_runicast};
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	khs_bcast_recv (&c_node, c, packetbuf_dataptr(), from);
};

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};

static void
collect_recv(const linkaddr_t *originator, uint8_t seqno, uint8_t hops)
{
  printf("Sink received from %d.%d, seqno %d, hops %d: len %d '%s'\n",
	 originator->u8[0], originator->u8[1],
	 seqno, hops,
	 packetbuf_datalen(),
	 (char *)packetbuf_dataptr());

	khs_rucast_recv(&c_node, NULL, packetbuf_dataptr(), originator, seqno);

};
static const struct collect_callbacks collect_call = { collect_recv };


/*---------------------------------------------------------------------------*/

/*
 *	KHS processes
 *
 */

/*---------------------------------------------------------------------------*/

 /*
 *	Init KHS process
 *
 */

 PROCESS_THREAD(wait_init, ev, data)
 {
 	PROCESS_EXITHANDLER(broadcast_close(&broadcast));
	PROCESS_EXITHANDLER(runicast_close(&runicast));
	PROCESS_BEGIN();

    // Current node in Grid #1 & Tier#1 (line of sight with the Base Station)
    uint8_t state = KHS_ST_SLEEPING;
    uint8_t grid_id = GRID_ID;
    uint8_t tier_id = TIER_ID;
    uint8_t msg_type = 250;
    uint8_t turn_sche = -1;
    uint8_t sensed_value = 0;
    uint8_t batt_level = INIT_BAT;
    uint16_t Nj = NUM_SN_GRID; //ONLY SNs, not sink node

    const linkaddr_t *node_addr = &linkaddr_node_addr;
    const linkaddr_t *next_node_queue_addr = NULL; //set in khs_builiding_queue_proc
    const linkaddr_t *current_working_node_addr = &linkaddr_node_addr;
    static struct etimer t_collect;
    static struct etimer t_idle;
    static struct etimer t_sleep;
    static struct etimer t_work;
    static struct etimer t_start;
    static struct etimer t_event;
    static struct etimer t_sigma;
    static struct etimer p_collect;
    static struct etimer p_report_sink;
    static struct etimer p_scheduling;
    static struct etimer t_init_sche;
    static struct etimer t_wait;
    static struct etimer p_sampling;
    uint8_t flag_dataset = ON;
    uint8_t flag_sim = ON;

    // Current node in Grid #0 & Tier#0 (Base Station (sink))
    uint8_t nlt_grid_id = 0; // Grid_id = 0 --> BS grid.
    uint8_t nlt_tier_id = 0; // Tier_id = 0 --> BS tier.
    uint8_t nlt_status = KHS_STATUS_WAIT_REPORT;
    uint8_t nlt_state = KHS_ST_WORKING; //Always-on

    // Process event allocation
    ev_sink = process_alloc_event();
    ev_init_rime = process_alloc_event();
    ev_initializing = process_alloc_event();
    ev_sleeping = process_alloc_event();
    ev_working = process_alloc_event();
    ev_assisting = process_alloc_event();
    ev_init_khs = process_alloc_event();

    #if TIME_SYNCH
      // Set the Authority level for Implicit Network Time Synchronization - timesynch file
      unsigned short id = linkaddr_node_addr.u8[0];
      timesynch_set_authority_level(id);
      printf("id: %d, timesynch_authority_level: %d \n", id, timesynch_authority_level());
      printf("CLOCK_SECOND: %lu\n", CLOCK_SECOND);
    #else
      //do nothing
    #endif

    // Start powertracing, once every ten seconds
    #if FW1_ACTIVE
      if(linkaddr_node_addr.u8[0] != 1 && linkaddr_node_addr.u8[0] != 0 )
      {
        powertrace_start(CLOCK_SECOND * 10);
      }
    #else
        powertrace_start(CLOCK_SECOND * 10);
    #endif

    // Open broadcast & runicast connection
    broadcast_open(&broadcast, BROADCAST_CH, &broadcast_call);
    runicast_open(&runicast, RUNICAST_CH, &runicast_call);

    /* OPTIONAL: Sender history: Taken from example-runicast */
    list_init(history_table);
    memb_init(&history_mem);

    // Open a collect app
    collect_open(&tc, COLLECT_NODE_QUEUE_CH, COLLECT_ROUTER, &collect_call);

  	// ev_init_khs (asynchronous event) is posted
  	process_post(PROCESS_CURRENT(), ev_init_rime, NULL);

   	while(1)
   	{
   		PROCESS_WAIT_EVENT();

   	 	if(ev == ev_init_rime)
     	{
	        #if FW1_ACTIVE
	          // RE-mote node aa:ee
	         if(linkaddr_node_addr.u8[0] == 1 && linkaddr_node_addr.u8[1] == 0)
	           {
                // Set as node in the lower tier
     	        	linkaddr_copy(&c_node.current_working_node_addr, current_working_node_addr);
     						// Set as node in the lower tier
     	        	c_node.node_addr =  node_addr;

     	        	//Transit to the Sink process
     	   	 			process_start(&sinking, NULL);
     	   	 			process_post(&sinking, ev_sink, NULL);
	            }
	            else
	            {
	              // ev_init_khs (asynchronous event) is posted
	              process_post(PROCESS_CURRENT(), ev_init_khs, NULL);
	            };
	        #elif CM5000
	            // CM5000 ID:14 sink node
	   	 		if(linkaddr_node_addr.u8[0] == 14 && linkaddr_node_addr.u8[1] == 0)
	   	 		{

	   	 			// Set as node in the lower tier
	        		linkaddr_copy(&c_node.current_working_node_addr, current_working_node_addr);

	        		//Transit to the Sink process
	   	 			process_start(&sinking, NULL);
	   	 			process_post(&sinking, ev_sink, NULL);
	   	 		}
	   	 		else
	   	 		{
	   	 			//Stay in the same state
	   	 			process_post(PROCESS_CURRENT(), ev_init_khs, NULL);
	   	 		};
	        #else
	           if(linkaddr_node_addr.u8[0] == 1 && linkaddr_node_addr.u8[1] == 0)
	           {
	         		   // Set as node in the lower tier
	       			   linkaddr_copy(&c_node.current_working_node_addr, current_working_node_addr);
	        		   //Transit to the Sink process
	   	 			     process_start(&sinking, NULL);
					       process_post(&sinking, ev_sink, NULL);;
	            }
	            else
	            {
	              // ev_init_khs (asynchronous event) is posted
	              process_post(PROCESS_CURRENT(), ev_init_khs, NULL);
	            };
	        #endif
     	};

     	if(ev == ev_init_khs)
     	{

	    	// Initialization variables KHS
		    khs_init_proc (&c_node, state, &c_flags, grid_id, tier_id, msg_type, turn_sche, sensed_value,
		                   batt_level, Nj,  node_addr, next_node_queue_addr, current_working_node_addr,
		                   &c_node_queue, &c_nti, &t_collect, &t_idle, &t_sleep, &t_work, &t_start,
		                   &t_event, &t_sigma, &p_collect, &p_report_sink, &tc, &p_scheduling, &t_init_sche, &p_sampling,
	                      &t_wait, flag_dataset, flag_sim);

	      #if FW1_ACTIVE
          // Set node addr in the lower tier
          linkaddr_t nlt_addr;
          nlt_addr.u8[0] = 1;
          nlt_addr.u8[1] = 0;
	      #elif CM5000
          // Set node addr in the lower tier
    			linkaddr_t nlt_addr;
    			nlt_addr.u8[0] = 14;
    			nlt_addr.u8[1] = 0;
	      #else
	        // Set node addr in the lower tier
	        linkaddr_t nlt_addr;
	        nlt_addr.u8[0] = 1;
	        nlt_addr.u8[1] = 0;
	      #endif

	      khs_update_neighbor_lower_tier_info_proc (&c_nti, nlt_grid_id, nlt_tier_id,
	                   &nlt_addr, nlt_status, nlt_state);

		   // Start other processes
		   process_start(&initializing, NULL);
		   process_start(&working, NULL);
		   process_start(&assisting, NULL);
		   process_start(&sleeping, NULL);

		   //Transit to the Sleeping state
		   process_post(&sleeping, ev_sleeping, PROCESS_CURRENT());
		   LOG_DBG("LOG: Example-khs.c : ev_init_khs - post sleeping. \n");
		};
	};
	PROCESS_END();
};


 /*---------------------------------------------------------------------------*/

 /*
 *	Initializing state process
 *
 */

static uint8_t flag_init_st = 0;

PROCESS_THREAD(initializing, ev, data)
{
	PROCESS_BEGIN();

	while(1)
	{
		PROCESS_WAIT_EVENT();

		if(ev == ev_initializing)
		{
			if (!flag_init_st)
			{
				etimer_set(c_node.t_start, c_node.interval_start);
				khs_set_event_timeout (&c_node);
				flag_init_st = 1;
			};

			if (c_node.state == KHS_ST_INITIALIZING)
			{
		      	// Node queue building process
		      	khs_exploring_proc (&c_node, &runicast, &broadcast);

		      	//Stay in the same state
		      	process_post(PROCESS_CURRENT(), ev_initializing, NULL);
		    }
		    else if (c_node.state == KHS_ST_SLEEPING)
		    {
		    	// Transit to the Sleeping state
          		process_start(&send_out_msg, NULL);
          		process_post(&sleeping, ev_sleeping, NULL);
        		LOG_DBG("LOG: Example-khs.c : ev_initializing - post sleeping. \n");
        	}
        	else if (c_node.state == KHS_ST_WORKING)
       		{
        		// Transit to the Working state
           		process_start(&send_out_msg, NULL);
        		process_post(&working, ev_working, NULL);
        		LOG_DBG("LOG: Example-khs.c : ev_initializing - post working. \n");
        	};
     	};
    };
    PROCESS_END();
};

/*---------------------------------------------------------------------------*/

 /*
 *	Workings state process
 *
 */

PROCESS_THREAD(working, ev, data)
{
	PROCESS_BEGIN();

	while(1)
	{
		PROCESS_WAIT_EVENT();

		if(ev == ev_working)
		{
	        if (c_node.state == KHS_ST_WORKING)
	        {
	        	// Call Node Scheduling and Data Gathering methods
	        	khs_collecting_data_proc (&c_node, &c_nti, &runicast, &broadcast);
	        	khs_sleeping_proc (&c_node, &runicast, &broadcast);
	        	khs_wakeup_proc (&c_node, &runicast, &broadcast);

	        	//Stay in the same state
	        	process_post(PROCESS_CURRENT(), ev_working, NULL);
	        }
	        else if (c_node.flags->flag_still_rcast_msg_to_send)
	        {
	        	//Stay in the same state
	        	process_post(PROCESS_CURRENT(), ev_working, NULL);
	        }
	        else if (c_node.state == KHS_ST_SLEEPING)
	        {
	        	//Transit to Sleeping state
	        	process_post(&sleeping, ev_sleeping, NULL);
	        	LOG_DBG("LOG: Example-khs.c : ev_working - post sleeping. \n");
	        }
	        else if (c_node.state == KHS_ST_ASSISTING)
	        {
	        	//Transit to Assiting state
	        	process_post(&assisting, ev_assisting, NULL);
	        	LOG_DBG("LOG: Example-khs.c : ev_working - assisting. \n");
	        };
	    };
	};
	PROCESS_END();
};

/*---------------------------------------------------------------------------*/

 /*
 *	Assisting state process
 *
 */

PROCESS_THREAD(assisting, ev, data)
{
	PROCESS_BEGIN();

	while(1)
	{
		PROCESS_WAIT_EVENT();

		if(ev == ev_assisting)
		{
			if (c_node.state == KHS_ST_ASSISTING)
			{
				// Call Data Gathering metho
				khs_collecting_data_proc (&c_node, &c_nti, &runicast, &broadcast);
				// Available for sending assisting msg to other SNs in the grid
      			khs_sleeping_proc (&c_node, &runicast, &broadcast);
      			khs_wakeup_proc (&c_node, &runicast, &broadcast);
      			//Stay in the same state
      			process_post(PROCESS_CURRENT(), ev_assisting, NULL);
      		}
      		else if (c_node.state == KHS_ST_SLEEPING)
      		{
            //Transit to Sleeping state
            khs_clear_out_msg_queue (&c_node);
      			process_post(&sleeping, ev_sleeping, NULL);
      			LOG_DBG("LOG: Example-khs.c : ev_assisting - post sleeping. \n");
      		}
      		else if (c_node.state == KHS_ST_WORKING)
      		{
      			//Transit to Working state
     				khs_clear_out_msg_queue (&c_node);
      			process_post(&working, ev_working, NULL);
      			LOG_DBG("LOG: Example-khs.c : ev_assisting - post working. \n");
      		};
      	};
    };
    PROCESS_END();
};

/*---------------------------------------------------------------------------*/
 /*
 *	Sleeping state process
 *
 */

PROCESS_THREAD(sleeping, ev, data)
{
	PROCESS_BEGIN();

	static struct process *last_process = NULL;
	static struct etimer t_0;

	while(1)
	{
		PROCESS_WAIT_EVENT();

		if(ev == ev_sleeping)
		{
			last_process = (struct process *) data;

			if(last_process == &wait_init)
			{

				// Set a preset time
				/* Allow some time for 1st neighbor discovery process
					for node queue building purpose */
				etimer_set(&t_0, (KHS_T_START)* CLOCK_SECOND);
				// Wait t_0 to transit from Sleeping state to Initializing state
				PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&t_0));
				LOG_DBG("LOG: Finished timer for preset KHS initial procedure.\n");

				// Transit to INITIALIZING state
				c_node.state = KHS_ST_INITIALIZING;
				process_post(&initializing, ev_initializing, NULL);
				LOG_DBG("LOG: Example-khs.c : ev_sleeping - post intializing. \n");
			}
			else
			{
				if (c_node.state == KHS_ST_SLEEPING)
				{
					khs_sleeping_proc (&c_node, &runicast, &broadcast); //check
        			khs_wakeup_proc (&c_node, &runicast, &broadcast);

        			//Stay in the same state
					process_post(PROCESS_CURRENT(), ev_sleeping, NULL);
				}
				else if (c_node.state == KHS_ST_WORKING)
				{
					//Transit to Working state
					process_post(&working, ev_working, NULL);
					LOG_DBG("LOG: Example-khs.c : ev_sleeping - post working. \n");
				}
				else if (c_node.state == KHS_ST_ASSISTING)
				{
					//Transit to Assisting state
					process_post(&assisting, ev_assisting, NULL);
					LOG_DBG("LOG: Example-khs.c : ev_sleeping - post assiting \n");
				}
			};
		};
	};
	PROCESS_END();
};

/*---------------------------------------------------------------------------*/
 /*
 *	Sending packet process
 *
 */

PROCESS_THREAD(send_out_msg, ev, data)
{
  static struct etimer et1, et2;
  static struct khs_out_msg_queue *out;
  struct khs_report_msg khs_report_msg;

	PROCESS_BEGIN();

	while(1)
	{
		//REMOVE from the list
    	//Execute periodically
    	etimer_set(&et1, CLOCK_SECOND * KHS_TIME_OUT_MSG);
    	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et1));

    	if(c_node.state != KHS_ST_SLEEPING)
        {

        	//LOG_DBG("LOG: After c_node.state condition in send_out_msg, out_msg_queue_list: length: %d\n",
	       //     	list_length(c_node.p_out_msg_list));

        	//LOG_DBG("LOG: Before 2nd timer expired in send_out_msg, tc.sending: %d\n",
	        //    	tc.sending);

        	while(list_length(c_node.p_out_msg_list))
        	{
        		//Give enough time to transmit the previous msg
        		//etimer_set(&et2, CLOCK_SECOND * KHS_TIME_PRE_OUT_MSG + random_rand() % (CLOCK_SECOND * KHS_TIME_PRE_OUT_MSG));
        		etimer_set(&et2, CLOCK_SECOND * KHS_TIME_PRE_OUT_MSG);
            PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et2));

            if (!tc.sending)
            {
	             //Remove from list
	             out = list_chop(c_node.p_out_msg_list); // Remove the last object on the list.

	             if (out->msg_id == KHS_MSG_DATA_REPORT)
               {
	        			khs_report_msg.msg_id = KHS_MSG_DATA_REPORT;
	        			khs_report_msg.src_grid_status = KHS_STATUS_COLLECTING;
              }
	      			else
	      			{
	        			khs_report_msg.msg_id = KHS_MSG_ALARM_REPORT;
	        			khs_report_msg.src_grid_status = KHS_STATUS_ALERTING;
	      			};

  	      			khs_report_msg.last_seqno = out->seqno;
  	      			khs_report_msg.src_grid_id = c_node.grid_id;
  	      			khs_report_msg.src_tier_id = c_node.tier_id;
  	      			khs_report_msg.dst_tier_id = c_node.khs_nti->tier_id;
  	      			linkaddr_copy(&khs_report_msg.src, c_node.node_addr);
  	      			const linkaddr_t *aux = &out->dst_addr;
  	      			linkaddr_copy(&khs_report_msg.dst, aux);
  	      			khs_report_msg.sensed_value = out->sensed_value;
  	      			packetbuf_copyfrom(&khs_report_msg, sizeof(khs_report_msg));

  	      			collect_send(&tc, MAX_RETRANSMISSIONS); //send to the sink directly

  	      			LOG_DBG("LOG: Node %d.%d: sending a msg (typ: %u) to the sink (%d.%d).\n",
                      c_node.node_addr->u8[0], c_node.node_addr->u8[1],
                      out->msg_id, aux->u8[0], aux->u8[1]);

  	            memb_free(c_node.p_out_msg_memb, out);
	            }
	            else
	            {
	            	LOG_DBG("LOG: WARNING: break from sending_packet_out\n"); //"Enough" time is not enough!!
                  	break;
	            };
            };
        };
    };

    PROCESS_END();
};

/*---------------------------------------------------------------------------*/
 /*
 *	Sink process
 *
 */

PROCESS_THREAD(sinking, ev, data)
{


  PROCESS_BEGIN();

  #if FW1_ACTIVE
    if(linkaddr_node_addr.u8[0] == 1 && linkaddr_node_addr.u8[1] == 0)
    {
      LOG_DBG("LOG: I am sink\n");
      collect_set_sink(&tc, 1);

	    // TURN ON ONBOARD LED - SLEEPING COLOR
	    leds_toggle(LEDS_SINK_NODE);
	   };
  #elif CM5000
      if(linkaddr_node_addr.u8[0] == 14 && linkaddr_node_addr.u8[1] == 0)
   		{
   			LOG_DBG("LOG: I am sink\n");

   			collect_set_sink(&tc, 1); // Real mote only one grid (the nearest one to the sink)

   			// TURN ON ONBOARD LED - SLEEPING COLOR
   			leds_toggle(LEDS_SINK_NODE);
 		};
  #else
	  if(linkaddr_node_addr.u8[0] == 1 &&
	      linkaddr_node_addr.u8[1] == 0) {
	        LOG_DBG("LOG: I am sink\n");

	        // Set this SN as the sink for all grids in the M2WSN
 			collect_set_sink(&tc, 1);

	        //Coloring node - Cooja
	        printf("#A color=GREEN\n");
	  };
  #endif

	while(1)
	{
		PROCESS_WAIT_EVENT();
	  	{
	  		//Wait to receive a message using callbacks
	  	};
	};
	PROCESS_END();
};
