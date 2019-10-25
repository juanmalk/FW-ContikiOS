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
 *                  <juan.aranda@usa.edu.co>
 */


//#include "contiki.h"
#include "net/rime/m2dagnos.h"
#include "lib/random.h"
#include "powertrace.h"
#include "dev/leds.h"
#include "trigger.h"
#include "dev/cc2520/cc2520.h"
#include <stdio.h>
#include <stdlib.h>
#include "net/netstack.h"

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

/*
 *	Struct, variables & parameters definition
 *
 */

M2DAGNOS_node c_node;
M2DAGNOS_flags c_flags;
M2DAGNOS_working_msg c_working_msg;
M2DAGNOS_broadcast_msg c_broadcast_msg;
M2DAGNOS_report_msg c_report_msg;
M2DAGNOS_neighbor_tier_info c_nti;

static struct broadcast_conn broadcast; // Broadcast connection.
static struct runicast_conn runicast; //Runicast connection.
static struct collect_conn tc; //Collect procedure


// Parameters
#define GRID_ID 1
#define TIER_ID 1
#define INIT_BAT 100
#define NUM_SN_GRID 4//5//10 //w/o sink node
#define TH_P 0
#define TH_Q 0
#define PTX_LONG_OPR 11//5//7//11//15

// Channels definition for message exchange
#define BROADCAST_CH 129
#define RUNICAST_CH 144
#define COLLECT_NODE_QUEUE_CH 130

#define MULTIHOP 0
#if MULTIHOP
	#define COLLECT_CH 130
	static struct collect_conn tc_0; //Collect procedure
#else
#endif

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
process_event_t ev_monitoring;
process_event_t ev_sampling;
extern process_event_t ev_initializing;
extern process_event_t ev_init_m2dagnos;
process_event_t ev_sink;
process_event_t ev_init_rime;

/*
 *	Processes definition
 *
 */

PROCESS(wait_init, "M2DaGNoS - Wait nodes starting up");
PROCESS(initializing, "M2DaGNoS - Intializing state process");
PROCESS(working, "M2DaGNoS - Working state process");
PROCESS(assisting, "M2DaGNoS - Assisting state process");
PROCESS(sleeping, "M2DaGNoS - Sleeping state process");
PROCESS(sinking, "M2DaGNoS - Sink process");
PROCESS(sampling, "M2DaGNoS - Event_sampling state process");
PROCESS(monitoring, "M2DaGNoS - Continuous_monitoring state process");
PROCESS(send_out_msg, "M2DaGNoS - Sending stored packetprocess");

AUTOSTART_PROCESSES(&wait_init);

/*
 *	Functions
 *
 */

 // static void
 // sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions)
 // {
 //    printf("runicast message sent to %d.%d, retransmissions %d\n",
 //             to->u8[0], to->u8[1], retransmissions);
 // };
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
		M2DAGNOS_rucast_recv (&c_node, c, packetbuf_dataptr(), from, seqno);

	}
	else
	{
		/* Detect duplicate callback */
		if(e->seq == seqno)
		{
			LOG_DBG("runicast message received from %d.%d, seqno %d (DUPLICATE)\n",
			from->u8[0], from->u8[1], seqno);
			return;
		};

		/* Update existing history entry */
		e->seq = seqno;
		//printf("runicast message received from %d.%d, seqno %d\n",
		//	from->u8[0], from->u8[1], seqno);
		// End: Code taken form example-runicast

		/* My code*/
		M2DAGNOS_rucast_recv (&c_node, c, packetbuf_dataptr(), from, seqno);
	};
};

static void
timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions)
{
	M2DAGNOS_rucast_timedout (&c_node, c, to, retransmissions);
};

// static const struct runicast_callbacks runicast_call = {recv_runicast,
// 							     sent_runicast,
// 							     timedout_runicast};

static const struct runicast_callbacks runicast_call = {recv_runicast,
							     timedout_runicast};
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	M2DAGNOS_bcast_recv (&c_node, c, packetbuf_dataptr(), from);
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

	M2DAGNOS_rucast_recv (&c_node, NULL, packetbuf_dataptr(), originator, seqno);
};
static const struct collect_callbacks collect_call = { collect_recv };

/*---------------------------------------------------------------------------*/

/*
 *	M2DaGNoS processes
 *
 */

/*---------------------------------------------------------------------------*/

 /*
 *	Init M2DaGNoS process
 *
 */

PROCESS_THREAD(wait_init, ev, data)
{
	PROCESS_EXITHANDLER(broadcast_close(&broadcast));
	PROCESS_EXITHANDLER(runicast_close(&runicast));

	PROCESS_BEGIN();

	// Parameters declaration
	uint8_t state = M2DAGNOS_ST_INITIALIZING; //M2DAGNOS_ST_SLEEPING;
	uint8_t state_sub_process = M2DAGNOS_ST_WN_INIT;
	uint8_t grid_id = GRID_ID;
	uint8_t tier_id = TIER_ID;
	uint8_t msg_type = 250;
	uint8_t turn_sche = -1;
	uint8_t sensed_value = 0;
	uint8_t batt_level = INIT_BAT;
	uint16_t Nj = NUM_SN_GRID;
	uint16_t S_curr = 0;
	uint16_t S_prev = 0;
	uint8_t p = TH_P;
	uint8_t q = TH_Q;

	const linkaddr_t *node_addr = &linkaddr_node_addr;
	const linkaddr_t *next_node_queue_addr =  &linkaddr_node_addr; //set in M2DAGNOS_builiding_queue_proc
	const linkaddr_t *current_working_node_addr = &linkaddr_node_addr;
	static struct etimer t_collect;
	static struct etimer t_sleep;
	static struct etimer t_work;
	static struct etimer t_start;
	static struct etimer t_event;
	static struct etimer t_sigma;
	static struct etimer p_report_sink;
	static struct etimer t_wait;
	static struct etimer p_exe_ped;
	static struct etimer p_sampling;
	uint8_t flag_dataset = ON; //OFF: Only CMnt
	uint8_t flag_sim = ON;

	// Current node in Grid #0 & Tier#0 (Base Station (sink))
	uint8_t nlt_grid_id = 0; // Grid_id = 0 --> BS grid.
	uint8_t nlt_tier_id = 0; // Tier_id = 0 --> BS tier.
	uint8_t nlt_status = M2DAGNOS_STATUS_WAIT_REPORT;
	uint8_t nlt_state = M2DAGNOS_ST_WORKING; //Always-on

	// Process event allocation
	ev_sink = process_alloc_event();
	ev_init_rime = process_alloc_event();
	ev_initializing = process_alloc_event();
	ev_sleeping = process_alloc_event();
	ev_working = process_alloc_event();
	ev_assisting = process_alloc_event();
	ev_init_m2dagnos = process_alloc_event();
	ev_monitoring = process_alloc_event();
	ev_sampling = process_alloc_event();

	// Set the Authority level for Implicit Network Time Synchronization - timesynch file
	unsigned short id = linkaddr_node_addr.u8[0];
	timesynch_set_authority_level(id);
	LOG_DBG("id: %d, timesynch_authority_level: %d \n", id, timesynch_authority_level());
	LOG_DBG("CLOCK_SECOND: %lu\n", CLOCK_SECOND);

	// Start powertracing, once every ten seconds
	#if FW1_ACTIVE
		if(linkaddr_node_addr.u8[0] != 1 && linkaddr_node_addr.u8[0] != 0 )
		{
			#if POWER_TRACE
				powertrace_start(CLOCK_SECOND * 10);
			#endif
		};
		// static uint8_t txpower;
		// txpower = CC2520_TXPOWER_MAX;
		// cc2520_set_txpower(txpower);
		// //printf("txpower set to %u\n", txpower);
		printf("Initial PTX! %u\n", cc2520_get_txpower());

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
	collect_get_m2dagnos_status(&tc, ON);
	collect_open(&tc, COLLECT_NODE_QUEUE_CH, COLLECT_ROUTER, &collect_call);

	// ev_init_m2dagnos (asynchronous event) is posted
	process_post(PROCESS_CURRENT(), ev_init_rime, NULL);

	while(1)
	{

	   	 PROCESS_WAIT_EVENT();

	   	 if(ev == ev_init_rime)
	   	 {
	   	 	#if FW1_ACTIVE
	   	 		// FW1_ACTIVE ID:14 sink node
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
	   	 			//Stay in the same state
	   	 			process_post(PROCESS_CURRENT(), ev_init_m2dagnos, NULL);
	   	 		};
	   	 	#else
	   	 		if(linkaddr_node_addr.u8[0] == 1 && linkaddr_node_addr.u8[1] == 0)
	   	 		{
	   	 			// Set as node in the lower tier
	       			linkaddr_copy(&c_node.current_working_node_addr, current_working_node_addr);
	        	//Transit to the Sink process
	   	 			process_start(&sinking, NULL);
					  process_post(&sinking, ev_sink, NULL);
				}
				else
				{
					// Stay in the same state
					process_post(PROCESS_CURRENT(), ev_init_m2dagnos, NULL);
				}
			#endif
		 };

		 if(ev == ev_init_m2dagnos)
		 {

		 	// Initialization variables M2DaGNoS
		 	M2DAGNOS_init_proc (&c_node, state, state_sub_process, &c_flags, grid_id, tier_id, msg_type, turn_sche, sensed_value,
					batt_level, S_curr, S_prev, p, q, Nj, node_addr, next_node_queue_addr, current_working_node_addr, node_addr,
					&c_nti, &t_collect, &t_sleep, &t_work, &t_start,
					&t_event, &t_sigma, &p_report_sink, &tc,
					&t_wait, &p_exe_ped, flag_dataset, &p_sampling, flag_sim);

			#if FW1_ACTIVE
		 		// Set node addr in the lower tier
				linkaddr_t nlt_addr;
				nlt_addr.u8[0] = 1;
				nlt_addr.u8[1] = 0;

			#else
	       		// Set node addr in the lower tier
	       		linkaddr_t nlt_addr;
	        	nlt_addr.u8[0] = 1;
	        	nlt_addr.u8[1] = 0;
			#endif

		    M2DAGNOS_update_neighbor_lower_tier_info_proc (&c_nti, nlt_grid_id, nlt_tier_id,
		                   &nlt_addr, nlt_status, nlt_state);

		    // Start other processes
		    process_start(&initializing, NULL);
		    process_start(&working, NULL);
		    process_start(&monitoring, NULL);
		    process_start(&sampling, NULL);
		    process_start(&assisting, NULL);
		    process_start(&sleeping, NULL);

			//Transit to the Node Scheduling
			process_post(&initializing, ev_initializing, NULL);
			LOG_DBG("LOG: Example-m2dagnos.c : ev_initializing - post waiting. \n");
	   	};
	}; //End of while
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

	static struct etimer et;

	while(1)
	{
		PROCESS_WAIT_EVENT();

		if(ev == ev_initializing)
		{
			if (!flag_init_st)
			{
				M2DAGNOS_set_event_timeout (&c_node);
				flag_init_st = 1;
				/* Allow some time for 1st neighbor discovery process
					for node queue building purpose */
				etimer_set(&et, M2DAGNOS_T_QUEUE_BUILD* CLOCK_SECOND);
				PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
			};

			if (c_node.state == M2DAGNOS_ST_INITIALIZING)
			{
				if(c_node.state_sub_process != M2DAGNOS_ST_WN_END)
				{
					// Node Scheduling method
					M2DAGNOS_exploring_proc (&c_node, &runicast, &broadcast);

					//Stay in the same state
					process_post(PROCESS_CURRENT(), ev_initializing, NULL);
				}
				else
				{

					#if MULTIHOP
						// Close collect app for node queue selection and open a new one for tree construction
      			collect_close(&tc);
      			collect_get_m2dagnos_status(&tc, OFF);
      			/* Allow some time for 2nd neighbor discovery process
						for routing purpose. */
						etimer_set(&et, (M2DAGNOS_T_START)* CLOCK_SECOND);
						//etimer_restart(&et);
						PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
      					collect_open(&tc, COLLECT_CH, COLLECT_ROUTER, &collect_call);
					#else
      					collect_get_m2dagnos_status(&tc, OFF);
					#endif

					// /* Allow some time for 2nd neighbor discovery process
					// for routing purpose. */
					// etimer_set(&et, (M2DAGNOS_T_START)* CLOCK_SECOND);
					// //etimer_restart(&et);
					// PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

					// Node Scheduling method
					M2DAGNOS_exploring_proc (&c_node, &runicast, &broadcast);

					//Stay in the same state
					process_post(PROCESS_CURRENT(), ev_initializing, NULL);
				};
			}
			else if (c_node.state == M2DAGNOS_ST_SLEEPING)
			{
				//#if FW1_ACTIVE
				#if WURx
					static uint8_t txpower;
					txpower = PTX_LONG_OPR;
					cc2520_set_txpower(txpower);
					printf("Init txpower set to %u\n", txpower);
				#endif

				// Transit to the Sleeping state
				process_start(&send_out_msg, NULL);
				process_post(&sleeping, ev_sleeping, NULL);
				LOG_DBG("LOG: Example-m2dagnos.c : ev_initializing - post sleeping. \n");

			}
			else if (c_node.state == M2DAGNOS_ST_WORKING)
			{
				//#if FW1_ACTIVE
				#if WURx
					static uint8_t txpower;
					txpower = PTX_LONG_OPR;
					cc2520_set_txpower(txpower);
					printf("Init txpower set to %u\n", txpower);
				#endif

				// Transit to the Working state
				process_start(&send_out_msg, NULL);
				process_post(&working, ev_working, NULL);
				LOG_DBG("LOG: Example-m2dagnos.c : ev_initializing - post working. \n");
			};
		};
	};//End of while
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
			//PROCESS_WAIT_EVENT_UNTIL(ev == ev_working);

	    if(ev == ev_working)
	    {
		    if (c_node.state == M2DAGNOS_ST_WORKING)
	    	{
	    		// Call Node Scheduling and partial Data Gathering methods
		    	M2DAGNOS_sleeping_proc (&c_node, &runicast, &broadcast);
		    	M2DAGNOS_wakeup_proc (&c_node, &runicast, &broadcast);

	    		//Stay in the same state
	    		process_post(PROCESS_CURRENT(), ev_working, NULL);
	    	}
	    	else if (c_node.state == M2DAGNOS_ST_CONTINUOUS_MONITORING)
	    	{
				//Transit to Continuous Reporting state
				process_post(&monitoring, ev_monitoring, NULL);
				LOG_DBG("LOG: Example-m2dagnos.c : ev_working - post monitoring. \n");
			}
			else if (c_node.state == M2DAGNOS_ST_SLEEPING)
			{
				//Transit to Sleeping state
				process_post(&sleeping, ev_sleeping, NULL);
				LOG_DBG("LOG: Example-m2dagnos.c : ev_working - post sleeping. \n");

			}
			else if (c_node.state == M2DAGNOS_ST_ASSISTING)
			{
				//Transit to Assiting state
				process_post(&assisting, ev_assisting, NULL);
				LOG_DBG("LOG: Example-m2dagnos.c : ev_working - post assisting. \n");
			};
		};
	};//End of while
	PROCESS_END();
};


/*---------------------------------------------------------------------------*/

 /*
 *	Continuous Reporting state process (Data Gathering)
 *
 */

 PROCESS_THREAD(monitoring, ev, data)
 {
 	PROCESS_BEGIN();

 	while(1)
 	{
 		PROCESS_WAIT_EVENT();
		//PROCESS_WAIT_EVENT_UNTIL(ev == ev_monitoring);

 		if(ev == ev_monitoring)
 		{
			//printf("LOG: Example-m2dagnos.c : ev_monitoring - ent sampling. \n");
  			if (c_node.state == M2DAGNOS_ST_CONTINUOUS_MONITORING)
 			{
 				// Call Data Gathering method
 				M2DAGNOS_collecting_data_proc (&c_node, &c_nti, &runicast, &broadcast);

 				//Stay in the same state
 				process_post(PROCESS_CURRENT(), ev_monitoring, NULL);
 			}
			else if (c_node.state == M2DAGNOS_ST_EVENT_SAMPLING)
			{
				//Transit to Event Sampling state
				process_post(&sampling, ev_sampling, NULL);
				LOG_DBG("LOG: Example-m2dagnos.c : ev_monitoring - post sampling. \n");
			}
			else if (c_node.state == M2DAGNOS_ST_WORKING)
			{
				//Transit to Working state
				process_post(&working, ev_working, NULL);
				LOG_DBG("LOG: Example-m2dagnos.c : ev_monitoring - post working. \n");
			};
		};
	}; //End of while
	PROCESS_END();
};

/*---------------------------------------------------------------------------*/

 /*
 *	Event sampling state process
 *
 */

 PROCESS_THREAD(sampling, ev, data)
 {
 	PROCESS_BEGIN();

 	while(1)
 	{
 		//PROCESS_WAIT_EVENT();
		PROCESS_WAIT_EVENT_UNTIL(ev == ev_sampling);
 		if(ev == ev_sampling)
 		{
			//printf("LOG: Example-m2dagnos.c : ev_sampling - ent sampling. \n");
 			if (c_node.state == M2DAGNOS_ST_EVENT_SAMPLING)
 			{
 				// Call Data Gathering method
 				M2DAGNOS_collecting_data_proc (&c_node, &c_nti, &runicast, &broadcast);
 				// Available for sending assisting msg to other SNs in the grid
 				M2DAGNOS_wakeup_proc (&c_node, &runicast, &broadcast);

 				//Stay in the same state
 				process_post(PROCESS_CURRENT(), ev_sampling, NULL);
 			}
			else if (c_node.state == M2DAGNOS_ST_CONTINUOUS_MONITORING)
			{
				//Transit to Continuous Reporting state
				process_post(&monitoring, ev_monitoring, NULL);
				LOG_DBG("LOG: Example-m2dagnos.c : ev_sampling - post monitoring. \n");
			}
			else if (c_node.state == M2DAGNOS_ST_WORKING)
			{
				//Transit to Working state
				process_post(&working, ev_working, NULL);
				LOG_DBG("LOG: Example-m2dagnos.c : ev_sampling - post working. \n");
			};
		};
	}; //End of while
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

	// static uint8_t txpower;

 	while(1)
 	{
 		PROCESS_WAIT_EVENT();


 		if(ev == ev_assisting)
 		{
 			if (c_node.state == M2DAGNOS_ST_ASSISTING)
 			{
 				// Call Data Gathering method
 				M2DAGNOS_collecting_data_proc (&c_node, &c_nti, &runicast, &broadcast);
 				// Available for sending assisting msg to other SNs in the grid
 				M2DAGNOS_wakeup_proc (&c_node, &runicast, &broadcast);
 				//Stay in the same state
 				process_post(PROCESS_CURRENT(), ev_assisting, NULL);
 			}
 			else if (c_node.state == M2DAGNOS_ST_SLEEPING)
 			{
 				//Transit to Sleeping state
 				M2DAGNOS_clear_out_msg_queue (&c_node);
 				process_post(&sleeping, ev_sleeping, NULL);
 				LOG_DBG("LOG: Example-m2dagnos.c : ev_assisting - post sleeping. \n");
 			}
 			else if (c_node.state == M2DAGNOS_ST_WORKING)
 			{
 				//Transit to Working state
 				M2DAGNOS_clear_out_msg_queue (&c_node);
 				process_post(&working, ev_working, NULL);
 				LOG_DBG("LOG: Example-m2dagnos.c : ev_assisting - post working. \n");
 			};
 		};
 	}; //End of while
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

	while(1)
	{
		PROCESS_WAIT_EVENT();

		if(ev == ev_sleeping)
		{
        	if (c_node.state == M2DAGNOS_ST_SLEEPING)
        	{
        		M2DAGNOS_sleeping_proc (&c_node, &runicast, &broadcast); //check
        		M2DAGNOS_wakeup_proc (&c_node, &runicast, &broadcast);

        		//Stay in the same state
        		process_post(PROCESS_CURRENT(), ev_sleeping, NULL);
        	}
        	else if (c_node.state == M2DAGNOS_ST_WORKING)
        	{
        		//Transit to Working state
        		process_post(&working, ev_working, NULL);
        		LOG_DBG("LOG: Example-m2dagnos.c : ev_sleeping - post working. \n");
        	}
        	else if (c_node.state == M2DAGNOS_ST_ASSISTING)
        	{
        		//Transit to Assisting state
        		process_post(&assisting, ev_assisting, NULL);
        		LOG_DBG("LOG: Example-m2dagnos.c : ev_sleeping - post assiting \n");
        	};
        };
    }; //End of while
    PROCESS_END();
};

/*---------------------------------------------------------------------------*/
 /*
 *	Sending packet process
 *
 */

 /*FOr QoS measurement only*/
 static uint8_t n_sent_pckts = 0;

 ///ends

PROCESS_THREAD(send_out_msg, ev, data)
{
	static struct etimer et1, et2;
  	static struct M2DAGNOS_out_msg_queue *out;

  	struct M2DAGNOS_report_msg M2DAGNOS_report_msg;

	PROCESS_BEGIN();

	while(1)
	{
		//REMOVE from the list
    	//Execute periodically
    	etimer_set(&et1, CLOCK_SECOND * M2DAGNOS_TIME_OUT_MSG);
    	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et1));

    	if(c_node.state != M2DAGNOS_ST_WORKING || c_node.state != M2DAGNOS_ST_SLEEPING)
        {


        	//LOG_DBG("LOG: After c_node.state condition in send_out_msg, out_msg_queue_list: length: %d\n",
	        //    	list_length(c_node.p_out_msg_list));

        	//LOG_DBG("LOG: Before 2nd timer expired in send_out_msg, tc.sending: %d\n",
	         //   	tc.sending);

        	while(list_length(c_node.p_out_msg_list))
        	{
        		//Give enough time to transmit the previous msg
        		//etimer_set(&et2, CLOCK_SECOND * M2DAGNOS_TIME_PRE_OUT_MSG + random_rand() % (CLOCK_SECOND * M2DAGNOS_TIME_PRE_OUT_MSG));
        		etimer_set(&et2, CLOCK_SECOND * M2DAGNOS_TIME_PRE_OUT_MSG);
              	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et2));

              	if (!tc.sending)
              	{
	              	//Remove from list
	              	out = list_chop(c_node.p_out_msg_list); // Remove the last object on the list.

									/*FOr QoS measurement only*/
									n_sent_pckts++;

									if(c_node.flags->flag_PTX_changed) //Changed to MAX_PTX?
									{
										#if FW1_ACTIVE
											static uint8_t txpower;
											txpower = PTX_LONG_OPR;
											cc2520_set_txpower(txpower);
											printf("Send_out_msg txpower set to %u\n", txpower);
										#endif
									}

	             if (out->msg_id == M2DAGNOS_MSG_DATA_REPORT)
	      			{
	        			M2DAGNOS_report_msg.msg_id = M2DAGNOS_MSG_DATA_REPORT;
	        			//M2DAGNOS_report_msg.src_grid_status = M2DAGNOS_STATUS_COLLECTING;
	      			}
	      			else
	      			{
	        			M2DAGNOS_report_msg.msg_id = M2DAGNOS_MSG_ALARM_REPORT;
	        			//M2DAGNOS_report_msg.src_grid_status = M2DAGNOS_STATUS_ALERTING;
	      			};

	      			M2DAGNOS_report_msg.last_seqno = out->seqno;
	      			M2DAGNOS_report_msg.src_grid_id = c_node.grid_id;
	      			//M2DAGNOS_report_msg.src_tier_id = c_node.tier_id;
	      			//M2DAGNOS_report_msg.dst_tier_id = c_node.M2DAGNOS_nti->tier_id;
	      			linkaddr_copy(&M2DAGNOS_report_msg.src, c_node.node_addr);
	      			const linkaddr_t *aux = &out->dst_addr;
	      			linkaddr_copy(&M2DAGNOS_report_msg.dst, aux);
	      			M2DAGNOS_report_msg.sensed_value = out->sensed_value;
							M2DAGNOS_report_msg.timestamp = n_sent_pckts; //out->timestamp;
	      			packetbuf_copyfrom(&M2DAGNOS_report_msg, sizeof(M2DAGNOS_report_msg));

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

 	// Open a collect app to track other grids in THE m
	collect_get_m2dagnos_status(&tc, OFF);

   	#if MULTIHOP
   		collect_get_m2dagnos_status(&tc_0, OFF);
		collect_open(&tc_0, COLLECT_CH, COLLECT_ROUTER, &collect_call); //Grid 6
	#else
	#endif

	#if TRIGGERING
			trigger_init();
	#else
			//DO NOTHING
	#endif

   	#if FW1_ACTIVE
   		if(linkaddr_node_addr.u8[0] == 1 && linkaddr_node_addr.u8[1] == 0)
   		{
   			LOG_DBG("LOG: I am sink\n");

   			collect_set_sink(&tc, 1); // Real mote only one grid (the nearest one to the sink)
				//NETSTACK_MAC.off(1); //for COntikiMAC (Always on)
   			#if MULTIHOP
   				collect_set_sink(&tc_0, 1);
			#else
			#endif

   			// TURN ON ONBOARD LED - SLEEPING COLOR
   			leds_toggle(LEDS_SINK_NODE);
 		};
 	#else
 		if(linkaddr_node_addr.u8[0] == 1 && linkaddr_node_addr.u8[1] == 0)
 		{
 			LOG_DBG("LOG: I am sink\n");
 			// Set this SN as the sink for all grids in the M2WSN
 			collect_set_sink(&tc, 1);
      #if MULTIHOP
   				collect_set_sink(&tc_0, 1);
			#else
			#endif

 			//Coloring node - Cooja
 			printf("#A color=GREEN\n");
 		};
 	#endif

 	while(1)
 	{
 		PROCESS_WAIT_EVENT();
 		{
	 		#if FW1_ACTIVE
	 			//Just wait for receiving msgs from others SNs in the Network
	  		#else
	 			// static linkaddr_t oldparent;
	 			// const linkaddr_t *parent;
	 			// parent = collect_parent(&tc);

	 			// if(!linkaddr_cmp(parent, &oldparent))
	 			// {
	 			// 	if(!linkaddr_cmp(&oldparent, &linkaddr_null))
	 			// 	{
	 			// 		printf("#L %d 0\n", oldparent.u8[0]);
	 			// 	};
	 			// 	if(!linkaddr_cmp(parent, &linkaddr_null))
	 			// 	{
	 			// 		printf("#L %d 1\n", parent->u8[0]);
	 			// 	};
	 			// 	linkaddr_copy(&oldparent, parent);
	 			// };
	 		#endif
	 	};
	};  //End of while
	PROCESS_END();
};
