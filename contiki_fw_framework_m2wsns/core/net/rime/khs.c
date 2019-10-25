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
 *         Kang's hybrid node scheduling
 *
 * \ref    Kang, Y., Hu, B., Ding, Y., & Tan, J. (2014). A hybrid node scheduling
 *         approach based on energy efficient chain routing for WSN. Advances in
 *         Mechanical Engineering, 2014, 1–12. https://doi.org/10.1155/2014/254761
 *
 * \author
 *         juanmalk <juan-aranda@javeriana.edu.co>
 *                  <juan.aranda@usa.edu.co>
 * \date_last_version
 * 					27/11/2017
 */


/**
 * Note: Code based on Tree-based hop-by-hop reliable data collection, 2007 SICS
 * Author: Adam Dunkels <adam@sics.se>
 */


#include "khs.h"
#include "lib/random.h"
#include "lib/ringbuf.h"


// DEBUG
#define LOG_DEBUG 0
#if LOG_DEBUG
#define LOG_DBG(...) printf(__VA_ARGS__)
#else
#define LOG_DBG(...)
#endif

// MEMORY POOL FOR NODE QUEUE
#define MAX_KHS_NODE_QUEUE 25         //Application-context dependent
MEMB(khs_node_queue_memb, struct khs_node_queue, MAX_KHS_NODE_QUEUE);
LIST(node_queue_list); //Contiki list that holds the node in the grid

// MEMORY POOL FOR NODE QUEUE
#define MAX_KHS_OUT_MSG_QUEUE 100         //Application-context dependent
MEMB(khs_out_msg_queue_memb, struct khs_out_msg_queue, MAX_KHS_OUT_MSG_QUEUE);
LIST(out_msg_queue_list); //Contiki list that holds the node in the grid

// Ring Buffer definition
#define MAX_KHS_VALUE_QUEUE  8
struct ringbuf sensed_value_buf;

static uint8_t sensed_value_buf_data[MAX_KHS_VALUE_QUEUE];

// Auxiliar global variable for scheduling purpose
static uint8_t count_Nj = 0;

// This fuction is called after "to" timeout expired
void khs_init_proc (khs_node *c_node, uint8_t state, khs_flags *flags, uint8_t grid_id, uint8_t tier_id, uint8_t msg_type,
                     uint8_t turn_sche, uint8_t sensed_value, uint8_t batt_level, uint16_t Nj,  const linkaddr_t *node_addr,
                     const linkaddr_t *next_node_queue_addr, const linkaddr_t *current_working_node_addr,
                     khs_node_queue *node_list, khs_neighbor_tier_info *khs_nti, struct etimer *t_collect,
                     struct etimer *t_idle, struct etimer *t_sleep, struct etimer *t_work, struct etimer *t_start,
                     struct etimer *t_event, struct etimer *t_sigma, struct etimer *p_collect, struct etimer *p_report_sink,
                     struct collect_conn *collect_app, struct etimer *p_scheduling, struct etimer *t_init_sche, struct etimer *p_sampling,
                     struct etimer *t_wait, uint8_t flag_dataset, uint8_t flag_sim)
{
	// Variables and structures
	c_node->state = state;
  	c_node->flags = flags;
 	c_node->grid_id = grid_id;
	c_node->tier_id = tier_id;
	c_node->sensed_value = sensed_value;
	c_node->batt_level = batt_level;
	c_node->Nj = Nj;
	c_node->msg_type = msg_type;
	c_node->turn_sche = turn_sche;
	c_node->node_addr = node_addr;
	linkaddr_copy(&c_node->current_working_node_addr, current_working_node_addr);
	c_node->node_list = node_list;
	c_node->khs_nti = khs_nti;

	//Timers (pointers)
	c_node->t_collect = t_collect;
	c_node->t_idle = t_idle;
	c_node->t_sleep = t_sleep;
	c_node->t_work = t_work;
	c_node->t_start = t_start;
	c_node->t_event = t_event;
	c_node->t_sigma = t_sigma;
	c_node->t_init_sche = t_init_sche;
	c_node->t_wait = t_wait;

	//Periods and intervals (pointers and values)
	c_node->p_collect = p_collect;
	c_node->p_report_sink = p_report_sink;
	c_node->collect_app = collect_app;
	c_node->p_scheduling = p_scheduling;
	c_node->interval_work = KHS_M_PARAMETER * KHS_COLLECT_PER;

	uint16_t aux = KHS_TIN - (random_rand() % KHS_TIN);
  	c_node->interval_start = aux;
  	printf("%lu Tin: %u. \n", clock_time(), aux);

  	c_node->period_collect = KHS_COLLECT_PER;
	c_node->interval_collect = (0.75)*KHS_COLLECT_PER;
	c_node->interval_idle = (0.5)*KHS_COLLECT_PER;
	c_node->interval_event = KHS_EVENT_PER;
	c_node->interval_sigma = KHS_SIGMA_TIMEOUT;
	c_node->period_report_sink = KHS_DATA_REPORT_SINK_PER;
	c_node->period_scheduling = KHS_M_PARAMETER * Nj* KHS_COLLECT_PER;
    c_node->p_sampling = p_sampling;
    c_node->period_sampling = KHS_SAMPLING_PER;

    // Set collect timeout & period
    khs_set_work_timeout (c_node, KHS_M_PARAMETER);
    khs_set_sleep_timeout (c_node, KHS_M_PARAMETER);
    khs_set_collect_period (c_node);
    khs_set_collect_timeout (c_node);
  	khs_set_data_report_period (c_node);
	etimer_set(c_node->t_idle, c_node->interval_idle);
	etimer_set(c_node->t_sigma, c_node->interval_sigma);
  	etimer_set(c_node->p_sampling, c_node->period_sampling);
  	khs_set_wait_time (c_node, CLOCK_SECOND * 0.4);


	/* For simulation purpose - Scheme 1------------*/
	//etimer_set(c_node->t_init_sche,  490* CLOCK_SECOND);

	/*------------------------------------------------*/

	// Flags
	c_node->flags->flag_node_queue &= ~KHS_FLAG_QUEUE_COMPLETED;
	c_node->flags->flag_process = KHS_FLAG_WAIT_INIT;
	c_node->flags->flag_grid_alert &= ~KHS_STATUS_ALERTING;
	c_node->flags->flag_grid_collect &= ~KHS_STATUS_COLLECTING;
	c_node->flags->flag_grid_assist_msg &= ~KHS_FLAG_TX_ASSISTING_MSG;
	c_node->flags->flag_grid_join_msg_res &= ~KHS_FLAG_RX_JOIN_MSG;
	c_node->flags->flag_grid_join_msg_req = KHS_FLAG_RX_JOIN_MSG;
	c_node->flags->flag_tx_node_queue_info &= ~KHS_FLAG_QUEUE_COMPLETED;
	c_node->flags->flag_rx_node_queue_info &= ~KHS_FLAG_QUEUE_COMPLETED;
	c_node->flags->flag_set_tout_node_queue_sending = OFF;
	c_node->flags->flag_set_sigma_tout = OFF;
	c_node->flags->flag_still_rcast_msg_to_send = OFF;
	c_node->flags->flag_set_fire_dataset = flag_dataset;
	c_node->flags->flag_sim = flag_sim;

  // Lists
  list_init(out_msg_queue_list);
  memb_init(&khs_node_queue_memb);

  list_init(node_queue_list);
  memb_init(&khs_out_msg_queue_memb);

  // Ring buffer for synthetic sensed value storage
  ringbuf_init(&sensed_value_buf, sensed_value_buf_data, sizeof(sensed_value_buf_data));

  c_node->p_out_msg_list = out_msg_queue_list;
  c_node->p_out_msg_memb = &khs_out_msg_queue_memb;


};

void khs_update_neighbor_lower_tier_info_proc (khs_neighbor_tier_info *c_info, uint8_t grid_id, uint8_t tier_id,
											   const linkaddr_t *neighbor_lower_tier_addr,
											   uint8_t neighbor_lower_tier_status, uint8_t neighbor_lower_tier_state)
{
	c_info->grid_id = grid_id;
 	c_info->tier_id = tier_id;
 	linkaddr_copy(&c_info->neighbor_lower_tier_addr, neighbor_lower_tier_addr);
	c_info->neighbor_lower_tier_status = neighbor_lower_tier_status;
	c_info->neighbor_lower_tier_state = neighbor_lower_tier_state;

	LOG_DBG("LOG - khs.c: neighbor_lower_tier_addr Node %d.%d \n",
						c_info->neighbor_lower_tier_addr.u8[0], c_info->neighbor_lower_tier_addr.u8[1]);

	//LOG_DBG("LOG: khs.c : exit  khs_update_neighbor_lower_tier_info_proc. \n");
};

void khs_exploring_proc (khs_node *c_node, struct runicast_conn *rucast, struct broadcast_conn *bcast)
{
	 if(etimer_expired(c_node->t_start))
	 {

	 	if (c_node->flags->flag_process == KHS_FLAG_RX_CLAIM_MSG)
	 	{
	 		if (c_node->flags->flag_grid_join_msg_req)
	 		{
		 		// Send a join node queue message to the WN
		 		c_node->msg_type = KHS_MSG_JOIN_REQ;
		 		khs_rucast_send (c_node, rucast, &c_node->current_working_node_addr, 0,
		 						 MAX_RETRANSMISSIONS);
		 	}

	 		if (c_node->flags->flag_grid_join_msg_res)
	 		{
	 			// Transit to SLEEPING state
	 			c_node->state = KHS_ST_SLEEPING;

	 			#if FW1_ACTIVE
	 				// TURN OFF && ON ONBOARD LED - YELLOW COLOR
	 				leds_off(LEDS_REBOOT);
	 				leds_toggle(LEDS_ST_SLEEPING);
	 			#else
	 				//Coloring node - Cooja
			 		printf("#A color=#D3D3D3\n");
			 	#endif

	 			// Set initial timeout to the 1st time as working state
				etimer_set(c_node->t_sleep, c_node->interval_work*(c_node->turn_sche-1));

				// For performance evaluation PURPOSE
        		printf("%lu INIT_2_SLEEP %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);


	 			LOG_DBG("LOG - khs_exploring_proc: Node %d.%d transit to SLEEPING state (%d) in grid %d \n",
						c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);
	 		}
	 	}
	 	else
	 	{
	 		// This node claims itself as WORKING node
	 		linkaddr_copy(&c_node->current_working_node_addr, c_node->node_addr);

			LOG_DBG("LOG: Node %d.%d is the 1st WN in grid %d.\n",
					c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

			// Send probing/claim messages to each node in the grid (Aj)
			c_node->msg_type = KHS_MSG_CLAIM;
	 		khs_bcast_send (c_node, bcast);

	 		// Set as 1st at the node queue
	 		khs_builiding_queue_proc (c_node, c_node->node_addr, c_node->state);

			// Transit to WORKING state
			c_node->state = KHS_ST_WORKING;

      		// For performance evaluation PURPOSE
      		printf("%lu WN %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

			#if FW1_ACTIVE
	 			// TURN ON ONBOARD LED - RED COLOR
				leds_off(LEDS_REBOOT);
	 			leds_toggle(LEDS_ST_WORKING);
	 		#else
				//Coloring node in RED - Cooja
				printf("#A color=#ff0000\n");
			#endif

			LOG_DBG("LOG - khs_exploring_proc: Node %d.%d transit to WORKING state (%d) in grid %d\n",
						c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);
	 	}
	 }

	 //LOG_DBG("LOG: khs.c : exit  khs_exploring_proc. \n");
};

void khs_sleeping_proc (khs_node *c_node, struct runicast_conn *rucast, struct broadcast_conn *bcast)
{
	if(!c_node->flags->flag_grid_alert)
	{
		if((c_node->state == KHS_ST_WORKING) && (c_node->flags->flag_node_queue))
		{
			if(etimer_expired(c_node->t_work))
			{
		 		if (c_node->flags->flag_tx_node_queue_info)
		 		{
		 			if(c_node->flags->flag_process != KHS_FLAG_TX_WORKING_MSG) // Enter only if there is no runicast TX process
                	{
			 			// Send a WORKING message to the next node in the node queue of the grid
			 			c_node->msg_type = KHS_MSG_WORKING;

			 			printf("%lu SENT_WORK_MSG %d.%d %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1],
		                	c_node->next_node_queue_addr.u8[0], c_node->next_node_queue_addr.u8[1], c_node->grid_id);

			 			khs_rucast_send (c_node, rucast, &c_node->next_node_queue_addr, 0,
			 							 MAX_RETRANSMISSIONS);

			 			c_node->flags->flag_process = KHS_FLAG_TX_WORKING_MSG;

			 			LOG_DBG("LOG - khs_sleeping_proc: Node %d.%d in grid %d sent a KHS_MSG_WORKING to node %d.%d. \n",
							c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id,
							c_node->next_node_queue_addr.u8[0], c_node->next_node_queue_addr.u8[1]);
			 		}
			 		else
	            	{
				 		// Restart t_sleep (timeout)
        				etimer_restart(c_node->t_sleep);

	            		// Transit to SLEEPING state
	            		c_node->state = KHS_ST_SLEEPING;

	            		#if FW1_ACTIVE
	            		// TURN ON ONBOARD LED - YELLOW COLOR
	            		leds_off(LEDS_REBOOT);
	            		//leds_toggle(LEDS_ST_SLEEPING);
	            		#else
	            		//Coloring node - Cooja
	            		printf("#A color=#D3D3D3\n");
	            		leds_off(LEDS_REBOOT);
	            		//leds_on(LEDS_ST_SLEEPING);
	            		#endif

				 		// For performance evaluation PURPOSE
			            printf("%lu Working2SLEEP %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

				 		LOG_DBG("LOG - khs_sleeping_proc: Node %d.%d transit to SLEEPING state (%d) in grid %d. \n",
								c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);
				 	};
		 		}
		 		else if (khs_check_wait_time(c_node))
				{
					// Send a WORKING message to the next node in the node queue of the grid
		 			c_node->msg_type = KHS_MSG_WORKING;

		 			khs_rucast_send (c_node, rucast, &c_node->next_node_queue_addr, 0,
		 							 MAX_RETRANSMISSIONS);

		 			LOG_DBG("LOG - khs_sleeping_proc: Node %d.%d in grid %d sent node queue info to node %d.%d. \n",
						c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id,
						c_node->next_node_queue_addr.u8[0], c_node->next_node_queue_addr.u8[1]);
		 		};
			};
		}
		else
		{
			if(c_node->state == KHS_ST_SLEEPING)
			{
				//if(etimer_expired(c_node->t_sleep))
				if(etimer_expired(c_node->t_sleep) || (c_node->flags->flag_process == KHS_FLAG_RX_WORKING_MSG) )
				{
					if (c_node->flags->flag_process == KHS_FLAG_RX_WORKING_MSG)
					{
						// This node become a WORKING node
						linkaddr_copy(&c_node->current_working_node_addr, c_node->node_addr);

						// Transit to WORKING state
			 			c_node->flags->flag_grid_collect = KHS_STATUS_COLLECTING;
			 			c_node->flags->flag_process &= ~KHS_FLAG_RX_ASSISTING_MSG;
			 			c_node->flags->flag_grid_assist_msg &= ~KHS_FLAG_TX_ASSISTING_MSG;
						c_node->state = KHS_ST_WORKING;

						// Restart relevant periods
        				etimer_restart(c_node->t_work);
        				etimer_restart(c_node->t_collect);
        				etimer_restart(c_node->p_collect);

						#if FW1_ACTIVE
                		// TURN ON ONBOARD LED - RED COLOR
                		leds_off(LEDS_REBOOT);
               		  	leds_toggle(LEDS_ST_WORKING);
                		#else
                		//Coloring node in RED - Cooja
               			printf("#A color=RED\n");
               			leds_off(LEDS_REBOOT);
               			leds_on(LEDS_ST_WORKING);
                		#endif

						// For performance evaluation PURPOSE
            			printf("%lu WN %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
                		printf("%lu SLEEP2Working %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

						LOG_DBG("LOG - khs_sleeping_proc: Node %d.%d transit to WORKING state (%d) in grid %d. \n",
							c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);
					};
				};
			};
		};
	//LOG_DBG("LOG: khs.c : exit  khs_sleeping_proc. \n");
	};
};

void khs_wakeup_proc (khs_node *c_node, struct runicast_conn *rucast, struct broadcast_conn *bcast)
{
	if(c_node->flags->flag_grid_alert)
	{
		//LOG_DBG("LOG: khs.c - khs_wakeup_proc:if(c_node->flags->flag_grid_alert)\n ");
		if(c_node->state == KHS_ST_WORKING)
		{
			if(etimer_expired(c_node->t_work))
			{
				 if (c_node->flags->flag_tx_node_queue_info)
				 {
				 	if(c_node->flags->flag_process != KHS_FLAG_TX_WORKING_MSG)  // Enter only if there is no runicast TX process
				 	{
				 		// Send a WORKING message to the next node in the node queue of the grid
					 	c_node->msg_type = KHS_MSG_WORKING;
					 	printf("%lu SENT_WORK_MSG %d.%d %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1],
		                c_node->next_node_queue_addr.u8[0], c_node->next_node_queue_addr.u8[1], c_node->grid_id);

					 	khs_rucast_send (c_node, rucast, &c_node->next_node_queue_addr, 0,
					 						 MAX_RETRANSMISSIONS);

					 	c_node->flags->flag_grid_assist_msg = KHS_FLAG_TX_ASSISTING_MSG;
					 	c_node->flags->flag_process = KHS_FLAG_TX_WORKING_MSG;

					 	LOG_DBG("LOG - khs_wakeup_proc: Node %d.%d in grid %d sent a KHS_MSG_WORKING to node %d.%d. \n",
									c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id,
									c_node->next_node_queue_addr.u8[0], c_node->next_node_queue_addr.u8[1]);
				 	}
				 	else
					{

				 		// Transit to ASSISTING state
						c_node->state = KHS_ST_ASSISTING;
						c_node->flags->flag_grid_collect &= ~KHS_STATUS_COLLECTING;
						c_node->flags->flag_grid_alert = KHS_STATUS_ALERTING;
	              		c_node->flags->flag_process &= ~KHS_FLAG_RX_ASSISTING_MSG;

	              		// Restart t_sleep (timeout)
	        			etimer_restart(c_node->t_sleep);

	         			#if FW1_ACTIVE
		 					// TURN ON ONBOARD LED - BLUE COLOR
							leds_off(LEDS_REBOOT);
		 					leds_toggle(LEDS_ST_ASSISTING);
		 				#else
							//Coloring node - Cooja
					 		printf("#A color=#00ffff\n");
						#endif

					 	// For performance evaluation PURPOSE
			            printf("%lu Working2ASSIST %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

				 		LOG_DBG("LOG: khs.c - khs_wakeup_proc: Node %d.%d transit from WORKING to ASSISTING state (%d) in grid %d\n",
								c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);
				 	};
				}
				else if (khs_check_wait_time(c_node))
				{
					// Send a WORKING message to the next node in the node queue of the grid
		 			c_node->msg_type = KHS_MSG_WORKING;

		 			khs_rucast_send (c_node, rucast, &c_node->next_node_queue_addr, 0,
		 							 MAX_RETRANSMISSIONS);

		 			LOG_DBG("LOG - khs_sleeping_proc: Node %d.%d in grid %d sent node queue info to node %d.%d. \n",
						c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id,
						c_node->next_node_queue_addr.u8[0], c_node->next_node_queue_addr.u8[1]);
		 		};
			}
			else
			{
				if(c_node->flags->flag_grid_assist_msg !=KHS_FLAG_TX_ASSISTING_MSG)
				{
					// For performance evaluation PURPOSE
		            printf("%lu SENT_ASSIST_MSG %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

					// Send assisting messages to each node in the grid (Aj)
					c_node->msg_type = KHS_MSG_ASSISTING;
		 			khs_bcast_send (c_node, bcast);
		 			c_node->flags->flag_grid_assist_msg = KHS_FLAG_TX_ASSISTING_MSG;

		 			// For performance evaluation PURPOSE
	                printf("%lu SENT_ASSIST_MSG %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

		 			LOG_DBG("LOG: khs.c - khs_wakeup_proc: Node %d.%d sent ASSISTING msgs to nodes in grid %d\n",
						c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
				};
			};
		};

		if( (c_node->state == KHS_ST_SLEEPING) || (c_node->state == KHS_ST_ASSISTING))
		{
			//LOG_DBG("LOG: khs.c - khs_wakeup_proc: if( (c_node->state == KHS_ST_SLEEPING))\n");
			if(etimer_expired(c_node->t_sleep))
			{
				if (c_node->flags->flag_process == KHS_FLAG_RX_WORKING_MSG)
				{
					// This node become a WORKING node
					linkaddr_copy(&c_node->current_working_node_addr, c_node->node_addr);

	 				// Transit to WORKING state
	 				c_node->flags->flag_grid_collect = KHS_STATUS_COLLECTING;
	 				c_node->flags->flag_process &= ~KHS_FLAG_RX_ASSISTING_MSG;
	 				c_node->flags->flag_grid_alert &= ~KHS_STATUS_ALERTING;
	 				c_node->flags->flag_grid_assist_msg &= ~KHS_FLAG_TX_ASSISTING_MSG;

	 				// Restart relevant periods
        			etimer_restart(c_node->t_work);
        			etimer_restart(c_node->t_collect);
        			etimer_restart(c_node->p_collect);

		            // Transit to WORKING state
		            c_node->state = KHS_ST_WORKING;

		            #if FW1_ACTIVE
		              // TURN ON ONBOARD LED - RED COLOR
		              leds_off(LEDS_REBOOT);
		              leds_toggle(LEDS_ST_WORKING);
		            #else
		              //Coloring node - Cooja
		              printf("#A color=RED\n");
		              leds_off(LEDS_REBOOT);
		              leds_on(LEDS_ST_WORKING);
		            #endif

		            // For performance evaluation PURPOSE
		            printf("%lu WN %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
		            printf("%lu SLEEP2Working %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

		            LOG_DBG("LOG: Node %d.%d became a WN in grid %d.\n",
		            c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
				}
				else
				{
					// If t_sleep expired, the node does not transit to WOrking state until it receives a Working msg. Remains in sleep state
				 	return;
				};
			}
			else if (c_node->flags->flag_process == KHS_FLAG_RX_ASSISTING_MSG)
			{
				// Transit to ASSISTING state
				c_node->state = KHS_ST_ASSISTING;
				c_node->flags->flag_grid_collect = KHS_STATUS_COLLECTING;
				c_node->flags->flag_process &= ~KHS_FLAG_RX_ASSISTING_MSG;

				 #if FW1_ACTIVE
		        // TURN ON ONBOARD LED - BLUE COLOR
		        leds_off(LEDS_REBOOT);
		        leds_toggle(LEDS_ST_ASSISTING);
		        #else
		        //Coloring node - Cooja
		        printf("#A color=#00ffff\n");
		        leds_off(LEDS_REBOOT);
		        leds_on(LEDS_ST_ASSISTING);
		        #endif

		        // For performance evaluation PURPOSE
		        printf("%lu SLEEP2ASSIST %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

				LOG_DBG("LOG - khs_wakeup_proc: Node %d.%d transit from SLEEPING to ASSISTING state (%d) in grid %d\n",
						c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);
			};
		};
		//LOG_DBG("LOG: khs.c : exit  khs_wakeup_proc. \n");
	};
};


void khs_collecting_data_proc (khs_node *c_node, khs_neighbor_tier_info *c_tier, struct runicast_conn *rucast,
                               struct broadcast_conn *bcast)
{
	uint8_t sensed_value = 0;
	static uint8_t seqno_1;
    static uint8_t seqno_2;
    static uint8_t seqno_3;

	if ((c_node->flags->flag_node_queue) || (c_node->state == KHS_ST_ASSISTING)) //complete node queue building OR Node at Assisting state
	{
		// Data generation - Temperature values
		if(c_node->flags->flag_sim) // For simulation runs
		{
			if (khs_check_sampling (c_node))
			{
	  			if (c_node->flags->flag_set_fire_dataset)
	  			{
	  				sensed_value = khs_fire_dataset (c_node);
	  			}
	  			else
	  			{
	  				// Generate a random temperature value
	  				sensed_value = ((random_rand() % RANDOM_RAND_MAX ) / 600000000) + 25; // random temperature values= Range [0, 50] *C
	  				//sensed_value = ((random_rand() % RANDOM_RAND_MAX ) / 60) + 25; // random temperature values= Range [0, 200] *C
	  			};

	  			// Store the temperature value generated into "memory"
                c_node->sensed_value = sensed_value;
                khs_store_sensed_value_queue_proc (c_node, sensed_value);
                printf("Sensed_value = %d *C. \n", c_node->sensed_value);
	  		};
	  	}
		else
		{
	     	 // For REAL NODES
	    	#if FW1_ACTIVE
	  			// if(button_sensor.value(BUTTON_SENSOR_VALUE_TYPE_LEVEL) ==
	  			// 	BUTTON_SENSOR_PRESSED_LEVEL)
	  			// {
	  			// 	sensed_value = 100;
	  			// 	c_node->flags->flag_grid_alert = KHS_STATUS_ALERTING;
	  			// 	//c_node->flags->flag_grid_assist_msg &= ~KHS_FLAG_TX_ASSISTING_MSG;
          //
	  			// 	LOG_DBG("LOG: Event generated by pressing a the USER buton in Node %d.%d in grid %d.\n",
	  			// 			c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
	  			// };
	        // 	else
	       	// 	{
	        //   		sensed_value = ((random_rand() % RANDOM_RAND_MAX ) / 600000000) + 25; // random temperature values= Range [0, 50] *C
	  			// 	//sensed_value = ((random_rand() % RANDOM_RAND_MAX ) / 60) + 25; // random temperature values= Range [0, 200] *C
	        // 	};

			#else
				// DO NOTHING
			#endif
		};

		if (c_node->state == KHS_ST_ASSISTING)
		{

			if ((c_node->sensed_value < KHS_EVENT_TH))
			{
				if ((!c_node->flags->flag_set_sigma_tout) && (c_node->flags->flag_grid_alert))
				{
					khs_restart_sigma_timeout(c_node);
					c_node->flags->flag_set_sigma_tout = ON;
				}
				else
				{
					if (etimer_expired(c_node->t_sigma))
					{
						// Transit to SLEEPING state
	 					c_node->state = KHS_ST_SLEEPING;
	 					seqno_3 = 0;

            			#if FW1_ACTIVE
               				 // TURN ON ONBOARD LED - SLEEPING COLOR
                			leds_off(LEDS_REBOOT);
                			//leds_toggle(LEDS_ST_SLEEPING);
                		#else
                			//Coloring node - Cooja
                			printf("#A color=#D3D3D3\n");
               				leds_off(LEDS_REBOOT);
                			//leds_on(LEDS_ST_SLEEPING);
               			 #endif

                		// For performance evaluation PURPOSE
               			printf("%lu ASSIST2SLEEP %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

	 					LOG_DBG("LOG - khs_collecting_data_proc: Node %d.%d transit from ASSISTING to SLEEPING state (%d) in grid %d\n",
						c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);

						c_node->flags->flag_grid_alert &= ~KHS_STATUS_ALERTING;
						c_node->flags->flag_set_sigma_tout = OFF;
	 				}
	 				else if (c_node->flags->flag_process == KHS_FLAG_RX_WORKING_MSG)
					{
						// Transit to KHS_ST_SLEEPING state in order to transit to WORKING state
						c_node->state = KHS_ST_SLEEPING;

						// For performance evaluation PURPOSE
               			printf("%lu ASSIST2SLEEP %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

						LOG_DBG("LOG - khs_wakeup_proc: Node %d.%d transit from  ASSISTING to WORKING state (%d) in grid %d\n",
								c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);
					};
				};
			}
			else
			{
				// Trace Event: Send an alarm report message to its neighbor in the lower tier
				c_node->flags->flag_grid_alert = KHS_STATUS_ALERTING;
				c_node->msg_type = KHS_MSG_DATA_REPORT; //not a MSG_ALARM_REPORT DUE TO WN transmits that MSG
				update_parent_collect_app (c_node);

				if (seqno_3 == 0)
		 		{
		 			etimer_stop(c_node->p_report_sink);
		 		};

				if(etimer_expired(c_node->p_report_sink))
                {
                	seqno_3++;

					// For performance evaluation PURPOSE
					printf("%lu SENT_ASSIST_DATA_REPORT_MSG %d.%d %d %d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1],
	                     c_node->grid_id, c_node->sensed_value, seqno_3);

			 		khs_rucast_send (c_node, rucast, &c_tier->neighbor_lower_tier_addr, seqno_3,
			 							 MAX_RETRANSMISSIONS);

			 		LOG_DBG("LOG: Assistant node %d.%d sent a DATA REPORT [%u *C] message in grid %d to Node %d.%d in tier-grid [%d-%d] \n",
							c_node->node_addr->u8[0], c_node->node_addr->u8[1], sensed_value, c_node->grid_id,
							c_tier->neighbor_lower_tier_addr.u8[0], c_tier->neighbor_lower_tier_addr.u8[1],
								c_tier->tier_id, c_tier->grid_id);

			 	    //Restart period according the desired data reporting frequency
                    etimer_restart(c_node->p_report_sink);
                };
			};
		};

		if (c_node->state == KHS_ST_WORKING)
		{
			if (c_node->sensed_value >= KHS_EVENT_TH)
			{
				c_node->flags->flag_grid_alert = KHS_STATUS_ALERTING;
		 		c_node->msg_type = KHS_MSG_ALARM_REPORT;
		 		update_parent_collect_app (c_node);

		 		if (seqno_1 == 0)
		 		{
		 			etimer_stop(c_node->p_report_sink);
		 		};

		 		// Send a event data report message to its neighbor in the lower tier
                if (etimer_expired(c_node->p_report_sink))
                {
			 		seqno_2 = 0;

	                seqno_1++;

	                // For performance evaluation PURPOSE SENT_EVENT_REPORT_MSG
	                printf(" E %lu %d.%d %d %d %d S %d.%d \n", clock_time(),  c_node->node_addr->u8[0], c_node->node_addr->u8[1],
	                    	c_node->grid_id, c_node->sensed_value, seqno_1,
	                    	c_tier->neighbor_lower_tier_addr.u8[0], c_tier->neighbor_lower_tier_addr.u8[1]);

			 		khs_rucast_send (c_node, rucast, &c_tier->neighbor_lower_tier_addr, seqno_1,
			 						 MAX_RETRANSMISSIONS);

			 		LOG_DBG("LOG: WN %d.%d sent a ALARM REPORT [%u *C] message in grid %d to Node %d.%d in tier-grid [%d-%d] \n",
							c_node->node_addr->u8[0], c_node->node_addr->u8[1], sensed_value, c_node->grid_id,
							c_tier->neighbor_lower_tier_addr.u8[0], c_tier->neighbor_lower_tier_addr.u8[1],
							c_tier->tier_id, c_tier->grid_id);

			 	   // Restart period according the desired data reporting frequency
                   etimer_restart(c_node->p_report_sink);
                };

                if(c_node->flags->flag_grid_assist_msg !=KHS_FLAG_TX_ASSISTING_MSG)
				{
					// For performance evaluation PURPOSE
		            printf("%lu SENT_ASSIST_MSG %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

					// Send assisting messages to each node in the grid (Aj)
					c_node->msg_type = KHS_MSG_ASSISTING;
		 			khs_bcast_send (c_node, bcast);
		 			c_node->flags->flag_grid_assist_msg = KHS_FLAG_TX_ASSISTING_MSG;

		 			LOG_DBG("LOG: khs.c - khs_wakeup_proc: Node %d.%d sent ASSISTING msgs to nodes in grid %d\n",
						c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
				};
			}
			else
			{
				c_node->flags->flag_grid_alert &= ~KHS_STATUS_ALERTING;
				c_node->flags->flag_grid_assist_msg &= ~KHS_FLAG_TX_ASSISTING_MSG;

				if (seqno_2 == 0)
				{
		 			etimer_stop(c_node->p_report_sink);
				};

				if(!etimer_expired(c_node->t_collect))
	 			{
					c_node->flags->flag_grid_collect = KHS_STATUS_COLLECTING;

					// Send a data report message to its neighbor in the lower tier
					if (etimer_expired(c_node->p_report_sink))
					{
						c_node->msg_type = KHS_MSG_DATA_REPORT;
						update_parent_collect_app (c_node);

						seqno_1 = 0;

                        seqno_2++;

                        // For performance evaluation PURPOSE SENT_DATA_REPORT_MSG
                        printf(" D %lu %d.%d %d %d %d S %d.%d \n", clock_time(),  c_node->node_addr->u8[0], c_node->node_addr->u8[1],
                            c_node->grid_id, c_node->sensed_value, seqno_2,
                            c_tier->neighbor_lower_tier_addr.u8[0], c_tier->neighbor_lower_tier_addr.u8[1]);

			 			khs_rucast_send (c_node, rucast, &c_tier->neighbor_lower_tier_addr, seqno_2,
			 							 MAX_RETRANSMISSIONS);

			 			LOG_DBG("LOG: WN %d.%d sent a DATA REPORT [%u *C] message in grid %d to Node %d.%d in tier-grid [%d-%d] \n",
								c_node->node_addr->u8[0], c_node->node_addr->u8[1], sensed_value, c_node->grid_id,
								c_tier->neighbor_lower_tier_addr.u8[0], c_tier->neighbor_lower_tier_addr.u8[1],
								c_tier->tier_id, c_tier->grid_id);

			 			// Restart period according the desired data reporting frequency
                        etimer_restart(c_node->p_report_sink);
					};
				}
				else
				{
					c_node->flags->flag_grid_collect &= ~KHS_STATUS_COLLECTING;
					LOG_DBG("LOG: WN %d.%d  in grid %d is NOT COLLECTING data. \n",
							c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
				};
			};
		};
	};

	//LOG_DBG("LOG: khs.c : exit khs_collecting_data_proc. \n");
};

void khs_builiding_queue_proc (khs_node *c_node, const linkaddr_t *from, uint8_t state)
{

	khs_node_queue *n;
	//static uint16_t count_Nj;

	LOG_DBG("LOG: khs.c : enter hs_builiding_queue_proc. count_Nj = %d\n", count_Nj);

	if (!c_node->flags->flag_node_queue) // not complete node queue building
	{
		/* Based on example-neighbors.c */
		// Check if we already know this node.
		for(n = list_head(node_queue_list); n != NULL; n = list_item_next(n))
		{

		/* We break out of the loop if the address of the node matches
    	   the address of the node from which we received this
    	   join message. */
			if(linkaddr_cmp(&n->addr, from))
				break;
      }
   		 /* If n is NULL, this node was not found in our list, and we
    	 allocate a new struct node from the khs_node_queue_memb memory
    	 pool. */

   	 	if(n == NULL)
   	 	{
   			 n = memb_alloc(&khs_node_queue_memb);

    		/* If we could not allocate a new node entry, we give up. We
    	   could have reused an old node_queue entry, but we do not do this
   		   for now. */
    		if(n == NULL)
    			return;

    		count_Nj++;

    		// Initialize the fields.
	    	linkaddr_copy(&n->addr, from);
	    	n->state = state;
	    	n->sqno = count_Nj;

    		// Place the node on the node_queue_list.
    		list_add(node_queue_list, n);

    		LOG_DBG("LOG: %d.%d node has been added in the node_queue.\n",
					from->u8[0], from->u8[1]);
		}
		else
		{
			LOG_DBG("LOG: %d.%d node already exists in the node_queue.\n",
					from->u8[0], from->u8[1]);
		}

		if (count_Nj == c_node->Nj)
		{
			c_node->flags->flag_node_queue = KHS_FLAG_QUEUE_COMPLETED;
			c_node->flags->flag_grid_collect = KHS_STATUS_COLLECTING;

			LOG_DBG("LOG: Node %d.%d finished building the node queue in grid %d.\n",
				c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

			//Define next node in the node queue to be WN
			linkaddr_copy(&c_node->next_node_queue_addr, khs_node_queue_scheduling (c_node, c_node->node_addr));

			LOG_DBG("LOG: khs_builiding_queue_proc  - Next WN node addr in the queue: %d.%d in grid %d.\n",
				c_node->next_node_queue_addr.u8[0], c_node->next_node_queue_addr.u8[1], c_node->grid_id);

			// Set collecting timeout
			etimer_restart(c_node->t_work);
       		etimer_restart(c_node->t_collect);
       		etimer_restart(c_node->p_collect);

		};
	}
	else
	{

		LOG_DBG("LOG: Enter again in khs_builiding_queue_proc() : Node %d.%d finished building the node queue in grid %d.\n",
				c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
		return;
	}

	//LOG_DBG("LOG: khs.c : exit khs_builiding_queue_proc. \n");
};

const linkaddr_t* khs_node_queue_scheduling (khs_node *c_node, const linkaddr_t* current_addr)
{
	khs_node_queue *n;

	for(n = list_head(node_queue_list); n != NULL; n = list_item_next(n))
	{
		if(linkaddr_cmp(&n->addr, current_addr))
		{
			n = list_item_next(n); // next node in the node queue

			if(n == NULL) // Reach the end of the list
				n = list_head(node_queue_list); // start again - cyclical queue (list)

			while (n->state == KHS_ST_DEAD)
			{
				n = list_item_next(n); //select next alive node

				if(n == NULL) // Reach the end of the list
					n = list_head(node_queue_list); // start again - cyclical queue (list)

				if(&n->addr == current_addr) //Reach the same point that it started.
				{
					n = list_item_next(n);
					LOG_DBG("LOG: There is no more alive nodes in grid %d, which belog node %d.%d. \n",
							c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
					break;
				}
			}

			break;
		}
	}

	LOG_DBG("LOG: Set next WN [%d.%d] in node %d.%d in grid %d.\n",
			n->addr.u8[0],n->addr.u8[1], c_node->node_addr->u8[0],
			c_node->node_addr->u8[1], c_node->grid_id);

	const linkaddr_t * aux =  &n->addr;

	return aux;
};

void khs_set_sleep_timeout (khs_node *c_node, uint8_t m)
{
	uint16_t k = m * c_node->Nj;
	//c_node->interval_work = m * KHS_COLLECT_PER;
	c_node->interval_sleep = (k-1)*m*c_node->interval_work;
	//etimer_set(c_node->t_work, c_node->interval_work);
	etimer_set(c_node->t_sleep, c_node->interval_sleep);
	//rtimer_set(c_node->t_work, c_node->interval_work,1, NULL, NULL);
	//rtimer_set(c_node->t_sleep, c_node->interval_sleep,1, NULL, NULL);

	LOG_DBG("LOG: Set sleep timeout to Node %d.%d in grid %d.\n",
			c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
};

void khs_set_work_timeout (khs_node *c_node, uint8_t m)
{
	c_node->interval_work = m * KHS_COLLECT_PER;
	etimer_set(c_node->t_work, c_node->interval_work);

	LOG_DBG("LOG: Set work timeout to Node %d.%d in grid %d.\n",
			c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
};

void khs_restart_start_timeout (khs_node *c_node)
{
	etimer_restart(c_node->t_start);
	LOG_DBG("LOG: Restar start timeout in Node %d.%d in grid %d.\n",
			c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
};

void khs_set_event_timeout (khs_node *c_node)
{
	etimer_set(c_node->t_event, c_node->interval_event);
	LOG_DBG("LOG: Set event timeout to Node %d.%d in grid %d.\n",
			c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
};

void khs_restart_event_timeout (khs_node *c_node)
{
	etimer_restart(c_node->t_event);
};

void khs_restart_sigma_timeout (khs_node *c_node)
{
	etimer_restart(c_node->t_sigma);
};

void khs_set_collect_timeout (khs_node *c_node)
{
	etimer_set(c_node->t_collect, c_node->interval_collect);
	LOG_DBG("LOG: Set collect timeout to Node %d.%d in grid %d.\n",
			c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

};
void khs_restart_collect_timeout (khs_node *c_node)
{
	LOG_DBG("LOG - khs.c : enter khs_restart_collect_timeout \n");
	etimer_restart(c_node->t_collect);
};

void khs_restart_idle_timeout (khs_node *c_node)
{
	etimer_restart(c_node->t_idle);
};

void khs_set_collect_period (khs_node *c_node)
{
	etimer_set(c_node->p_collect, c_node->period_collect);
	LOG_DBG("LOG: Set collect period to Node %d.%d in grid %d.\n",
			c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
};

uint8_t khs_check_collect_period (khs_node *c_node)
{
	if (etimer_expired(c_node->p_collect))
	{
		etimer_restart(c_node->p_collect);
		return 1;
	}
	else
	{
		return 0;
	}
};

void khs_set_data_report_period (khs_node *c_node)
{

	etimer_set(c_node->p_report_sink, c_node->period_report_sink);
	LOG_DBG("LOG: Set report tp sink period to Node %d.%d in grid %d.\n",
			c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
};

uint8_t khs_check_data_report_period (khs_node *c_node)
{
	if (etimer_expired(c_node->p_report_sink))
	{
		etimer_restart(c_node->p_report_sink);
		return 1;
	}
	else
	{
		return 0;
	}
};

uint8_t khs_check_scheduling_period (khs_node *c_node)
{
	if (etimer_expired(c_node->p_scheduling))
	{
		//etimer_restart(c_node->t_work);
		//etimer_restart(c_node->p_scheduling);
		etimer_set(c_node->p_scheduling, c_node->period_scheduling);
		LOG_DBG("LOG: khs.c : etimer_expired(c_node->p_scheduling)\n");
		return 1;
	}
	else
	{
		return 0;
	}
};

uint8_t khs_check_wait_time (khs_node *c_node)
{
	if (etimer_expired(c_node->t_wait))
	{
		etimer_restart(c_node->t_wait);
		return 1;
	}
	else
	{
		return 0;
	}
};

void khs_set_wait_time (khs_node *c_node, clock_time_t interval_wait)
{
	etimer_set(c_node->t_wait, interval_wait);
};


//for death node management
void khs_update_scheduling_period (khs_node *c_node, uint8_t m)
{
	uint16_t k = m * c_node->Nj;

	c_node->interval_sleep = (k-1)*m*c_node->interval_work;
	etimer_set(c_node->t_sleep, c_node->interval_sleep);

	c_node->period_scheduling = k* KHS_COLLECT_PER;
	etimer_set(c_node->p_scheduling , c_node->period_scheduling);

	LOG_DBG("LOG: Update sleep timeout & T_Scheduling to Node %d.%d in grid %d.\n",
			c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
};


uint8_t khs_check_sampling (khs_node *c_node)
{
	if (etimer_expired(c_node->p_sampling))
	{
		etimer_restart(c_node->p_sampling);
		return 1;
	}
	else
	{
		return 0;
	}
};

void khs_rucast_recv (khs_node *c_node, struct runicast_conn *rucast, void *msg,
					  const linkaddr_t *from, uint8_t seqno)
{
	LOG_DBG("LOG: khs.c : enter khs_rucast_recv \n");
	LOG_DBG("LOG:c_node->node_addr %d.%d sink %d.%d\n",
            c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->current_working_node_addr.u8[0], c_node->current_working_node_addr.u8[1]);

	if (((khs_working_msg*)msg)->grid_id == c_node->grid_id)
	{
		if (((khs_working_msg*)msg)->msg_id == KHS_MSG_WORKING)
		{
			// NEEDED: Check working!
			if (!c_node->flags->flag_node_queue) // not complete node queue building
			{
				if ((!c_node->flags->flag_rx_node_queue_info) && (c_node->turn_sche !=1))
				{
					if (count_Nj <= c_node->Nj)
					{

						khs_builiding_queue_proc (c_node, &((khs_working_msg*)msg)->src, ((khs_working_msg*)msg)->state);

						// For performance evaluation PURPOSE NODE QUEUE INFO SENDING
	                	printf(" I %lu %d.%d %d R %d.%d %d %d.%d \n", clock_time(),  c_node->node_addr->u8[0], c_node->node_addr->u8[1],
	                    	c_node->grid_id, from->u8[0], from->u8[1], count_Nj,
	                    	((khs_working_msg*)msg)->src.u8[0],((khs_working_msg*)msg)->src.u8[1]);

						if (c_node->flags->flag_node_queue)
						{
							c_node->flags->flag_node_queue = KHS_FLAG_QUEUE_COMPLETED;

							// Set flag in RX_WORKING_MSG
							c_node->flags->flag_process = KHS_FLAG_RX_WORKING_MSG;

							c_node->flags->flag_rx_node_queue_info = KHS_FLAG_QUEUE_COMPLETED;

							if(c_node->turn_sche == c_node->Nj) //last node in the node_queue
								c_node->flags->flag_tx_node_queue_info = KHS_FLAG_QUEUE_COMPLETED; //for manage purpose

							//For evaluation purpose
							printf("%lu RECV_WORK_MSG %d.%d %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1],
			 				from->u8[0], from->u8[1], c_node->grid_id);

							LOG_DBG("LOG: Redundant node %d.%d finished building the node queue in grid %d.\n",
								c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
						};
					};
				};
			}
			else
			{
				// Set flag in RX_WORKING_MSG
				c_node->flags->flag_process = KHS_FLAG_RX_WORKING_MSG;

				//For evaluation purpose
				printf("%lu RECV_WORK_MSG %d.%d %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1],
			 	from->u8[0], from->u8[1], c_node->grid_id);

				// Print the working message received (it's received only if t_sleep has expired)
				LOG_DBG("LOG: %d.%d: rucast_WORKING_received (%d) from node %d.%d in grid %d\n",
				c_node->node_addr->u8[0], c_node->node_addr->u8[1], ((khs_working_msg*)msg)->msg_id,
				from->u8[0], from->u8[1], ((khs_working_msg*)msg)->grid_id);
			};

		}
		else if (((khs_join_msg*)msg)->msg_id == KHS_MSG_JOIN_REQ)
		{
			// Set flag in RX_JOIN_MSG
			c_node->flags->flag_process = KHS_FLAG_RX_JOIN_MSG;

			linkaddr_t n_addr; // Sender node address
			linkaddr_copy(&n_addr, from);

			// Print the claim message received
			LOG_DBG("LOG: %d.%d: rucast_JOIN_MSG_REQ_received (%d) from node %d.%d in grid %d\n",
					c_node->node_addr->u8[0], c_node->node_addr->u8[1], ((khs_join_msg*)msg)->msg_id,
					from->u8[0], from->u8[1], ((khs_join_msg*)msg)->grid_id);

			// Add the node to the node_queue
			khs_builiding_queue_proc (c_node, from, ((khs_join_msg*)msg)->state);

			// Send response JOIN message
			 c_node->msg_type = KHS_MSG_JOIN_RES;
			 khs_rucast_send (c_node, rucast, &n_addr, 0, 0);
		}
		else if (((khs_join_msg*)msg)->msg_id == KHS_MSG_JOIN_RES)
		{

			c_node->turn_sche = ((khs_join_msg*)msg)->turn;
			c_node->flags->flag_grid_join_msg_res = KHS_FLAG_RX_JOIN_MSG; //response
			// Print the claim message received
			LOG_DBG("LOG: %d.%d: rucast_JOIN_MSG_RES_received (%d) from node %d.%d in grid %d- TURN: %d\n",
					c_node->node_addr->u8[0], c_node->node_addr->u8[1], ((khs_join_msg*)msg)->msg_id,
					from->u8[0], from->u8[1], ((khs_join_msg*)msg)->grid_id,((khs_join_msg*)msg)->turn);

			//c_node->flags->flag_tx_node_queue_info = KHS_FLAG_QUEUE_COMPLETED; //set not to send node_queue_info
		}
		else
		{
			//Discard message.
			LOG_DBG("LOG: A runist_recv msg discarded!\n");
			return;
		};
	}
	else if(linkaddr_cmp(&c_node->current_working_node_addr, c_node->node_addr))
	{
		if (((khs_report_msg*)msg)->msg_id == KHS_MSG_DATA_REPORT)
		{
			// For performance evaluation PURPOSE (src_addr, src_grid_id, sensed_value)RECV_DATA_REPORT_MSG
			printf(" D %lu %d.%d %d %d %d R %d.%d \n", clock_time(),((uint8_t *)msg)[4], ((uint8_t *)msg)[5],
			  		((uint8_t *)msg)[1],((uint8_t *)msg)[10], ((uint8_t *)msg)[9],
			  		c_node->node_addr->u8[0], c_node->node_addr->u8[1]);
		}
		else if (((khs_report_msg*)msg)->msg_id == KHS_MSG_ALARM_REPORT)
		{
			// For performance evaluation PURPOSE RECV_EVENT_REPORT_MSG
			printf(" E %lu %d.%d %d %d %d R %d.%d \n", clock_time(),((uint8_t *)msg)[4], ((uint8_t *)msg)[5],
			  		((uint8_t *)msg)[1],((uint8_t *)msg)[10], ((uint8_t *)msg)[9],
			  		c_node->node_addr->u8[0], c_node->node_addr->u8[1]);
		};
	};
};

void khs_rucast_send (khs_node *c_node, struct runicast_conn *rucast, const linkaddr_t *to, uint8_t seq_no,
	uint8_t retransmissions)
{

	LOG_DBG("LOG: khs.c : enter khs_rucast_send - msg_type = %d\n", c_node->msg_type);

	khs_working_msg khs_working_msg;
	khs_join_msg khs_join_msg;
	khs_node_queue *n;

  static uint8_t seqno_1;
  static uint8_t seqno_2;
  static uint8_t seqno_0;

  if (c_node->msg_type == KHS_MSG_DATA_REPORT || c_node->msg_type == KHS_MSG_ALARM_REPORT)
  {
      if (c_node->msg_type == KHS_MSG_DATA_REPORT)
      {
          seqno_2 = 0; //reset seqno
          //if (seqno_1 > 255)
          //    seqno_1 = 0;

          seqno_1++;
          seqno_0 = seqno_1;
      }
      else
      {
          seqno_1 = 0; //reset seqno
          //if (seqno_2 > 255)
          //    seqno_2 = 0;

          seqno_2++;
          seqno_0 = seqno_2;
      };

      khs_store_msg (c_node, c_node->msg_type, to, seqno_0);
  };

	if(!runicast_is_transmitting(rucast)) // Enter only if there is no runicast TX process
	{
    	// Constructing a reliable unicast message
    	if (c_node->msg_type == KHS_MSG_JOIN_REQ)
    	{
        khs_join_msg.msg_id = KHS_MSG_JOIN_REQ;
        khs_join_msg.grid_id = c_node->grid_id;
        khs_join_msg.state = c_node->state;
        khs_join_msg.src = c_node->node_addr;
        khs_join_msg.last_seqno = seq_no;
        khs_join_msg.turn = -1;
        packetbuf_copyfrom(&khs_join_msg, sizeof(khs_join_msg));
        c_node->flags->flag_grid_join_msg_req &= ~KHS_FLAG_RX_JOIN_MSG;
      }
      else if(c_node->msg_type == KHS_MSG_JOIN_RES)
      {
        khs_join_msg.msg_id = KHS_MSG_JOIN_RES;
        khs_join_msg.grid_id = c_node->grid_id;
        khs_join_msg.state = c_node->state;
        khs_join_msg.src = c_node->node_addr;
        khs_join_msg.last_seqno = seq_no;

        for(n = list_head(node_queue_list); n != NULL; n = list_item_next(n))
        {
          if(linkaddr_cmp(&n->addr, to))
          {
            khs_join_msg.turn = n->sqno;
            break;
          };
        };

        packetbuf_copyfrom(&khs_join_msg, sizeof(khs_join_msg));
      }
      else if (c_node->msg_type == KHS_MSG_WORKING)
      {
        c_node->flags->flag_process &= ~KHS_FLAG_RX_WORKING_MSG;

        if ((!c_node->flags->flag_tx_node_queue_info) && (c_node->turn_sche != c_node->Nj))
        {
          khs_node_queue *n;
          uint8_t seqno = 0;
			    static uint8_t nj = 0;

          for(n = list_head(node_queue_list); n != NULL; n = list_item_next(n))
          {
            if((!runicast_is_transmitting(rucast)) && (seqno==nj))
            {
              khs_working_msg.msg_id = KHS_MSG_WORKING;
              khs_working_msg.grid_id = c_node->grid_id;
              khs_working_msg.state = n->state;
              const linkaddr_t * aux = &n->addr;
              linkaddr_copy(&khs_working_msg.src, aux);
              khs_working_msg.dst = to;
              khs_working_msg.last_seqno = seqno;
              packetbuf_copyfrom(&khs_working_msg, sizeof(khs_working_msg));
              nj++;

              // For performance evaluation PURPOSE NODE QUEUE INFO SENDING
              printf(" I %lu %d.%d %d S %d.%d %d %d.%d \n", clock_time(),  c_node->node_addr->u8[0], c_node->node_addr->u8[1],
                    c_node->grid_id, to->u8[0], to->u8[1], nj,
                    khs_working_msg.src.u8[0], khs_working_msg.src.u8[1]);

						  runicast_send(rucast, to, retransmissions);
              LOG_DBG("LOG: khs_rucast_send - Node %d.%d: Sent node_queue_info (typ: %u) to address %d.%d\n",
								c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->msg_type, to->u8[0], to->u8[1]);

              LOG_DBG("LOG: khs_rucast_send - Nj = %d\n", nj);

              if(nj == c_node->Nj)
              {
                c_node->flags->flag_tx_node_queue_info = KHS_FLAG_QUEUE_COMPLETED;
                break;
              };
            };
            seqno++;
          };
        }
        else
        {
          khs_working_msg.msg_id = KHS_MSG_WORKING;
       		khs_working_msg.grid_id = c_node->grid_id;
       		khs_working_msg.state = c_node->state;
       		linkaddr_copy(&khs_working_msg.src, c_node->node_addr);
          khs_working_msg.dst = to;
          khs_working_msg.last_seqno = 0;
          packetbuf_copyfrom(&khs_working_msg, sizeof(khs_working_msg));
        }
      }
    	else
    	{
        LOG_DBG("ERROR: Wrong runicast message type \n");
       	//Discard message.
			  return;
      };

      runicast_send(rucast, to, retransmissions);
  };
};

void khs_rucast_timedout (khs_node *c_node, struct runicast_conn *rucast, const linkaddr_t *to, uint8_t retransmissions)
{
	 LOG_DBG("runicast message timed out when sending to %d.%d, retransmissions %d\n",
	to->u8[0], to->u8[1], retransmissions);
};

void khs_bcast_recv (khs_node *c_node, struct broadcast_conn *bcast, khs_broadcast_msg *msg, const linkaddr_t *sender)
{
	LOG_DBG("LOG: khs.c : enter khs_bcast_recv - msg_type = %d &  grid_id = %d\n",
		msg->msg_id, msg->msg_id);

	if (msg->grid_id == c_node->grid_id)
	{
		if (msg->msg_id == KHS_MSG_ASSISTING)
		{
			// Set flag in RX_ASSISTING_MSG
			c_node->flags->flag_process = KHS_FLAG_RX_ASSISTING_MSG;
			c_node->flags->flag_grid_alert = KHS_STATUS_ALERTING;

			// Print the assisting message received (it's received only if t_sleep has expired)
			printf("LOG: %d.%d: bcast_ASSISTING_MSG_received (%d) from WN %d.%d in grid %d\n",
					c_node->node_addr->u8[0], c_node->node_addr->u8[1], msg->msg_id,
					sender->u8[0], sender->u8[1], msg->grid_id);
		}
		else if (msg->msg_id == KHS_MSG_CLAIM)
		{
			if(c_node->state == KHS_ST_WORKING)
			{
				printf("BREAK+2WNs\n");
			}
			else
			{
				// Set flag in RX_CLAIM_MSG
				linkaddr_copy(&c_node->current_working_node_addr, sender);

				c_node->flags->flag_process = KHS_FLAG_RX_CLAIM_MSG;

				// Print the claim message received
				printf("LOG: %d.%d: bcast_CLAIM_MSG_received (%d) from WN %d.%d in grid %d\n",
						c_node->node_addr->u8[0], c_node->node_addr->u8[1], msg->msg_id,
						sender->u8[0], sender->u8[1], msg->grid_id);
			};

		};
	}
	else
	{
		//Discard message.
		LOG_DBG("LOG: A bcast_recv msg discarded!\n");
		return;
	}
};

void khs_bcast_send (khs_node *c_node, struct broadcast_conn *bcast)
{

	khs_broadcast_msg broadcast_msg;
	//static struct etimer et;
	static uint8_t seqno;
	//static uint8_t STOP_BROADCAST;

	LOG_DBG("LOG: khs.c : enter khs_bcast_send - msg_type = %d\n", c_node->msg_type);

	// Delay 2-4 seconds (exponentially-weighted moving average)
	//etimer_set(&et, CLOCK_SECOND * 2 + random_rand() % (CLOCK_SECOND * 2));
	seqno = 0;
	//STOP_BROADCAST = c_node->Nj;

	//while(seqno <= STOP_BROADCAST)
	//{
      //  if(etimer_expired(&et))
        //{
        	// Constructing a assiting message
            if (c_node->msg_type == KHS_MSG_ASSISTING)
            {
            	if(!c_node->flags->flag_grid_assist_msg)
            	{
            		broadcast_msg.msg_id = KHS_MSG_ASSISTING;
            		c_node->flags->flag_grid_assist_msg = KHS_FLAG_TX_ASSISTING_MSG;
            		LOG_DBG("LOG: khs.c : enter khs_bcast_send - broadcast_msg.msg_id = %d\n", broadcast_msg.msg_id);
            	}
            	else
            	{
            		LOG_DBG("LOG: khs.c : A bcast msg Assisting type was already sent!\n");
            		return; // A bcast msg Assisting type was already sent.

            	}
            }
            else if (c_node->msg_type == KHS_MSG_CLAIM)
            {
            	broadcast_msg.msg_id = KHS_MSG_CLAIM;
            	//NEEDED >_<: Need the number of node in the Aj by EEC mechanims ????
            	LOG_DBG("LOG: khs.c : enter khs_bcast_send - broadcast_msg.msg_id = %d\n", broadcast_msg.msg_id);
            }
            else
            {
				LOG_DBG("ERROR: Wrong broadcast message type \n");
				//break;
				return;
			}

			broadcast_msg.grid_id = c_node->grid_id;
			broadcast_msg.src = c_node->node_addr;
			broadcast_msg.seqno = seqno;
			packetbuf_copyfrom(&broadcast_msg, sizeof(broadcast_msg));

			// Sending a assiting  msg to each neighbor in the grid (until Nj)
			broadcast_send(bcast);

			printf("LOG: KHS_bcast_send: Sending a broadcast msg (typ: %u) to each neighbor in A_%i by WN (%d.%d)\n",
					broadcast_msg.msg_id, broadcast_msg.grid_id, broadcast_msg.src->u8[0], broadcast_msg.src->u8[1]);
			seqno++;

			//if(seqno < STOP_BROADCAST) //to avoid restaring etimer after seqno == STOP_BCAST
			//	etimer_restart(&et);
		//}
	//}
};

void update_parent_collect_app (khs_node *c_node)
{
	/* For collect app (example-collect.c) */
	static linkaddr_t oldparent;
    const linkaddr_t *parent;
    parent = collect_parent(c_node->collect_app);

    if(!linkaddr_cmp(parent, &oldparent))
    {
    	if(!linkaddr_cmp(&oldparent, &linkaddr_null))
    		LOG_DBG("#L %d 0\n", oldparent.u8[0]);

    	if(!linkaddr_cmp(parent, &linkaddr_null))
         LOG_DBG("#L %d 1\n", parent->u8[0]);

        linkaddr_copy(&oldparent, parent);
    }
}

 uint8_t fire[] = {		  90,   90,  90,  95,  95,  95, 101, 101, 101, 101, 101,101,
                         100,  105, 110,  90,  90,  90,  80,  80,  80,  70, 70, 70,
                         100,  100, 100, 120, 125, 125, 131, 131,  90,  90, 90,
                          90,   95,  95,  95, 101, 101, 101, 101, 101, 101,
                         100,  105, 110,  90,  90,  90,  80,  80,  80,  70, 70, 70,
                         100,  100, 100, 120, 125, 125, 131, 131,  90,
                      	  90,   90,  90,  95,  95,  95, 101, 101, 101, 101, 101,101,
                         100,  105, 110,  90,  90,  90,  80,  80,  80,  70, 70, 70,
                         100,  100, 100, 120, 125, 125, 131, 131,  90,  90, 90,
                          90,   95,  95,  95, 101, 101, 101, 101, 101, 101,
                         100,  105, 110,  90,  90,  90,  80,  80,  80,  70, 70, 70,
                         100,  100, 100, 120, 125, 125, 131, 131,  90,
                      	  90,   90,  90,  95,  95,  95, 101, 101, 101, 101, 101,101,
                         100,  105, 110,  90,  90,  90,  80,  80,  80,  70, 70, 70,
                         100,  100, 100, 120, 125, 125, 131, 131,  90,  90, 90,
                          90,   95,  95,  95, 101, 101, 101, 101, 101, 101,
                         100,  105, 110,  90,  90,  90,  80,  80,  80,  70, 70, 70,
                         100,  100, 100, 120, 125, 125, 131, 131,  90,
                          90,   90,  90,  95,  95,  95, 101, 101, 101, 101, 101,101,
                         100,  105, 110,  90,  90,  90,  80,  80,  80,  70, 70, 70,
                         100,  100, 100, 120, 125, 125, 131, 131,  90,  90, 90,
                          90,   95,  95,  95, 101, 101, 101, 101, 101, 101,
                         100,  105, 110,  90,  90,  90,  80,  80,  80,  70, 70, 70,};


uint8_t khs_fire_dataset (khs_node *c_node)
{
	static uint8_t i= -1;
	uint8_t *pfire = fire;

	if (c_node->flags->flag_set_fire_dataset == 1)
	{
		i++;
		if (i == 255)
			i = 0;

		// LOG_DBG("LOG: KHS_.c : c_node->fire[i] = %d, %d, %d.\n",
		// 	pfire[i], i, c_node->flags->flag_set_fire_dataset);

		return pfire[i];
	}
	else
	{
		uint8_t sensed_value = ((random_rand() % RANDOM_RAND_MAX ) / 6000) + 25; // random temperature values= Range [0, 200] *C
		return sensed_value;
	}
};

void khs_store_msg (khs_node *c_node,  uint8_t msg_id, const linkaddr_t *to, uint8_t seqno)
{
    struct khs_out_msg_queue *n;

    n = memb_alloc(&khs_out_msg_queue_memb);

     if(n == NULL)
    {
        LOG_DBG("LOG: ERROR: we could not allocate a new entry for <<out_msg_queue_list)>>\n");
        return;
    }
    else
    {
        n->msg_id = msg_id;
        linkaddr_copy(&n->dst_addr, to);
        n->seqno = seqno;
        n->sensed_value = c_node->sensed_value;

         if (c_node->msg_type == KHS_MSG_DATA_REPORT)
        {
            list_push(out_msg_queue_list, n); // Add an item to the start of the list.
        }
        else
        {
            list_add(out_msg_queue_list, n); // Add an item at the end of a list.
        };


        printf("LOG: khs_store_msg : In %d.%d the packet %d (type %d) has been stored in the <<out_msg_queue_list)>> (length: %d).\n",
          c_node->node_addr->u8[0], c_node->node_addr->u8[1], seqno, msg_id, list_length(out_msg_queue_list));
    };
};

void khs_clear_out_msg_queue (khs_node *c_node)
{
    while(list_length(out_msg_queue_list))
    {
        list_pop(out_msg_queue_list);
    };

    printf("LOG: khs_clear_out_msg_queue : In %d.%d the <<out_msg_queue_list)>> has been cleaned up (length: %d).\n",
          c_node->node_addr->u8[0], c_node->node_addr->u8[1], list_length(out_msg_queue_list));
};

void khs_store_sensed_value_queue_proc (khs_node *c_node, uint8_t sensed_value)
{
    ringbuf_put(&sensed_value_buf, sensed_value);
    //LOG_DBG("LOG: hdg.c : khs_store: Store value = %d *C in Node %d.%d.\n",
    //      sensed_value, c_node->node_addr->u8[0], c_node->node_addr->u8[1]);
};

uint8_t khs_get_sensed_value_queue_proc (khs_node *c_node)
{
	uint8_t value_stored = 0;

	// Get & remove a byte from the ring buffer.
    value_stored = ringbuf_get(&sensed_value_buf);
    //printf("value_stored = %d *C. \n", value_stored);
	return value_stored;
};

/*---------------------------------------------------------------------------*/
