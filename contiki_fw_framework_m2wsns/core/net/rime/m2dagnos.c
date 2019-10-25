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
 *         MultiModal-Data Gathering and Node Scheduling Mechanism
 *
 * \author
 *         juanmalk <juan-aranda@javeriana.edu.co>
 *                  <juan.aranda@usa.edu.co>
 * \date_last_version
 *                  14/06/2018
 */


/**
 * Note: Code based on Tree-based hop-by-hop reliable data collection, 2007 SICS
 * Author: Adam Dunkels <adam@sics.se>
 */

#include "m2dagnos.h"
#include "lib/random.h"
#include "lib/ringbuf.h"
#include "dev/leds.h"
#include "trigger.h" /* the .c file should be added to the Makefile.feuerwhere
                        as CONTIKI_TARGET_SOURCEFILES      */
#include "dev/cc2520/cc2520.h"
// DEBUG
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define LOG_DBG(...) printf(__VA_ARGS__)
#else
#define LOG_DBG(...)
#endif

#define DEBUG_2 0
#if DEBUG_2
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define DEBUG_3 0
#if DEBUG_3
#include <stdio.h>
#define LOG_DBG2(...) printf(__VA_ARGS__)
#else
#define LOG_DBG2(...)
#endif

// MEMORY POOL FOR NODE QUEUE
#define MAX_M2DAGNOS_NODE_QUEUE 25         //Application-context dependent
MEMB(M2DAGNOS_node_queue_memb, struct M2DAGNOS_node_queue, MAX_M2DAGNOS_NODE_QUEUE);
LIST(node_queue_list); //Contiki list that holds the node in the grid

// MEMORY POOL FOR NODE QUEUE
#define MAX_M2DAGNOS_OUT_MSG_QUEUE 25         //Application-context dependent
MEMB(M2DAGNOS_out_msg_queue_memb, struct M2DAGNOS_out_msg_queue, MAX_M2DAGNOS_OUT_MSG_QUEUE);
LIST(out_msg_queue_list); //Contiki list that holds the node in the grid

// Ring Buffer definition
#define MAX_M2DAGNOS_VALUE_QUEUE  8
struct ringbuf sensed_value_buf;
static uint8_t sensed_value_buf_data[MAX_M2DAGNOS_VALUE_QUEUE];

// Auxiliar global variable for scheduling purpose
static uint8_t count_Nj = 0;

// This fuction is called after "to" timeout expired
void M2DAGNOS_init_proc (M2DAGNOS_node *c_node, uint8_t state, uint8_t state_sub_process, M2DAGNOS_flags *flags, uint8_t grid_id, uint8_t tier_id, uint8_t msg_type,
                     uint8_t turn_sche, uint8_t sensed_value, uint8_t batt_level, uint16_t S_curr, uint16_t S_prev, uint8_t p, uint8_t q,
                     uint16_t Nj,  const linkaddr_t *node_addr, const linkaddr_t *next_node_queue_addr, const linkaddr_t *current_working_node_addr,
                     const linkaddr_t *lower_id, M2DAGNOS_neighbor_tier_info *M2DAGNOS_nti, struct etimer *t_collect,
                     struct etimer *t_sleep, struct etimer *t_work, struct etimer *t_start,
                     struct etimer *t_event, struct etimer *t_sigma, struct etimer *p_report_sink,
                     struct collect_conn *collect_app, struct etimer *t_wait, struct etimer *p_exe_ped,
                     uint8_t flag_dataset, struct etimer *p_sampling, uint8_t flag_sim)
  {
    // Variables and structures
    c_node->state = state;
    c_node->state_sub_process = state_sub_process;
    c_node->flags = flags;
    c_node->grid_id = grid_id;
    c_node->tier_id = tier_id;
    c_node->sensed_value = sensed_value;
    c_node->S_curr = S_curr;
    c_node->S_prev = S_prev;
    c_node->p = p;
    c_node->q = q;
    c_node->batt_level = batt_level;
    c_node->Nj = Nj;
    c_node->msg_type = msg_type;
    c_node->turn_sche = turn_sche;
    c_node->node_addr = node_addr;
    linkaddr_copy(&c_node->lower_id, lower_id);
    linkaddr_copy(&c_node->next_node_queue_addr, next_node_queue_addr);
    linkaddr_copy(&c_node->current_working_node_addr, current_working_node_addr);
    c_node->M2DAGNOS_nti = M2DAGNOS_nti;
    c_node->collect_app = collect_app;

    //Timers (pointers)
    c_node->t_collect = t_collect;
    c_node->t_sleep = t_sleep;
    c_node->t_work = t_work;
    c_node->t_start = t_start;
    c_node->t_event = t_event;
    c_node->t_sigma = t_sigma;
    c_node->t_wait = t_wait;

    //Periods and intervals (pointers and values)
    c_node->p_report_sink = p_report_sink;
    c_node->p_exe_ped = p_exe_ped;
    c_node->p_sampling = p_sampling;
    c_node->period_report_sink = M2DAGNOS_DATA_REPORT_SINK_PER; //For CMnt mode
    c_node->period_exe_ped =  M2DAGNOS_EXE_PED_PER;
    c_node->period_sampling = M2DAGNOS_SAMPLING_PER;
    c_node->interval_work = M2DAGNOS_M_PARAMETER * M2DAGNOS_COLLECT_PER;
    c_node->interval_start = 1 * CLOCK_SECOND;
    //c_node->interval_collect = (0.75)*M2DAGNOS_COLLECT_PER;
    c_node->interval_collect = M2DAGNOS_M_PARAMETER * M2DAGNOS_COLLECT_PER; // same as t_work
    c_node->interval_event = M2DAGNOS_EVENT_PER;
    c_node->interval_sigma = M2DAGNOS_SIGMA_TIMEOUT;

    // Set collect timeout & period
    M2DAGNOS_set_work_timeout (c_node, M2DAGNOS_M_PARAMETER);
    M2DAGNOS_set_collect_timeout (c_node);
    M2DAGNOS_set_sleep_timeout (c_node, M2DAGNOS_M_PARAMETER);
    M2DAGNOS_set_data_report_period (c_node);
    M2DAGNOS_set_sampling_per (c_node);
    M2DAGNOS_set_ped_per (c_node);
    etimer_set(c_node->t_sigma, c_node->interval_sigma);

    // Flags
    c_node->flags->flag_node_queue &= ~M2DAGNOS_FLAG_QUEUE_COMPLETED;
    c_node->flags->flag_process = M2DAGNOS_FLAG_WAIT_INIT;
    c_node->flags->flag_grid_alert &= ~M2DAGNOS_STATUS_ALERTING;
    c_node->flags->flag_grid_collect &= ~M2DAGNOS_STATUS_COLLECTING;
    c_node->flags->flag_grid_assist_msg &= ~M2DAGNOS_FLAG_TX_ASSISTING_MSG;
    c_node->flags->flag_set_sigma_tout = OFF;
    c_node->flags->flag_set_fire_dataset = flag_dataset;
    c_node->flags->flag_sim = flag_sim;
    c_node->flags->flag_completed_sorting_node_queue = OFF;
    c_node->flags->flag_PTX_changed = OFF;

    // Lists
    list_init(out_msg_queue_list);
    memb_init(&M2DAGNOS_node_queue_memb);

    list_init(node_queue_list);
    memb_init(&M2DAGNOS_out_msg_queue_memb);

    c_node->p_out_msg_list = out_msg_queue_list;
    c_node->p_out_msg_memb = &M2DAGNOS_out_msg_queue_memb;

    // Ring buffer for synthetic sensed value storage
    ringbuf_init(&sensed_value_buf, sensed_value_buf_data, sizeof(sensed_value_buf_data));

    #if FW1_ACTIVE
    // TURN OFF ONBOARD LED
    leds_off(LEDS_REBOOT);
    #else
    leds_off(LEDS_REBOOT);
    #endif

    #if TRIGGERING
        trigger_init();
    #else
        //DO NOTHING
    #endif

  };

//TODO: Check if this function is going to be used in M2DagNos
void M2DAGNOS_update_neighbor_lower_tier_info_proc (M2DAGNOS_neighbor_tier_info *c_info, uint8_t grid_id, uint8_t tier_id,
  const linkaddr_t *neighbor_lower_tier_addr,
  uint8_t neighbor_lower_tier_status, uint8_t neighbor_lower_tier_state)
  {
    c_info->grid_id = grid_id;
    c_info->tier_id = tier_id;
    linkaddr_copy(&c_info->neighbor_lower_tier_addr, neighbor_lower_tier_addr);
    c_info->neighbor_lower_tier_status = neighbor_lower_tier_status;
    c_info->neighbor_lower_tier_state = neighbor_lower_tier_state;

    LOG_DBG("LOG - m2dagnos.c: neighbor_lower_tier_addr Node %d.%d \n",
    c_info->neighbor_lower_tier_addr.u8[0], c_info->neighbor_lower_tier_addr.u8[1]);

    //LOG_DBG("LOG: m2dagnos.c : exit  M2DAGNOS_update_neighbor_lower_tier_info_proc. \n");
  };

void M2DAGNOS_exploring_proc (M2DAGNOS_node *c_node, struct runicast_conn *rucast, struct broadcast_conn *bcast)
{

  /*
   * Working node selection process based on Contiki collect neighbor discovery app
   *
  */

  struct collect_neighbor *n;
  struct collect_neighbor_list *neighbor_list = &c_node->collect_app->neighbor_list;
  //const linkaddr_t *sink = &c_node->M2DAGNOS_nti->neighbor_lower_tier_addr;
  const linkaddr_t *aux = &c_node->lower_id;
  uint8_t neighbor_n;

  switch (c_node->state_sub_process)
  {
    case M2DAGNOS_ST_WN_INIT:

    // TODO: Future implementations
    // Transit from the INITIALIZING to EXCHANGING state
    c_node->state_sub_process = M2DAGNOS_ST_WN_EXCHANGE;

    // For performance evaluation PURPOSE
    PRINTF("%lu WN_INIT_2_WN_EXCHANGE %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

    break;

    case M2DAGNOS_ST_WN_EXCHANGE:

    neighbor_n = collect_neighbor_list_num (neighbor_list);

    if (neighbor_n >= c_node->Nj-1) //-1 : w/o considering the own node
    {
        if (collect_neighbor_list_find(neighbor_list, &c_node->M2DAGNOS_nti->neighbor_lower_tier_addr) != NULL)
        {
            // Sink is considered a neighbor, wait until the neighborhood is completed
            if (neighbor_n == c_node->Nj)
            {
                // Transit from the EXCHANGING to BUILDING state
                c_node->state_sub_process = M2DAGNOS_ST_WN_BUILDING;
                // For performance evaluation PURPOSE
                PRINTF("%lu WN_EXCHANGE_2_WN_BUILDING %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
            };
        }
        else
        {
            // Transit from the EXCHANGING to BUILDING state
            c_node->state_sub_process = M2DAGNOS_ST_WN_BUILDING;

            // For performance evaluation PURPOSE
            PRINTF("%lu WN_EXCHANGE_2_WN_BUILDING %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

        };
    };

    break;

    case M2DAGNOS_ST_WN_BUILDING:

    if (!c_node->flags->flag_node_queue)
    {
      // Coping the elements of the neighbor discovery list and the sort it
      for(n = list_head(neighbor_list->list); n != NULL; n = list_item_next(n))
      {
        M2DAGNOS_builiding_queue_proc (c_node, &n->addr, c_node->state);
      };

      // Transit from the INITIALIZING to EXCHANGING state
      c_node->state_sub_process = M2DAGNOS_ST_WN_END;

      // For performance evaluation PURPOSE
      PRINTF("%lu WN_SETTING_2_WN_END %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
    };

    break;

    case M2DAGNOS_ST_WN_END:

      if(linkaddr_cmp(c_node->node_addr, aux))
      {

        // The SN with the LOWER_ID claims itself as WORKING node
        linkaddr_copy(&c_node->current_working_node_addr, c_node->node_addr);

        LOG_DBG("LOG: Node %d.%d is the 1st WN in grid %d.\n",
        c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

        // Transit to WORKING state & Set TURN
        c_node->state = M2DAGNOS_ST_WORKING;

        //Define next node in the node queue to be WN & set turn in the node queue
        linkaddr_copy(&c_node->next_node_queue_addr, M2DAGNOS_node_queue_scheduling (c_node, c_node->node_addr));

        LOG_DBG("LOG: WN :: Position in the node_queue %d & Next WN node addr in the queue: %d.%d in grid %d.\n",
              c_node->turn_sche, c_node->next_node_queue_addr.u8[0], c_node->next_node_queue_addr.u8[1], c_node->grid_id);

        #if FW1_ACTIVE
        // TURN ON ONBOARD LED - RED COLOR
        leds_off(LEDS_REBOOT);
        leds_toggle(LEDS_ST_WORKING);
        #else
        //Coloring node in RED - Cooja
        printf("#A color=#ff0000\n");
        leds_off(LEDS_REBOOT);
        //leds_on(LEDS_ST_WORKING);
        leds_toggle(LEDS_ST_WORKING);
        #endif

        // For performance evaluation PURPOSE
        PRINTF("%lu WN %d.%d %d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->turn_sche, c_node->grid_id);
        PRINTF("%lu INIT2Working %d.%d %d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->turn_sche, c_node->grid_id);

        LOG_DBG("LOG - M2DAGNOS_exploring_proc: Node %d.%d transit to WORKING state (%d) in grid %d\n",
        c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);

        // Restart relevant periods
        etimer_restart(c_node->t_work);
        etimer_restart(c_node->t_collect);
      }
      else
      {
        // Transit to SLEEPING state
        c_node->state = M2DAGNOS_ST_SLEEPING;
        c_node->flags->flag_node_queue = M2DAGNOS_FLAG_QUEUE_COMPLETED;

        //Define next node in the node queue to be WN & set turn in the node queue
        linkaddr_copy(&c_node->next_node_queue_addr, M2DAGNOS_node_queue_scheduling (c_node, c_node->node_addr));

        LOG_DBG("LOG: Redundant SNs :: Position in the node_queue %d & Next WN node addr in the queue: %d.%d in grid %d.\n",
              c_node->turn_sche, c_node->next_node_queue_addr.u8[0], c_node->next_node_queue_addr.u8[1], c_node->grid_id);

        #if FW1_ACTIVE
        // TURN OFF && ON ONBOARD LED - YELLOW COLOR
        leds_off(LEDS_REBOOT);
        // leds_toggle(LEDS_ST_SLEEPING);
        #else
        //Coloring node - Cooja
        printf("#A color=#D3D3D3\n");
        leds_off(LEDS_REBOOT);
        //leds_on(LEDS_ST_SLEEPING);
        leds_toggle(LEDS_ST_SLEEPING);
        #endif

        // Restart t_sleep (timeout)
        etimer_restart(c_node->t_sleep);

        // For performance evaluation PURPOSE
        PRINTF("%lu INIT_2_SLEEP %d.%d %d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->turn_sche, c_node->grid_id);

        LOG_DBG("LOG - M2DAGNOS_exploring_proc: Node %d.%d transit to SLEEPING state (%d) in grid %d \n",
        c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);
      };
    break;
  };
};


void M2DAGNOS_sleeping_proc (M2DAGNOS_node *c_node, struct runicast_conn *rucast, struct broadcast_conn *bcast)
{
    if(!c_node->flags->flag_grid_alert) // Not on an emergency circumstance
      {
        if(c_node->state == M2DAGNOS_ST_WORKING)
        {
            if(etimer_expired(c_node->t_work))
            {
                // condition required to have time to post the ev_working (transit from EDR to Working to Sleeping states)
                if(c_node->flags->flag_process != M2DAGNOS_FLAG_TX_WORKING_MSG) // Enter only if there is no runicast TX process
                {
	                // Send a WORKING message to the next node in the node queue of the grid
	                c_node->msg_type = M2DAGNOS_MSG_WORKING;
                  LOG_DBG2("%lu SENT_WORK_MSG %d.%d %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1],
                        c_node->next_node_queue_addr.u8[0], c_node->next_node_queue_addr.u8[1], c_node->grid_id);
                  c_node->flags->flag_process = M2DAGNOS_FLAG_TX_WORKING_MSG;
                  M2DAGNOS_rucast_send (c_node, rucast, &c_node->next_node_queue_addr,
	                    MAX_RETRANSMISSIONS);
                    //c_node->flags->flag_process = M2DAGNOS_FLAG_TX_WORKING_MSG;
	                LOG_DBG("LOG - M2DAGNOS_sleeping_proc: Node %d.%d in grid %d sent a M2DAGNOS_MSG_WORKING to node %d.%d. \n",
	                    c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id,
	                    c_node->next_node_queue_addr.u8[0], c_node->next_node_queue_addr.u8[1]);
	            }
	            else
	            {
	            	// Transit to SLEEPING state
	                c_node->state = M2DAGNOS_ST_SLEEPING;

		            // Restart t_sleep (timeout)
        			etimer_restart(c_node->t_sleep);

	                #if FW1_ACTIVE
	                  // TURN ON ONBOARD LED - YELLOW COLOR
	                  leds_off(LEDS_REBOOT);
	                  // leds_toggle(LEDS_ST_SLEEPING);
	                #else
	                  //Coloring node - Cooja
	                  printf("#A color=#D3D3D3\n");
	                  leds_off(LEDS_REBOOT);
	                  //leds_on(LEDS_ST_SLEEPING);
                    leds_toggle(LEDS_ST_SLEEPING);
	                #endif

	                // For performance evaluation PURPOSE
	                PRINTF("%lu Working2SLEEP %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

	                LOG_DBG("LOG - M2DAGNOS_sleeping_proc: Node %d.%d transit to SLEEPING state (%d) in grid %d. \n",
	                c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);
	            };
            }
            else
            {
                // Transit to CONTINUOUS_MONITORING state
                c_node->state = M2DAGNOS_ST_CONTINUOUS_MONITORING;

                // For performance evaluation PURPOSE
                PRINTF("%lu Working2CMnt %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

                LOG_DBG("LOG - M2DAGNOS_wakeup_proc: Node %d.%d transit from WORKING to CMnt state (%d) in grid %d\n",
                c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);
            };
        }
        else
        {
            if(c_node->state == M2DAGNOS_ST_SLEEPING)
            {
                if(etimer_expired(c_node->t_sleep) || (c_node->flags->flag_process == M2DAGNOS_FLAG_RX_WORKING_MSG))
                {
                    /*
                        NEEDED??? 2nd condition if clauses:  It was set for furture WuRx implementation
                    */

                    // Wait to receive a Working message from the current Woriking node to become a WN or 2nd condition if clauses
                    if (c_node->flags->flag_process == M2DAGNOS_FLAG_RX_WORKING_MSG)
                    {
                        // Stop Sleeping timer
                        etimer_stop(c_node->t_sleep);

                        // This node become the current working node in the grid
                        linkaddr_copy(&c_node->current_working_node_addr, c_node->node_addr);

                        // Transit to WORKING state
                        c_node->flags->flag_grid_collect = M2DAGNOS_STATUS_COLLECTING;
                        c_node->flags->flag_process &= ~M2DAGNOS_FLAG_RX_ASSISTING_MSG;
                        c_node->flags->flag_grid_assist_msg &= ~M2DAGNOS_FLAG_TX_ASSISTING_MSG;
                        c_node->state = M2DAGNOS_ST_WORKING;

                        // Restart relevant periods
        				etimer_restart(c_node->t_work);
        				etimer_restart(c_node->t_collect);

                        #if FW1_ACTIVE
                        // TURN ON ONBOARD LED - RED COLOR
                        leds_off(LEDS_REBOOT);
                        leds_toggle(LEDS_ST_WORKING);
                        #else
                        //Coloring node in RED - Cooja
                        printf("#A color=RED\n");
                        leds_off(LEDS_REBOOT);
                        //leds_on(LEDS_ST_WORKING);
                        leds_toggle(LEDS_ST_WORKING);
                        #endif

                        // For performance evaluation PURPOSE
                        PRINTF("%lu WN %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
                        PRINTF("%lu Sleeping2Working %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

                        LOG_DBG("LOG - M2DAGNOS_sleeping_proc: Node %d.%d transit to WORKING state (%d) in grid %d. \n",
                        c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);
                    };
                };
            };
        };
        //LOG_DBG("LOG: m2dagnos.c : exit  M2DAGNOS_sleeping_proc. \n");
    };
};

void M2DAGNOS_wakeup_proc (M2DAGNOS_node *c_node, struct runicast_conn *rucast, struct broadcast_conn *bcast)
{
    if(c_node->flags->flag_grid_alert) // On an emergency circumstance
    {
        if(c_node->state == M2DAGNOS_ST_WORKING)
        {
            if(etimer_expired(c_node->t_work))
            {
            	// condition required to have time to post the ev_working (transit from EDR to Working to Assiting states)
                if(c_node->flags->flag_process != M2DAGNOS_FLAG_TX_WORKING_MSG)  // Enter only if there is no runicast TX process
                {
	                // Send a WORKING message to the next node in the node queue of the grid
	                c_node->msg_type = M2DAGNOS_MSG_WORKING;
                  LOG_DBG2("%lu SENT_WORK_MSG %d.%d %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1],
                        c_node->next_node_queue_addr.u8[0], c_node->next_node_queue_addr.u8[1], c_node->grid_id);
                  c_node->flags->flag_process = M2DAGNOS_FLAG_TX_WORKING_MSG;
                  M2DAGNOS_rucast_send (c_node, rucast, &c_node->next_node_queue_addr,
	                    MAX_RETRANSMISSIONS);
                    //c_node->flags->flag_process = M2DAGNOS_FLAG_TX_WORKING_MSG;
 					        LOG_DBG("LOG - M2DAGNOS_wakeup_proc-2: Node %d.%d in grid %d sent a M2DAGNOS_MSG_WORKING to node %d.%d. \n",
	                    c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id,
	                    c_node->next_node_queue_addr.u8[0], c_node->next_node_queue_addr.u8[1]);
	            }
	            else
	            {
	                // Restart t_sleep (timeout)
        			    etimer_restart(c_node->t_sleep);

	                // Transit to ASSISTING state
	                c_node->state = M2DAGNOS_ST_ASSISTING;
	                c_node->flags->flag_grid_collect &= ~M2DAGNOS_STATUS_COLLECTING;
	                c_node->flags->flag_grid_alert = M2DAGNOS_STATUS_ALERTING;
	                c_node->flags->flag_process &= ~M2DAGNOS_FLAG_RX_ASSISTING_MSG;

	                #if FW1_ACTIVE
	                  // TURN ON ONBOARD LED - BLUE COLOR
	                  leds_off(LEDS_REBOOT);
	                  leds_toggle(LEDS_ST_ASSISTING);
	                #else
	                  //Coloring node - Cooja
	                  printf("#A color=#00ffff\n");
	                  leds_off(LEDS_REBOOT);
	                  leds_on(LEDS_ST_ASSISTING);
                    //leds_toggle(LEDS_ST_WORKING);
	                #endif

	                // For performance evaluation PURPOSE
	                PRINTF("%lu Working2ASSIST %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

	                LOG_DBG("LOG: m2dagnos.c - M2DAGNOS_wakeup_proc: Node %d.%d transit from WORKING to ASSISTING state (%d) in grid %d\n",
	                c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);
	            };
            };
        }
        else if(c_node->state == M2DAGNOS_ST_EVENT_SAMPLING)
        {
        	// Send once an assisting message to other nodes in the grid
            if(!c_node->flags->flag_grid_assist_msg) //No M2DAGNOS_FLAG_TX_ASSISTING_MSG
            {
                  // Send assisting messages to each node in the grid (Aj)
                c_node->msg_type = M2DAGNOS_MSG_ASSISTING;
                M2DAGNOS_bcast_send (c_node, bcast);
                // For performance evaluation PURPOSE
                PRINTF("%lu SENT_ASSIST_MSG %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

                LOG_DBG("LOG: m2dagnos.c - M2DAGNOS_wakeup_proc: Node %d.%d sent ASSISTING msgs to nodes in grid %d\n",
                    c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
            };
        };

        if((c_node->state == M2DAGNOS_ST_SLEEPING) || (c_node->state == M2DAGNOS_ST_ASSISTING))
        {
            if(etimer_expired(c_node->t_sleep) || (c_node->flags->flag_process == M2DAGNOS_FLAG_RX_WORKING_MSG))
            //if( etimer_expired(c_node->t_sleep) )
            {
                // Wait to receive a Working message from the current Woriking node to become a WN or 2nd condition if clauses
                if (c_node->flags->flag_process == M2DAGNOS_FLAG_RX_WORKING_MSG)
                {
                    /*
                        2nd condition if clauses:  It was set for furture WuRx implementation
                    */

                    // This node become a WORKING node
                    linkaddr_copy(&c_node->current_working_node_addr, c_node->node_addr);

                    // Transit to WORKING state
                    c_node->flags->flag_grid_collect = M2DAGNOS_STATUS_COLLECTING;
                    c_node->flags->flag_process &= ~M2DAGNOS_FLAG_RX_ASSISTING_MSG;
                    c_node->flags->flag_grid_alert &= ~M2DAGNOS_STATUS_ALERTING;
                    c_node->flags->flag_grid_assist_msg &= ~M2DAGNOS_FLAG_TX_ASSISTING_MSG;


                    // Restart relevant periods
              			etimer_restart(c_node->t_work);
              			etimer_restart(c_node->t_collect);

                    // Transit to WORKING state
                    c_node->state = M2DAGNOS_ST_WORKING;

                    #if FW1_ACTIVE
                      // TURN ON ONBOARD LED - RED COLOR
                      leds_off(LEDS_REBOOT);
                      leds_toggle(LEDS_ST_WORKING);
                    #else
                      //Coloring node - Cooja
                      printf("#A color=RED\n");
                      leds_off(LEDS_REBOOT);
                      leds_toggle(LEDS_ST_WORKING);
                    #endif

                    // For performance evaluation PURPOSE
                    PRINTF("%lu WN %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
                    PRINTF("%lu SLEEP2Working %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

                    LOG_DBG("LOG: Node %d.%d became a WN in grid %d.\n",
                    c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

                }
                else
                {
                    // If t_sleep expired, the node does not transit to WOrking state until it receives a Working msg. Remains in sleep state
                    return;
                };
            }
            else if (c_node->flags->flag_process == M2DAGNOS_FLAG_RX_ASSISTING_MSG)
            {
                 // Transit to ASSISTING state
                c_node->state = M2DAGNOS_ST_ASSISTING;
                c_node->flags->flag_grid_collect = M2DAGNOS_STATUS_COLLECTING;
                c_node->flags->flag_process &= ~M2DAGNOS_FLAG_RX_ASSISTING_MSG;
                #if FW1_ACTIVE
                  // TURN ON ONBOARD LED - BLUE COLOR
                  leds_off(LEDS_REBOOT);
                  leds_toggle(LEDS_ST_ASSISTING);
                #else
                  //Coloring node - Cooja
                  printf("#A color=#00ffff\n");
                  leds_off(LEDS_REBOOT);
                  leds_on(LEDS_ST_ASSISTING);
                  //leds_toggle(LEDS_ST_WORKING);
                #endif

                // For performance evaluation PURPOSE
                PRINTF("%lu SLEEP2ASSIST %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

                LOG_DBG("LOG - M2DAGNOS_wakeup_proc: Node %d.%d transit from SLEEPING to ASSISTING state (%d) in grid %d\n",
                    c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);
            };
        };
        //LOG_DBG("LOG: m2dagnos.c : exit  M2DAGNOS_wakeup_proc. \n");
    };                      //leds_on(LEDS_ST_WORKING);
};

void M2DAGNOS_collecting_data_proc (M2DAGNOS_node *c_node, M2DAGNOS_neighbor_tier_info *c_tier, struct runicast_conn *rucast, struct broadcast_conn *bcast)
{
    uint8_t sensed_value = 0;
    static uint8_t seqno_1;
    static uint8_t seqno_2;
    rtimer_clock_t timestamp;

    // Settings & implementation of ficticial temperature values generation
    if ((c_node->flags->flag_node_queue) || (c_node->state == M2DAGNOS_ST_ASSISTING))
    {
        /*
        * Complete node queue building OR Node at Assisting state before completing the node_queue
        */

        // Synthetic data generation - Temperature values
        if(c_node->flags->flag_sim) // For simulation runs and CM5000 motes
        {
            if (M2DAGNOS_check_samping_per (c_node))
            {
                if (c_node->flags->flag_set_fire_dataset)
                {
                    sensed_value = M2DAGNOS_fire_dataset (c_node);
                }
                else
                {
                    // Generate a random temperature value
                    sensed_value = ((random_rand() % RANDOM_RAND_MAX ) / 600000000) + 25; // random temperature values= Range [0, 50] *C
                };

                // Store the temperature value generated into "memory"
                c_node->sensed_value = sensed_value;
                M2DAGNOS_store_sensed_value_queue_proc (c_node, sensed_value);

                // For performance evaluation PURPOSE
                //PRINTF("%lu Sensed_value (%d) %d.%d %d\n", clock_time(), sensed_value, c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
          };
        }
        else
        {
            // For REAL NODES
            #if FW1_ACTIVE
                // if(button_sensor.value(BUTTON_SENSOR_VALUE_TYPE_LEVEL) == BUTTON_SENSOR_PRESSED_LEVEL)
                // {
                //     sensed_value = 100; //Should be increased or decreased according to button action (e.g., once, increase; twice, decrease)
                //     c_node->flags->flag_grid_alert = M2DAGNOS_STATUS_ALERTING;
                //     c_node->flags->flag_grid_assist_msg &= ~M2DAGNOS_FLAG_TX_ASSISTING_MSG;
                //
                //     LOG_DBG("LOG: Event generated by pressing a the USER buton in Node %d.%d in grid %d.\n",
                //         c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
                // }
                // else
                // {
                //      // Generate a random temperature value
                //     sensed_value = ((random_rand() % RANDOM_RAND_MAX ) / 600000000) + 25; // random temperature values= Range [0, 50] *C
                //     //sensed_value = ((random_rand() % RANDOM_RAND_MAX ) / 60) + 25; // random temperature values= Range [0, 200] *C
                // };
            #else
                // DO NOTHING
            #endif
        };

        // States transition
        if (c_node->state == M2DAGNOS_ST_ASSISTING)
        {
            c_node->S_curr = c_node->sensed_value;

            if (c_node->S_curr < M2DAGNOS_EVENT_TH)
            {
                // Set the sigma timer during a emergency situation
                if ((!c_node->flags->flag_set_sigma_tout) && (c_node->flags->flag_grid_alert))
                {
                    M2DAGNOS_restart_sigma_timeout(c_node);
                    c_node->flags->flag_set_sigma_tout = ON;
                }
                else
                {
                    if (etimer_expired(c_node->t_sigma))
                    {
                        // Transit to SLEEPING state if sigma timer expired
                        c_node->state = M2DAGNOS_ST_SLEEPING;

                        #if FW1_ACTIVE
                            // // TURN ON ONBOARD LED - SLEEPING COLOR
                            leds_off(LEDS_REBOOT);
                        #else
                            //Coloring node - Cooja or CM5000
                            printf("#A color=#D3D3D3\n");
                            leds_off(LEDS_REBOOT);
                        #endif

                        // Reset period according the desired data reporting frequency (CMnt)
                				c_node->period_report_sink = M2DAGNOS_DATA_REPORT_SINK_PER;
                				etimer_set(c_node->p_report_sink, c_node->period_report_sink);
                				etimer_stop(c_node->p_report_sink);

                        // For performance evaluation PURPOSE
                        PRINTF("%lu ASSIST2SLEEP %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

                        LOG_DBG("LOG - M2DAGNOS_collecting_data_proc: Node %d.%d transit from ASSISTING to SLEEPING state (%d) in grid %d\n",
                            c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);

                        c_node->flags->flag_grid_alert &= ~M2DAGNOS_STATUS_ALERTING;
                        c_node->flags->flag_set_sigma_tout = OFF;
                    };
                };
            }
            else
            {

                // Trace Event: Send an alarm report message to its neighbor in the lower tier
                c_node->flags->flag_grid_alert = M2DAGNOS_STATUS_ALERTING;
                c_node->msg_type = M2DAGNOS_MSG_DATA_REPORT; //not a MSG_ALARM_REPORT DUE TO WN transmits that MSG

                if(etimer_expired(c_node->p_report_sink))
                {
                    // For performance evaluation PURPOSE
                    PRINTF("%lu SENT_ASSIST_DATA_REPORT_MSG %d.%d %d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1],
                     c_node->grid_id, c_node->S_curr);

                    M2DAGNOS_rucast_send (c_node, rucast, &c_tier->neighbor_lower_tier_addr,
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

        if(etimer_expired(c_node->t_work) && c_node->state != M2DAGNOS_ST_ASSISTING && c_node->state != M2DAGNOS_ST_SLEEPING) // to return to WORKING state
        {
            if (c_node->state == M2DAGNOS_ST_CONTINUOUS_MONITORING)
            {
                PRINTF("%lu CMnt2Working %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
                LOG_DBG("LOG - M2DAGNOS_collecting_data_proc: Node %d.%d transit from MONITORING to WORKING state (%d) in grid %d. \n",
                    c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);
            }
            else if (c_node->state == M2DAGNOS_ST_EVENT_SAMPLING)
            {
                PRINTF("%lu EDR2Working %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
                LOG_DBG("LOG - M2DAGNOS_collecting_data_proc: Node %d.%d transit from SAMPLING to WORKING state (%d) in grid %d. \n",
                    c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);

                #if FW1_ACTIVE
                    leds_off(LEDS_ST_SAMPLING);
                #else
                    leds_off(LEDS_ST_SAMPLING);
                #endif
            };

            // Transit to Working state
            c_node->state = M2DAGNOS_ST_WORKING;
        }
        else if(c_node->state != M2DAGNOS_ST_ASSISTING && c_node->state != M2DAGNOS_ST_SLEEPING)
        {
            // Run PED algorithm to decide if it is needed to switch between data-reporting modes
            M2DAGNOS_ped_proc (c_node);

            if (c_node->state == M2DAGNOS_ST_CONTINUOUS_MONITORING)
            {
                c_node->flags->flag_grid_alert &= ~M2DAGNOS_STATUS_ALERTING;

                //It is during the collecting time windows
                if(!etimer_expired(c_node->t_collect))
                {
                    c_node->flags->flag_grid_collect = M2DAGNOS_STATUS_COLLECTING;

                    // Send a data report message to its neighbor in the lower tier
                    if (etimer_expired(c_node->p_report_sink))
                    {
                        c_node->msg_type = M2DAGNOS_MSG_DATA_REPORT;
                        seqno_1 = 0;
                        //if (seqno_2 > 255)
                        //    seqno_2 = 0;

                        seqno_2++;

                        // For performance evaluation PURPOSE SENT_DATA_REPORT_MSG
                        #if TIMESYNCH_CONF_ENABLED
                          timestamp = timesynch_time();
                          //PRINTF("TIMESYNCH_CONF_ENABLED \n");
                        #else
                          timestamp = RTIMER_NOW();
                        #endif /* TIMESYNCH_CONF_ENABLED */

                        LOG_DBG2(" D %u %d.%d %d %d %d S %d.%d \n", timestamp,
                              c_node->node_addr->u8[0], c_node->node_addr->u8[1],
                              c_node->grid_id, c_node->S_curr, seqno_2,
                              c_tier->neighbor_lower_tier_addr.u8[0],
                              c_tier->neighbor_lower_tier_addr.u8[1]);

                            // #if FW1_MULTIHOPS //FOR CMnt For energy model validation purpose
                            //   #if TRIGGERING
                            //       trigger_set(); 	//send the trigger signal, high GPIO
                            //       clock_delay(WAIT_TIME); // wait for a while
                            //       trigger_clear(); 	//clear the signal, low GPIO
                            //   #else
                            //         //DO NOTHING
                            //   #endif
                            // #endif

                            M2DAGNOS_rucast_send (c_node, rucast,
                              &c_tier->neighbor_lower_tier_addr,
                              MAX_RETRANSMISSIONS);

                            LOG_DBG("LOG: WN %d.%d sent a DATA REPORT [%u *C] message in grid %d to Node %d.%d in tier-grid [%d-%d] \n",
                                  c_node->node_addr->u8[0], c_node->node_addr->u8[1],
                                  c_node->S_curr, c_node->grid_id,
                                  c_tier->neighbor_lower_tier_addr.u8[0],
                                  c_tier->neighbor_lower_tier_addr.u8[1],
                                  c_tier->tier_id, c_tier->grid_id);

                        // Restart period according the desired data reporting frequency
                        etimer_restart(c_node->p_report_sink);
                    };
                }
                else
                {
                    //End collecting period, in the next step it should transit to Sleeping
                    c_node->flags->flag_grid_collect &= ~M2DAGNOS_STATUS_COLLECTING;
                    LOG_DBG("LOG: WN %d.%d  in grid %d is NOT COLLECTING data. \n",
                        c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
                };
            }
            else if (c_node->state == M2DAGNOS_ST_EVENT_SAMPLING)
            {
                c_node->flags->flag_grid_alert = M2DAGNOS_STATUS_ALERTING;
                c_node->msg_type = M2DAGNOS_MSG_ALARM_REPORT;

                // Send a event data report message to its neighbor in the lower tier
                if (etimer_expired(c_node->p_report_sink))
                {
                  seqno_2 = 0;
                  //if (seqno_1 > 255)
                  //      seqno_1 = 0;

                	seqno_1++;
                    // For performance evaluation PURPOSE SENT_EVENT_REPORT_MSG
                    #if TIMESYNCH_CONF_ENABLED
                      timestamp = timesynch_time();
                      //PRINTF("TIMESYNCH_CONF_ENABLED \n");
                    #else
                      timestamp = RTIMER_NOW();
                    #endif /* TIMESYNCH_CONF_ENABLED */

                    LOG_DBG2(" E %u %d.%d %d %d %d S %d.%d \n", timestamp,  c_node->node_addr->u8[0], c_node->node_addr->u8[1],
                    	c_node->grid_id, c_node->S_curr, seqno_1,
                    	c_tier->neighbor_lower_tier_addr.u8[0], c_tier->neighbor_lower_tier_addr.u8[1]);

                    #if TRIGGERING
                          if (seqno_1 == 1)
                          {
                            trigger_set(); 	//send the trigger signal, high GPIO
                            clock_delay(WAIT_TIME); // wait for a while
                            trigger_clear(); 	//clear the signal, low GPIO
                          };
                    #else
                          //DO NOTHING
                    #endif

                    M2DAGNOS_rucast_send (c_node, rucast, &c_tier->neighbor_lower_tier_addr,
                        MAX_RETRANSMISSIONS);

                    LOG_DBG("LOG: WN %d.%d sent a ALARM REPORT [%u *C] message in grid %d to Node %d.%d in tier-grid [%d-%d] \n",
                        c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->S_curr, c_node->grid_id,
                        c_tier->neighbor_lower_tier_addr.u8[0], c_tier->neighbor_lower_tier_addr.u8[1],
                        c_tier->tier_id, c_tier->grid_id);

                    // Restart period according the desired data reporting frequency
                    etimer_restart(c_node->p_report_sink);
                };
            }
            else
            {
                // Error non-state transition
                LOG_DBG("LOG: WN %d.%d can not transit to any reporting mode \n",
                    c_node->node_addr->u8[0], c_node->node_addr->u8[1]);
                return;
            };
        };
    };
    //LOG_DBG("LOG: m2dagnos.c : exit M2DAGNOS_collecting_data_proc. \n");
};


void M2DAGNOS_builiding_queue_proc (M2DAGNOS_node *c_node, const linkaddr_t *from, uint8_t state)
{

  M2DAGNOS_node_queue *n;

  if (!c_node->flags->flag_completed_sorting_node_queue)
  {
    for(n = list_head(node_queue_list); n != NULL; n = list_item_next(n))
    {

    /* We break out of the loop if the address of the node matches
         the address of the node from which we received this
         join message. */
      if(linkaddr_cmp(&n->addr, from))
        break;
    };

    /* If n is NULL, this node was not found in our list, and we
    allocate a new struct node from the M2DAGNOS_node_queue_memb memory
    pool. */
    if(n == NULL)
    {
      /*Adding this SN to the node queue*/
      if (count_Nj == 0 )
      {
        n = memb_alloc(&M2DAGNOS_node_queue_memb);

        if(n == NULL)
          return;

        count_Nj++;

        // Copy info of this node
        const linkaddr_t *aux = c_node->node_addr;
        linkaddr_copy(&n->addr, aux);
        n->state = state;
        n->sqno = count_Nj;

        // Place the node on the node_queue_list.
        list_add(node_queue_list, n);

        LOG_DBG("LOG: M2DAGNOS_builiding_queue_proc : %d.%d node has been added in the node_queue in the position Nj = %d.\n",
          aux->u8[0], aux->u8[1], count_Nj);
      };

      n = memb_alloc(&M2DAGNOS_node_queue_memb);

      /* If we could not allocate a new node entry, we give up. We
       could have reused an old node_queue entry, but we do not do this
       for now. */
      if(n == NULL)
        return;

      const linkaddr_t *sink = &c_node->M2DAGNOS_nti->neighbor_lower_tier_addr;
      LOG_DBG("LOG: M2DAGNOS_builiding_queue_proc : const linkaddr_t *sink: %d.%d \n",
         sink->u8[0], sink->u8[1]);

      /*Adding neighbors to the node queue*/
      if(!linkaddr_cmp(from, sink))
      {
        count_Nj++;
        // Copy info of other nodes
        linkaddr_copy(&n->addr, from);
        n->state = state;
        n->sqno = count_Nj;
        // Place the node on the node_queue_list.
        list_add(node_queue_list, n);

        LOG_DBG("LOG: M2DAGNOS_builiding_queue_proc : %d.%d node has been added in the node_queue in the position Nj = %d.\n",
          from->u8[0], from->u8[1], count_Nj);
      };
    }
    else
    {
      LOG_DBG("LOG: %d.%d node already exists in the node_queue.\n",
          from->u8[0], from->u8[1]);
    };

    if (c_node->Nj == count_Nj)
    {
      for(n = list_head(node_queue_list); n != NULL; n = list_item_next(n))
      {
        // For performance evaluation PURPOSE
        PRINTF("w/o node_queue :: Node: %d.%d in the position %d\n", n->addr.u8[0], n->addr.u8[1], n->sqno );
      };

      /*Sorting the neighbor list for the lower to higher id_addr*/
      M2DAGNOS_node_queue *n = list_head(node_queue_list);
      M2DAGNOS_node_queue *nn = list_item_next(n);
      linkaddr_t s_1;
      linkaddr_t s_2;

      while ( n != NULL )
      {
        nn = n->next;
        while ( nn != NULL )
        {
          if(n->addr.u8[0] == nn->addr.u8[0])
          {
            if(n->addr.u8[1] > nn->addr.u8[1])
            {
              const linkaddr_t *aux = &n->addr;
              const linkaddr_t *aux_2 = &nn->addr;
              linkaddr_copy(&n->addr, aux_2);
              linkaddr_copy(&nn->addr, aux);
            };
          }
          else if(n->addr.u8[0] > nn->addr.u8[0] )
          {

            const linkaddr_t *aux_1 = &n->addr;
            linkaddr_copy(&s_1, aux_1);
            const linkaddr_t *aux_2 = &nn->addr;
            linkaddr_copy(&s_2, aux_2);
            const linkaddr_t *aux_3 = &s_1;
            linkaddr_copy(&nn->addr, aux_3);
            const linkaddr_t *aux_4 = &s_2;
            linkaddr_copy(&n->addr, aux_4);
          };
          nn = nn->next;
        };
        n = n->next;
      };
      c_node->flags->flag_completed_sorting_node_queue = ON;
    };
  };

  if (c_node->flags->flag_completed_sorting_node_queue)
  {

    for(n = list_head(node_queue_list); n != NULL; n = list_item_next(n))
    {
      // For performance evaluation PURPOSE
      PRINTF("Sorted node_queue :: Node: %d.%d in the position %d\n", n->addr.u8[0], n->addr.u8[1], n->sqno );
    };

    for(n = list_head(node_queue_list); n != NULL; n = list_item_next(n))
    {
      const linkaddr_t *aux = &n->addr;
      linkaddr_copy(&c_node->lower_id, aux);
      // For performance evaluation PURPOSE
      LOG_DBG("Set Lower_id to %d.%d in grid  %d!\n", c_node->lower_id.u8[0], c_node->lower_id.u8[1], c_node->grid_id);
      break;
    };

    //Set flag to notify the ending node queue process
    c_node->flags->flag_node_queue = M2DAGNOS_FLAG_QUEUE_COMPLETED;
    count_Nj = 0;

    LOG_DBG("LOG: Node %d.%d finished building the node queue in grid %d.\n",
      c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
  };
};


const linkaddr_t* M2DAGNOS_node_queue_scheduling (M2DAGNOS_node *c_node, const linkaddr_t* current_addr)
{
    M2DAGNOS_node_queue *n;

  uint8_t TURN = 0;

    for(n = list_head(node_queue_list); n != NULL; n = list_item_next(n))
    {
        TURN +=1;
        if(linkaddr_cmp(&n->addr, current_addr))
        {
            n = list_item_next(n); // next node in the node queue
            c_node->turn_sche = TURN; //set TURN in the node_queue

            if(n == NULL) // Reach the end of the list
                n = list_head(node_queue_list); // start again - cyclical queue (list)

            while (n->state == M2DAGNOS_ST_DEAD)
            {
                n = list_item_next(n); //select next alive node

                if(n == NULL) // Reach the end of the list
                    n = list_head(node_queue_list); // start again - cyclical queue (list)

                if(&n->addr == current_addr) //Reach the same point that it started.
                {
                    n = list_item_next(n);
                    LOG_DBG("LOG: There is no more alive nodes in grid %d, which belong node %d.%d. \n",
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

//Paramenter Event Detection Algorithm from Lee & Lim 2012
void M2DAGNOS_ped_proc (M2DAGNOS_node *c_node)
{
  if(c_node->state != M2DAGNOS_ST_ASSISTING)
  {
    if(M2DAGNOS_check_ped_per(c_node))
    {
      //Determine the current value of the threshold variables over the current time inteval
      c_node->S_prev = c_node->S_curr;
      c_node->S_curr = M2DAGNOS_calc_avg_s_curr_proc(c_node);

      if(c_node->state == M2DAGNOS_ST_CONTINUOUS_MONITORING)
      {
        if(c_node->S_curr >= M2DAGNOS_EVENT_TH)
        {
          if(c_node->S_curr >= c_node->S_prev)
          {
            c_node->p++;
            c_node->q++;

            LOG_DBG("LOG - M2DAGNOS_ped_proc: TRUE-EVENT-DRIVEN mode in node %d.%d: p = (%d) & q = (%d).\n",
            c_node->node_addr->u8[0], c_node->node_addr->u8[1],
            c_node->p, c_node->q);
          }
          else
          {
            c_node->q++;

            LOG_DBG("LOG - M2DAGNOS_ped_proc: FALSE-EVENT-DRIVEN mode in node %d.%d: p = (%d) & q = (%d).\n",
            c_node->node_addr->u8[0], c_node->node_addr->u8[1],
            c_node->p, c_node->q);
          };
        }
        else
        {
          //To eliminate the risk of transient response, the counter variables are reset
          c_node->p = 0;
          c_node->q = 0;
        };

        if ((c_node->p >= M2DAGNOS_P_START) || (c_node->q >= M2DAGNOS_Q_START))
        {
          // Transit to EVENT_SAMPLING state
          c_node->state = M2DAGNOS_ST_EVENT_SAMPLING;

          // Set period according the desired data reporting frequency
          c_node->period_report_sink = M2DAGNOS_EVENT_REPORT_SINK_PER;
          etimer_set(c_node->p_report_sink, c_node->period_report_sink);
          etimer_stop(c_node->p_report_sink);

          // For performance evaluation PURPOSE (AN EVENT HAS OCCURRED)
          PRINTF("%lu CMnt2EDR %d.%d %d %d %d\n", clock_time(),
          c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id,c_node->p, c_node->q);

          LOG_DBG("LOG - M2DAGNOS_ped_proc: Node %d.%d transit from CMnt to EDR state (%d) in grid %d\n",
          c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);

          //Set flag to enable assisting message sending
          c_node->flags->flag_grid_assist_msg &= ~M2DAGNOS_FLAG_TX_ASSISTING_MSG;

          #if FW1_ACTIVE
          leds_off(LEDS_ST_SAMPLING);
          // leds_toggle(LEDS_ST_SAMPLING);
          #else
          //leds_on(LEDS_ST_SAMPLING);
          leds_off(LEDS_ST_SAMPLING);
          leds_toggle(LEDS_ST_SAMPLING);
          #endif
        }
        else
        {
          // Stay on current state, i.e., CMnt
          return;
        };
      }
      else if(c_node->state == M2DAGNOS_ST_EVENT_SAMPLING)
      {
        if(c_node->S_curr < M2DAGNOS_EVENT_TH)
        {
          if(c_node->S_curr < c_node->S_prev)
          {
            c_node->p++;
            c_node->q++;

            LOG_DBG("LOG - M2DAGNOS_ped_proc: TRUE-TIME-DRIVEN mode in node %d.%d: p = (%d) & q = (%d).\n",
            c_node->node_addr->u8[0], c_node->node_addr->u8[1],
            c_node->p, c_node->q);
          }
          else
          {
            c_node->q++;

            LOG_DBG("LOG - M2DAGNOS_ped_proc: FALSE-TIME-DRIVEN mode in node %d.%d: p = (%d) & q = (%d).\n",
            c_node->node_addr->u8[0], c_node->node_addr->u8[1],
            c_node->p, c_node->q);
          };
        }
        else
        {
          c_node->p = 0;
          c_node->q = 0;
        };

        if ((c_node->p >= M2DAGNOS_P_STOP) || (c_node->q >= M2DAGNOS_Q_STOP))
        {
          // Transit to CONTINUOUS_MONITORING state
          c_node->state = M2DAGNOS_ST_CONTINUOUS_MONITORING;

          // Set period according the desired data reporting frequency
          c_node->period_report_sink = M2DAGNOS_DATA_REPORT_SINK_PER;
          etimer_set(c_node->p_report_sink, c_node->period_report_sink);
          etimer_stop(c_node->p_report_sink);

          // For performance evaluation PURPOSE
          PRINTF("%lu EDR2CMnt %d.%d %d %d %d\n", clock_time(),
          c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id,c_node->p, c_node->q);

          LOG_DBG("LOG - M2DAGNOS_ped_proc: Node %d.%d transit from EDR to CMnt state (%d) in grid %d\n",
          c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->state, c_node->grid_id);

          #if FW1_ACTIVE
          // TURN OFF ONBOARD LED
          leds_off(LEDS_ST_SAMPLING);
          #else
          leds_off(LEDS_ST_SAMPLING);
          #endif

        }
        else
        {
          // Stay on current state, i.e., EDR
          return;
        }
      }
      else
      {
        LOG_DBG("LOG - M2DAGNOS_ped_proc: WRONG current mode (%d) in node %d.%d.\n",
        c_node->state, c_node->node_addr->u8[0],
        c_node->node_addr->u8[1]);
        return;
      };
    }
    else
    {
      //Result of PED is indifferent in ASSISTING_state
      return;
    };
  };
};

uint16_t M2DAGNOS_calc_avg_s_curr_proc (M2DAGNOS_node *c_node)
{
  uint16_t sum = 0;
    uint16_t result = 0;
    uint8_t count_n_store_value = 0;
    uint8_t value_stored = 0;

  //while (value_stored  != -1)
  while (value_stored  != 255)
  {
    // Get & remove a byte from the ring buffer.
    value_stored = ringbuf_get(&sensed_value_buf);

    //if (value_stored  != -1)
    if (value_stored  != 255)
    {
      count_n_store_value +=1;
      sum = sum + value_stored;
      //PRINTF("LOG: M2DAGNOS_calc: Store value = %d *C [%d], sum = %d in Node %d.%d.\n",
        //value_stored, count_n_store_value, sum, c_node->node_addr->u8[0], c_node->node_addr->u8[1]);
    };
  };

  //Compute the average
  result = sum/count_n_store_value;

  // For evaluation purpose
  PRINTF("S_curr (average) = %d calculated in Node %d.%d.\n",
    result, c_node->node_addr->u8[0], c_node->node_addr->u8[1]);

  return result;
};

void M2DAGNOS_store_sensed_value_queue_proc (M2DAGNOS_node *c_node, uint8_t sensed_value)
{
    ringbuf_put(&sensed_value_buf, sensed_value);
    //LOG_DBG("LOG: hdg.c : M2DAGNOS_store: Store value = %d *C in Node %d.%d.\n",
    //      sensed_value, c_node->node_addr->u8[0], c_node->node_addr->u8[1]);
};

void M2DAGNOS_set_sleep_timeout (M2DAGNOS_node *c_node, uint8_t m)
{
    uint16_t k = m * c_node->Nj;
    c_node->interval_sleep = (k-1)*m*c_node->interval_work;
    etimer_set(c_node->t_sleep, c_node->interval_sleep);

    LOG_DBG("LOG: Set sleep timeout to Node %d.%d in grid %d.\n",
            c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
};

void M2DAGNOS_set_work_timeout (M2DAGNOS_node *c_node, uint8_t m)
{
    c_node->interval_work = m * M2DAGNOS_COLLECT_PER;
    etimer_set(c_node->t_work, c_node->interval_work);

    LOG_DBG("LOG: Set work timeout to Node %d.%d in grid %d.\n",
            c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
};

void M2DAGNOS_set_event_timeout (M2DAGNOS_node *c_node)
{
  etimer_set(c_node->t_event, c_node->interval_event);
  LOG_DBG("LOG: Set event timeout to Node %d.%d in grid %d.\n",
      c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
};

void M2DAGNOS_restart_event_timeout (M2DAGNOS_node *c_node)
{
  etimer_restart(c_node->t_event);
};

void M2DAGNOS_restart_sigma_timeout (M2DAGNOS_node *c_node)
{
    etimer_restart(c_node->t_sigma);
      LOG_DBG("LOG: Restart sigma timeout to Node %d.%d in grid %d.\n",
      c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
};

void M2DAGNOS_set_collect_timeout (M2DAGNOS_node *c_node)
{
    etimer_set(c_node->t_collect, c_node->interval_collect);
    LOG_DBG("LOG: Set collect timeout to Node %d.%d in grid %d.\n",
            c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);

};

void M2DAGNOS_set_data_report_period (M2DAGNOS_node *c_node)
{

    etimer_set(c_node->p_report_sink, c_node->period_report_sink);
    LOG_DBG("LOG: Set report tp sink period to Node %d.%d in grid %d.\n",
            c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
};

uint8_t M2DAGNOS_check_wait_time (M2DAGNOS_node *c_node)
{
  if (etimer_expired(c_node->t_wait))
    {
    LOG_DBG("LOG: m2dagnos.c : etimer_expired(c_node->t_wait)\n");
        return 1;
    }
    else
    {
        return 0;
    };
};

void M2DAGNOS_set_wait_time (M2DAGNOS_node *c_node, clock_time_t interval_wait)
{
    etimer_set(c_node->t_wait, interval_wait);
};

//for death node management
void M2DAGNOS_update_scheduling_period (M2DAGNOS_node *c_node, uint8_t m)
{
    uint16_t k = m * c_node->Nj;

    c_node->interval_sleep = (k-1)*m*c_node->interval_work;
    etimer_set(c_node->t_sleep, c_node->interval_sleep);

    LOG_DBG("LOG: Update sleep timeout to Node %d.%d in grid %d.\n",
            c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->grid_id);
};

void M2DAGNOS_set_ped_per (M2DAGNOS_node *c_node)
{
    etimer_set(c_node->p_exe_ped, c_node->period_exe_ped);
    LOG_DBG("LOG: Set PED period in Node %d.%d.\n",
            c_node->node_addr->u8[0], c_node->node_addr->u8[1]);
};

uint8_t M2DAGNOS_check_ped_per (M2DAGNOS_node *c_node)
{
    if (etimer_expired(c_node->p_exe_ped))
    {
        etimer_restart(c_node->p_exe_ped);
        return 1;
    }
    else
    {
        return 0;
    }
};

void M2DAGNOS_set_sampling_per (M2DAGNOS_node *c_node)
{
    etimer_set(c_node->p_sampling, c_node->period_sampling);
    LOG_DBG("LOG: Set Sampling period in Node %d.%d.\n",
    c_node->node_addr->u8[0], c_node->node_addr->u8[1]);
};

uint8_t M2DAGNOS_check_samping_per (M2DAGNOS_node *c_node)
{
    if (etimer_expired(c_node->p_sampling))
    {
        etimer_restart(c_node->p_sampling);
        return 1;
    }
    else
    {
        return 0;
    };
};

void M2DAGNOS_rucast_recv (M2DAGNOS_node *c_node, struct runicast_conn *rucast, void *msg,
                      const linkaddr_t *from, uint8_t seqno)
{

  rtimer_clock_t timestamp2;
  #if TIMESYNCH_CONF_ENABLED
    timestamp2 = timesynch_time();
    //PRINTF("TIMESYNCH_CONF_ENABLED \n");
  #else
    timestamp2 = RTIMER_NOW();
  #endif /* TIMESYNCH_CONF_ENABLED */


  // LOG_DBG("LOG: m2dagnos.c : enter M2DAGNOS_rucast_recv \n");

  //PRINTF("LOG:c_node->node_addr %d.%d sink %d.%d\n",
  //          c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->current_working_node_addr.u8[0], c_node->current_working_node_addr.u8[1]);

  // Only process received message that belong to the same grid
  if (((M2DAGNOS_working_msg*)msg)->grid_id == c_node->grid_id)
  {
  	if (((M2DAGNOS_working_msg*)msg)->msg_id == M2DAGNOS_MSG_WORKING)
    {
      // Set flag in RX_WORKING_MSG
      c_node->flags->flag_process = M2DAGNOS_FLAG_RX_WORKING_MSG;

      PRINTF("%lu RECV_WORK_MSG %d.%d %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1],
      	 from->u8[0], from->u8[1], c_node->grid_id);

      // Print the working message received (it's received only if t_sleep has expired)
      LOG_DBG("LOG: %d.%d: rucast_WORKING_received (%d) from node %d.%d in grid %d\n",
        c_node->node_addr->u8[0], c_node->node_addr->u8[1], ((M2DAGNOS_working_msg*)msg)->msg_id,
        from->u8[0], from->u8[1], ((M2DAGNOS_working_msg*)msg)->grid_id);
    };
  }
  else if(linkaddr_cmp(&c_node->current_working_node_addr, c_node->node_addr))
  {
  	if (((M2DAGNOS_report_msg*)msg)->msg_id == M2DAGNOS_MSG_DATA_REPORT)
  	{
  		// For performance evaluation PURPOSE (src_addr, src_grid_id, sensed_value)RECV_DATA_REPORT_MSG
  		printf(" D %u %u %d.%d %d %d %d R %d.%d \n", timestamp2, ((M2DAGNOS_report_msg*)msg)->timestamp,((uint8_t *)msg)[2], ((uint8_t *)msg)[3],
  			((uint8_t *)msg)[1],((uint8_t *)msg)[7], ((uint8_t *)msg)[6],
  			c_node->node_addr->u8[0], c_node->node_addr->u8[1]);

        // #if FW1_MULTIHOPS //FOR CMnt For energy model validation purpose
        //   #if TRIGGERING
        //       trigger_set(); 	//send the trigger signal, high GPIO
        //       clock_delay(WAIT_TIME); // wait for a while
        //       trigger_clear(); 	//clear the signal, low GPIO
        //   #else
        //         //DO NOTHING
        //   #endif
        // #endif
    }
    else if (((M2DAGNOS_report_msg*)msg)->msg_id == M2DAGNOS_MSG_ALARM_REPORT)
    {

    	// For performance evaluation PURPOSE RECV_EVENT_REPORT_MSG
    	printf(" E %u %u %d.%d %d %d %d R %d.%d \n", timestamp2,  ((M2DAGNOS_report_msg*)msg)->timestamp, ((uint8_t *)msg)[2], ((uint8_t *)msg)[3],
  			((uint8_t *)msg)[1],((uint8_t *)msg)[7], ((uint8_t *)msg)[6],
  			c_node->node_addr->u8[0], c_node->node_addr->u8[1]);

        #if TRIGGERING
              if (((uint8_t *)msg)[6] == 1 && ((uint8_t *)msg)[2] == 2 )
              //if (((uint8_t *)msg)[6] == 1)
              {
                trigger_set(); 	//send the trigger signal, high GPIO
                clock_delay(WAIT_TIME); // wait for a while
                trigger_clear(); 	//clear the signal, low GPIOsrc
              };
        #else
              //DO NOTHING
        #endif
    };
  }
  else
  {
  	//Discard message.
  	LOG_DBG("LOG: A runist_recv msg discarded!\n");
  	return;
  };
};

void M2DAGNOS_rucast_send (M2DAGNOS_node *c_node, struct runicast_conn *rucast, const linkaddr_t *to,
                      uint8_t retransmissions)
{
    M2DAGNOS_working_msg M2DAGNOS_working_msg;
    static uint8_t seqno_1;
    static uint8_t seqno_2;
    static uint8_t seqno;

    if (c_node->msg_type == M2DAGNOS_MSG_DATA_REPORT || c_node->msg_type == M2DAGNOS_MSG_ALARM_REPORT)
    {
        if (c_node->msg_type == M2DAGNOS_MSG_DATA_REPORT)
        {
            seqno_2 = 0; //reset seqno
            //if (seqno_1 > 255)
            //    seqno_1 = 0;

            seqno_1++;
            seqno = seqno_1;


        }
        else
        {
            seqno_1 = 0; //reset seqno
            //if (seqno_2 > 255)
            //    seqno_2 = 0;

            seqno_2++;
            seqno = seqno_2;
        };

        rtimer_clock_t timestamp;
        #if TIMESYNCH_CONF_ENABLED
          timestamp = timesynch_time();
          //PRINTF("TIMESYNCH_CONF_ENABLED \n");
        #else
          timestamp = RTIMER_NOW();
        #endif /* TIMESYNCH_CONF_ENABLED */

        M2DAGNOS_store_msg (c_node, c_node->msg_type, to, seqno, timestamp);
    };

    if(!runicast_is_transmitting(rucast)) // Enter only if there is no runicast TX process
    {
        // Constructing a reliable unicast message
        if (c_node->msg_type == M2DAGNOS_MSG_WORKING)
        {
            c_node->flags->flag_process &= ~M2DAGNOS_FLAG_RX_WORKING_MSG;
            M2DAGNOS_working_msg.msg_id = M2DAGNOS_MSG_WORKING;
            M2DAGNOS_working_msg.grid_id = c_node->grid_id;
            M2DAGNOS_working_msg.state = c_node->state;
            linkaddr_copy(&M2DAGNOS_working_msg.src, c_node->node_addr);
            M2DAGNOS_working_msg.dst = to;
            M2DAGNOS_working_msg.last_seqno = 0;
            packetbuf_copyfrom(&M2DAGNOS_working_msg, sizeof(M2DAGNOS_working_msg));
            // #if WURx
            //     //collect_send(c_node->collect_app, retransmissions); //send via the collect tree
            // /*
            //   Runicast send module is used instead of the collect_send module for
            //   communication between nodes different to the sink. The collect_send
            //   is not allowing to relay a packet to a different node, having
            //   the sink as a father.???
            //   */
            //   	static uint8_t txpower;
            //     txpower = CC2520_TXPOWER_MAX;
            //     cc2520_set_txpower(txpower);
            //     printf("Runicast txpower set to %u\n", txpower);
            //     runicast_send(rucast, to, retransmissions);
            // #else
            //     runicast_send(rucast, to, retransmissions);
            // #endif

              /* If the next WN ID is more than 2 times the current WN node addr increment the PTX
                It is assuming that the next WN is at at least 2 hop-distance
              */
              //if(c_node->next_node_queue_addr.u8[0] > c_node->node_adc_node->lower_iddr->u8[0]+2 || c_node->next_node_queue_addr.u8[0] == c_node->lower_id.u8[0])
              // if(c_node->next_node_queue_addr.u8[0] == c_node->lower_id.u8[0]) //first node in the node queue far away from the sink for measurement purposes
              // {
              //   static uint8_t txpower;
              //   txpower = CC2520_TXPOWER_MAX;
              //   cc2520_set_txpower(txpower);
              //   printf("M2DAGNOS_rucast_send: Wait for TX power setting!\n");
              //   //clock_delay(10000); //time for change the power TX value
              //   printf("M2DAGNOS_rucast_send: txpower set to %u %d>%d\n", txpower, c_node->next_node_queue_addr.u8[0],
              //           c_node->node_addr->u8[0]);
              // }

              runicast_send(rucast, to, retransmissions);

            //c_node->flags->flag_PTX_changed = ON;  //Changed to MAX_PTX!
            LOG_DBG("LOG: M2DAGNOS_rucast_send - Node %d.%d: sending runicast msg (typ: %u) to address %d.%d\n",
                c_node->node_addr->u8[0], c_node->node_addr->u8[1], c_node->msg_type, to->u8[0], to->u8[1]);
        };
    }
    else
    {
      c_node->flags->flag_process &= ~M2DAGNOS_FLAG_TX_WORKING_MSG;
      LOG_DBG("ERROR: Wrong runicast message type \n");
      //Discard message.
      return;
    };
};

void M2DAGNOS_rucast_timedout (M2DAGNOS_node *c_node, struct runicast_conn *rucast, const linkaddr_t *to, uint8_t retransmissions)
{
    LOG_DBG("runicast message timed out when sending to %d.%d, retransmissions %d\n",
        to->u8[0], to->u8[1], retransmissions);
};

void M2DAGNOS_bcast_recv (M2DAGNOS_node *c_node, struct broadcast_conn *bcast, M2DAGNOS_broadcast_msg *msg, const linkaddr_t *sender)
{
    //LOG_DBG("LOG: m2dagnos.c : enter M2DAGNOS_bcast_recv - msg_type = %d &  grid_id = %d\n",
    //  msg->msg_id, msg->msg_id);

    if (msg->grid_id == c_node->grid_id)
    {
        if (msg->msg_id == M2DAGNOS_MSG_ASSISTING)
        {
        	// For performance evaluation PURPOSE
            PRINTF("%lu RCV_ASSIST_MSG %d.%d %d.%d %d\n", clock_time(), c_node->node_addr->u8[0], c_node->node_addr->u8[1],
            	msg->src.u8[0], msg->src.u8[1], c_node->grid_id);

            // Set flag in RX_ASSISTING_MSG
            c_node->flags->flag_process = M2DAGNOS_FLAG_RX_ASSISTING_MSG;
            c_node->flags->flag_grid_alert = M2DAGNOS_STATUS_ALERTING;

      		// Set the timer for event data reporting to the sink
     		c_node->period_report_sink = M2DAGNOS_EVENT_REPORT_SINK_PER;
      		etimer_set(c_node->p_report_sink, c_node->period_report_sink);
      		etimer_stop(c_node->p_report_sink);

            // Print the assisting message received (it's received only if t_sleep has expired)
            LOG_DBG("LOG: %d.%d: bcast_ASSISTING_MSG_received (%d) from WN %d.%d in grid %d\n",
                    c_node->node_addr->u8[0], c_node->node_addr->u8[1], msg->msg_id,
                    sender->u8[0], sender->u8[1], msg->grid_id);
        };
    }
    else
    {
        //Discard message.
        LOG_DBG("LOG: A bcast_recv msg discarded!\n");
        return;
    };
};

void M2DAGNOS_bcast_send (M2DAGNOS_node *c_node, struct broadcast_conn *bcast)
{

  M2DAGNOS_broadcast_msg broadcast_msg;
    static uint8_t seqno;
    seqno = 0;

  // Constructing a assiting message
  if (c_node->msg_type == M2DAGNOS_MSG_ASSISTING)
  {
    if(!c_node->flags->flag_grid_assist_msg)
    {
      broadcast_msg.msg_id = M2DAGNOS_MSG_ASSISTING;
      c_node->flags->flag_grid_assist_msg = M2DAGNOS_FLAG_TX_ASSISTING_MSG;
      // LOG_DBG("LOG: m2dagnos.c : enter M2DAGNOS_bcast_send - broadcast_msg.msg_id = %d\n", broadcast_msg.msg_id);
    }
    else
    {
      LOG_DBG("LOG: m2dagnos.c : A bcast msg Assisting type was already sent!\n");
      return; // A bcast msg Assisting type was already sent.
    };
  }
  else
  {
    LOG_DBG("ERROR: Wrong broadcast message type \n");
    return;
  };

  // static uint8_t txpower;
  // txpower = CC2520_TXPOWER_MAX;
  // cc2520_set_txpower(txpower);  //Changed to MAX_PTX!
  // printf("M2DAGNOS_bcast_send txpower set to %u\n", txpower);
  // c_node->flags->flag_PTX_changed = ON;

  broadcast_msg.grid_id = c_node->grid_id;
  linkaddr_copy(&broadcast_msg.src, c_node->node_addr);
  broadcast_msg.seqno = seqno;
  packetbuf_copyfrom(&broadcast_msg, sizeof(broadcast_msg));

  // Sending an assiting  msg to each neighbor in the grid (until Nj)
  broadcast_send(bcast);


  // LOG_DBG("LOG: M2DAGNOS_bcast_send: Sending assiting broadcast msg (typ: %u) to each neighbor in A_%i by WN (%d.%d)\n",
  //   broadcast_msg.msg_id, broadcast_msg.grid_id, broadcast_msg.src->u8[0], broadcast_msg.src->u8[1]);
  seqno++;
};

// uint8_t fire[] = {        90,   90,  90,  95,  95,  95, 101, 101, 101, 101, 101,101,
//                          100,  105, 110,  90,  90,  90,  80,  80,  80,  70, 70, 70,
//                          100,  100, 100, 120, 125, 125, 131, 131,  90,  90, 90,
//                           90,   95,  95,  95, 101, 101, 101, 101, 101, 101,
//                          100,  105, 110,  90,  90,  90,  80,  80,  80,  70, 70, 70,
//                          100,  100, 100, 120, 125, 125, 131, 131,  90,
//                           90,   90,  90,  95,  95,  95, 101, 101, 101, 101, 101,101,
//                          100,  105, 110,  90,  90,  90,  80,  80,  80,  70, 70, 70,
//                          100,  100, 100, 120, 125, 125, 131, 131,  90,  90, 90,
//                           90,   95,  95,  95, 101, 101, 101, 101, 101, 101,
//                          100,  105, 110,  90,  90,  90,  80,  80,  80,  70, 70, 70,
//                          100,  100, 100, 120, 125, 125, 131, 131,  90,
//                           90,   90,  90,  95,  95,  95, 101, 101, 101, 101, 101,101,
//                          100,  105, 110,  90,  90,  90,  80,  80,  80,  70, 70, 70,
//                          100,  100, 100, 120, 125, 125, 131, 131,  90,  90, 90,
//                           90,   95,  95,  95, 101, 101, 101, 101, 101, 101,
//                          100,  105, 110,  90,  90,  90,  80,  80,  80,  70, 70, 70,
//                          100,  100, 100, 120, 125, 125, 131, 131,  90,
//                           90,   90,  90,  95,  95,  95, 101, 101, 101, 101, 101,101,
//                          100,  105, 110,  90,  90,  90,  80,  80,  80,  70, 70, 70,
//                          100,  100, 100, 120, 125, 125, 131, 131,  90,  90, 90,
//                           90,   95,  95,  95, 101, 101, 101, 101, 101, 101,
//                          100,  105, 110,  90,  90,  90,  80,  80,  80,  70, 70, 70,};

uint8_t fire[] = {       70,	70,	71,	72,	73,	74,	71,	71,	72,	73,	74,	75,	72,	72,	73,	74,	75,
                         76,	77,	78,	79,	80,	81,	82,	83,	84,	85,	86,	87,	88,	89,	90,	91,	92,
                         93,	94,	95,	96,	97,	96,	97,	96,	97,	96,	97,	96,	98,	99,	98,	99,	98,
                         99,	98,	99,	100,	101,	102,	103,	104,	105,	106,	107,	106,	106,
                         106,	108,	107,	106,	105,	104,	102,	100,	99,	98,	97,	96,	95,	94,
                         95,	95,	92,	91,	90,	89,	88,	87,	86,	85,	84,	83,	82,	83,	84,	85,	86,	87,
                         88,	89,	90,	91,	92,	93,	94,	95,	96,	97,	98,	99,	100,	101,	102,	103,
                         104,	105,	104,	104,	105,	106,	107,	108,
                  };

uint8_t M2DAGNOS_fire_dataset (M2DAGNOS_node *c_node)
{
    static uint8_t i= -1;
    uint8_t *pfire = fire;

    if (c_node->flags->flag_set_fire_dataset == 1)
    {
        i++;
        //if (i == 255)
        if (i >= 120 && c_node->state != M2DAGNOS_ST_ASSISTING)
            i = 0;

        // LOG_DBG("LOG: M2DAGNOS.c : c_node->fire[i] = %d, %d, %d.\n",
        //  pfire[i], i, c_node->flags->flag_set_fire_dataset);

        return pfire[i];
    }
    else
    {
        uint8_t sensed_value = ((random_rand() % RANDOM_RAND_MAX ) / 6000) + 25; // random temperature values= Range [0, 200] *C
        return sensed_value;
    };
};

void M2DAGNOS_store_msg (M2DAGNOS_node *c_node,  uint8_t msg_id, const linkaddr_t *to, uint8_t seqno, rtimer_clock_t timestamp)
{
    struct M2DAGNOS_out_msg_queue *n;

    n = memb_alloc(&M2DAGNOS_out_msg_queue_memb);

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
        n->sensed_value = c_node->S_curr; //c_node->sensed_value;
        n->timestamp = timestamp;

         if (c_node->msg_type == M2DAGNOS_MSG_DATA_REPORT)
        {
            list_push(out_msg_queue_list, n); // Add an item to the start of the list.
        }
        else
        {
            list_add(out_msg_queue_list, n); // Add an item at the end of a list.
        };


        LOG_DBG("LOG: M2DAGNOS_store_msg : In %d.%d the packet %d (type %d) has been stored in the <<out_msg_queue_list)>> (length: %d).\n",
          c_node->node_addr->u8[0], c_node->node_addr->u8[1], seqno, msg_id, list_length(out_msg_queue_list));
    };
};

void M2DAGNOS_clear_out_msg_queue (M2DAGNOS_node *c_node)
{
    while(list_length(out_msg_queue_list))
    {
        list_pop(out_msg_queue_list);
    };

    LOG_DBG("LOG: M2DAGNOS_clear_out_msg_queue : In %d.%d the <<out_msg_queue_list)>> has been cleaned up (length: %d).\n",
          c_node->node_addr->u8[0], c_node->node_addr->u8[1], list_length(out_msg_queue_list));
};

/*---------------------------------------------------------------------------*/
