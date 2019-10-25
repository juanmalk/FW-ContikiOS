/*
 * Copyright (c) 2018 PUJ-USeA.
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
 *         Header file MultiModal-Data Gathering and Node Scheduling Mechanism
 *
 * \author
 *         juanmalk <juan-aranda@javeriana.edu.co>
 *                  <juan.aranda@usa.edu.co>
 * \date_last_version
 *          14/06/2018
 */


/**
 * Note: Code based on Tree-based hop-by-hop reliable data collection, 2007 SICS
 * Author: Adam Dunkels <adam@sics.se>
 */


#ifndef M2DAGNOS_H_
#define M2DAGNOS_H_

#include "contiki.h"
#include "net/rime/rime.h"
#include "net/linkaddr.h"
#include "net/rime/collect.h"
#include "sys/node-id.h"
#include "lib/memb.h"
#include "lib/list.h"
#include <string.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <assert.h>

/*
 *
 * PARAMETERS DEFINITION
 *
 */

// States - State machine
#define M2DAGNOS_ST_SLEEPING                    0x01
#define M2DAGNOS_ST_INITIALIZING                0x02  //Neighbor Scheduling
#define M2DAGNOS_ST_WORKING                     0x04
#define M2DAGNOS_ST_ASSISTING                   0x08
#define M2DAGNOS_ST_CONTINUOUS_MONITORING       0x10
#define M2DAGNOS_ST_EVENT_SAMPLING              0x20
#define M2DAGNOS_ST_DEAD                        0x40

// States - Sub State machine WN election
#define M2DAGNOS_ST_WN_INIT                     0x01
#define M2DAGNOS_ST_WN_EXCHANGE                 0x02
#define M2DAGNOS_ST_WN_SETTING                  0x04
#define M2DAGNOS_ST_WN_BUILDING                 0x08
#define M2DAGNOS_ST_WN_END                      0x10

// Type of messages
#define M2DAGNOS_MSG_WORKING                    0x01
#define M2DAGNOS_MSG_ASSISTING                  0x02
#define M2DAGNOS_MSG_DATA_REPORT                0x04
#define M2DAGNOS_MSG_ALARM_REPORT               0x08

// Flags
#define M2DAGNOS_FLAG_QUEUE_COMPLETED           0x01
#define M2DAGNOS_FLAG_RX_WORKING_MSG            0x02
#define M2DAGNOS_FLAG_TX_WORKING_MSG            0x04
#define M2DAGNOS_FLAG_RX_ASSISTING_MSG          0x08
#define M2DAGNOS_FLAG_TX_ASSISTING_MSG          0x10
#define M2DAGNOS_FLAG_RX_REPORT_SINK_MSG		    0x20
#define M2DAGNOS_FLAG_WAIT_INIT                 0x40


// Grid status
#define M2DAGNOS_STATUS_DEAD                    0x01
#define M2DAGNOS_STATUS_ALERTING                0x02
#define M2DAGNOS_STATUS_COLLECTING              0x04
#define M2DAGNOS_STATUS_WAIT_REPORT             0x08

// THRESHOLDS & MAXIMUM
#define M2DAGNOS_EVENT_TH                       100 //degree Celsius (°C)
#define M2DAGNOS_P_START                        3
#define M2DAGNOS_Q_START                        5
#define M2DAGNOS_P_STOP                         3
#define M2DAGNOS_Q_STOP                         5
#define MAX_RETRANSMISSIONS                     5//10//30//10//30//20//10
#define ON                                      1
#define OFF                                     0
#define DELTA                                   0.5  // Data reporting granularity

// PERIODS & TIMEOUTS
#define M2DAGNOS_COLLECT_PER                    120 * CLOCK_SECOND//50 * CLOCK_SECOND // PERIOD ALLOWED A SN TO COLLECT DATA
#define M2DAGNOS_SAMPLING_PER                   1 * CLOCK_SECOND //ACQUIRING DATA
#define M2DAGNOS_DATA_REPORT_SINK_PER           5 * CLOCK_SECOND//10 * CLOCK_SECOND
#define M2DAGNOS_EVENT_REPORT_SINK_PER          DELTA * M2DAGNOS_DATA_REPORT_SINK_PER
#define M2DAGNOS_EXE_PED_PER                    2 * CLOCK_SECOND//5 * CLOCK_SECOND
#define M2DAGNOS_SIGMA_TIMEOUT                  5 * CLOCK_SECOND
#define M2DAGNOS_T_START                        100//60
#define M2DAGNOS_T_QUEUE_BUILD                  120
#define M2DAGNOS_EVENT_PER                      10* CLOCK_SECOND//900 * CLOCK_SECOND//200 * CLOCK_SECOND//60 * CLOCK_SECOND // 490* CLOCK_SECOND
#define M2DAGNOS_M_PARAMETER                    1
#define M2DAGNOS_TIME_OUT_MSG                   0.001
#define M2DAGNOS_TIME_PRE_OUT_MSG               0.002

// CM500O LED COLORS
#define LEDS_ST_WORKING        LEDS_BLUE//LEDS_RED   //&LEDS_ST_MONITORING
#define LEDS_ST_ASSISTING      LEDS_RED///LEDS_BLUE
#define LEDS_ST_SLEEPING       LEDS_ALL // ALL OFF
#define LEDS_ST_SAMPLING       LEDS_GREEN
#define LEDS_REBOOT            LEDS_ALL
#define LEDS_SINK_NODE         LEDS_ALL
/*
 *
 * STRUCTURES DEFINITION
 *
 */
typedef struct M2DAGNOS_node M2DAGNOS_node;
typedef struct M2DAGNOS_flags M2DAGNOS_flags;
typedef struct M2DAGNOS_node_queue M2DAGNOS_node_queue;
typedef struct M2DAGNOS_working_msg M2DAGNOS_working_msg;
typedef struct M2DAGNOS_broadcast_msg M2DAGNOS_broadcast_msg;
typedef struct M2DAGNOS_report_msg M2DAGNOS_report_msg;
typedef struct M2DAGNOS_neighbor_tier_info M2DAGNOS_neighbor_tier_info;

// Hybrid node properties
struct M2DAGNOS_node {
    uint8_t state;  // current state
    uint8_t state_sub_process; // Node queue build
    M2DAGNOS_flags *flags; // pointer to the flags structure
    uint8_t grid_id; // grid identification
    uint8_t tier_id; // the network is divided into grid, and each grid belong to a particular tier
    uint8_t sensed_value; // synthetic sensed value (e.g., temperature value)
    uint16_t S_curr; // Current average sensed value
    uint16_t S_prev; // Previous average sensed value
    uint8_t p;  // PED parameter
    uint8_t q;  // PED parameter
    uint8_t batt_level; // Initial battery load
    uint16_t Nj; // Number of nodes within the grid
    uint8_t msg_type; //Type of message
    uint8_t turn_sche; //Turn assigned during the node queue building proces
    const linkaddr_t *node_addr; // Node Rime address
    linkaddr_t lower_id;
    linkaddr_t current_working_node_addr; //Current WN Rime address
    linkaddr_t next_node_queue_addr; // Rime address of the next node within the node queue
    list_t p_out_msg_list;
    struct memb *p_out_msg_memb;

    M2DAGNOS_neighbor_tier_info *M2DAGNOS_nti; //pointer to the lower tier neighbor structure (e.g., sink)
    struct etimer *t_collect; // pointer
    clock_time_t interval_collect; // time period
    struct etimer *t_sleep;
    clock_time_t interval_sleep;
    struct etimer *t_work;
    clock_time_t interval_work;
    struct etimer *t_start;
    clock_time_t interval_start;
    struct etimer *t_event;
    clock_time_t interval_event;
    struct etimer *t_sigma;
    clock_time_t interval_sigma;
    struct etimer *p_report_sink;
    clock_time_t period_report_sink;
    struct collect_conn *collect_app;
    struct etimer *t_wait;
    clock_time_t interval_wait;
    struct etimer *p_exe_ped;
    clock_time_t period_exe_ped;
    struct etimer *p_sampling;
    clock_time_t period_sampling;
};

struct M2DAGNOS_flags {
    uint8_t flag_node_queue;
    uint8_t flag_process;
    uint8_t flag_grid_alert;
    uint8_t flag_grid_collect;
    uint8_t flag_grid_assist_msg;
    uint8_t flag_set_sigma_tout;
    uint8_t flag_sim; // for simulation purpose
    uint8_t flag_set_fire_dataset;
    uint8_t flag_completed_sorting_node_queue;
    uint8_t flag_PTX_changed;
};

// Node queue inside a grid
struct M2DAGNOS_node_queue {
    struct M2DAGNOS_node_queue *next;
    linkaddr_t addr; //join node addr
    uint8_t state;   //state of join node
    uint8_t sqno;   //sequence number (position in the node queue)
};

// Node queue inside a grid
struct M2DAGNOS_out_msg_queue {
    struct M2DAGNOS_out_msg_queue *next;
    uint8_t msg_id;
    linkaddr_t dst_addr;
    uint8_t seqno;
    uint8_t sensed_value;
    rtimer_clock_t timestamp;
};

// Working Message (runicast)
struct M2DAGNOS_working_msg {
    uint8_t msg_id;
    uint8_t grid_id;
    uint8_t state;
    linkaddr_t src; //current_working_node_addr
    const linkaddr_t *dst; //next_working_node_addr
    uint8_t last_seqno;
};

// Claim Message or Assisting Message (Broadcast) //neighbor-discovery.h
struct M2DAGNOS_broadcast_msg {
    uint8_t msg_id; // M2DAGNOS_MSG_ASSISTING
    uint8_t grid_id;
    linkaddr_t src; //working_node_addr
    uint8_t seqno;
};

// Report message (runicast)
struct M2DAGNOS_report_msg
{ //rime/rucb.h
    uint8_t msg_id; // M2DAGNOS_MSG_DATA_REPORT or M2DAGNOS_MSG_ALARM_REPORT
    uint8_t src_grid_id;
    // uint8_t src_tier_id;
    // uint8_t dst_tier_id;
    linkaddr_t src; //working_node_addr
    linkaddr_t dst; //neighbor_lower_tier_addr
    // uint8_t src_grid_status;
    uint8_t last_seqno;
    uint8_t sensed_value;
    rtimer_clock_t timestamp;
};

// Info about neighbor in the lower tier
struct M2DAGNOS_neighbor_tier_info {
  	uint8_t grid_id;
  	uint8_t tier_id;
  	linkaddr_t neighbor_lower_tier_addr;
  	uint8_t neighbor_lower_tier_status;
  	uint8_t neighbor_lower_tier_state;
};


/*
 *
 * EVENTS DEFINITION
 *
 */

process_event_t ev_sleeping;
process_event_t ev_working;
process_event_t ev_assisting;
process_event_t ev_monitoring;
process_event_t ev_sampling;
process_event_t ev_initializing;
process_event_t ev_init_m2dagnos;

/*
 *
 * FUNCTIONS DEFINITION
 *
 */

// Initialization procedure by master processs
void M2DAGNOS_init_proc (M2DAGNOS_node *c_node, uint8_t state, uint8_t state_sub_process, M2DAGNOS_flags *flags, uint8_t grid_id, uint8_t tier_id, uint8_t msg_type,
                     uint8_t turn_sche, uint8_t sensed_value, uint8_t batt_level, uint16_t S_curr, uint16_t S_prev, uint8_t p, uint8_t q,
                     uint16_t Nj,  const linkaddr_t *node_addr, const linkaddr_t *next_node_queue_addr, const linkaddr_t *current_working_node_addr,
                     const linkaddr_t *lower_id, M2DAGNOS_neighbor_tier_info *M2DAGNOS_nti, struct etimer *t_collect,
                     struct etimer *t_sleep, struct etimer *t_work, struct etimer *t_start,
                     struct etimer *t_event, struct etimer *t_sigma, struct etimer *p_report_sink,
                     struct collect_conn *collect_app, struct etimer *t_wait, struct etimer *p_exe_ped,
                     uint8_t flag_dataset, struct etimer *p_sampling, uint8_t flag_sim);

// Acquiring or updating information about the neighbor in the lower tier
void M2DAGNOS_update_neighbor_lower_tier_info_proc (M2DAGNOS_neighbor_tier_info *c_info, uint8_t grid_id, uint8_t tier_id,
											   const linkaddr_t *neighbor_lower_tier_addr,
											   uint8_t neighbor_lower_tier_status, uint8_t neighbor_lower_tier_state);

// Node exploring procedure inside the grid
void M2DAGNOS_exploring_proc (M2DAGNOS_node *c_node, struct runicast_conn *rucast, struct broadcast_conn *bcast);

// Node sleeping scheduling inside the grid
void M2DAGNOS_sleeping_proc (M2DAGNOS_node *c_node, struct runicast_conn *rucast, struct broadcast_conn *bcast);

// Node wakeup scheduling inside the grid
void M2DAGNOS_wakeup_proc (M2DAGNOS_node *c_node, struct runicast_conn *rucast, struct broadcast_conn *bcast);

// Collecting data procedure
void M2DAGNOS_collecting_data_proc (M2DAGNOS_node *c_node, M2DAGNOS_neighbor_tier_info *c_tier, struct runicast_conn *rucast,
                               struct broadcast_conn *bcast);

// Building node queue procedure
void M2DAGNOS_builiding_queue_proc (M2DAGNOS_node *c_node, const linkaddr_t *from, uint8_t state);

// Node queue scheduling cycle
const linkaddr_t* M2DAGNOS_node_queue_scheduling (M2DAGNOS_node *c_node, const linkaddr_t* current_addr); //function not clear in the paper.

// Parameter-based event detection (PED) algorithm
void M2DAGNOS_ped_proc (M2DAGNOS_node *c_node);

// Calc the average of sensed values over a time interval
uint16_t M2DAGNOS_calc_avg_s_curr_proc (M2DAGNOS_node *c_node);

//Store sensed value
void M2DAGNOS_store_sensed_value_queue_proc (M2DAGNOS_node *c_node, uint8_t sensed_value);

// Setting & updating sleep timeout
void M2DAGNOS_set_sleep_timeout (M2DAGNOS_node *c_node, uint8_t m);

// Setting & updating work timeout
void M2DAGNOS_set_work_timeout (M2DAGNOS_node *c_node, uint8_t m);

// Restart timers
void M2DAGNOS_set_event_timeout (M2DAGNOS_node *c_node);
void M2DAGNOS_restart_event_timeout (M2DAGNOS_node *c_node);
void M2DAGNOS_restart_sigma_timeout (M2DAGNOS_node *c_node);
void M2DAGNOS_set_collect_timeout (M2DAGNOS_node *c_node);
void M2DAGNOS_set_data_report_period (M2DAGNOS_node *c_node);
uint8_t M2DAGNOS_check_wait_time (M2DAGNOS_node *c_node);
void M2DAGNOS_set_wait_time (M2DAGNOS_node *c_node, clock_time_t interval_wait);
void M2DAGNOS_update_scheduling_period (M2DAGNOS_node *c_node, uint8_t m);
void M2DAGNOS_set_ped_per (M2DAGNOS_node *c_node);
uint8_t M2DAGNOS_check_ped_per (M2DAGNOS_node *c_node);
void M2DAGNOS_set_sampling_per (M2DAGNOS_node *c_node);
uint8_t M2DAGNOS_check_samping_per (M2DAGNOS_node *c_node);

// Receiving procedure of reliable unicast messages
void M2DAGNOS_rucast_recv (M2DAGNOS_node *c_node, struct runicast_conn *rucast, void *msg,
					  const linkaddr_t *from, uint8_t seqno);

// Sending procedure of reliable unicast messages
void M2DAGNOS_rucast_send (M2DAGNOS_node *c_node, struct runicast_conn *rucast, const linkaddr_t *to,
					            uint8_t retransmissions);

// Timeout procedure of reliable unicast messages
void M2DAGNOS_rucast_timedout (M2DAGNOS_node *c_node, struct runicast_conn *rucast,
	                      const linkaddr_t *to, uint8_t retransmissions);

// Receiving procedure of broadcast messages
void M2DAGNOS_bcast_recv (M2DAGNOS_node *c_node, struct broadcast_conn *bcast,
				     M2DAGNOS_broadcast_msg *msg, const linkaddr_t *sender);

// Sending procedure of broadcast messages
void M2DAGNOS_bcast_send (M2DAGNOS_node *c_node, struct broadcast_conn *bcast);

// Helper C function
uint8_t M2DAGNOS_fire_dataset (M2DAGNOS_node *c_node);

void M2DAGNOS_store_msg (M2DAGNOS_node *c_node,  uint8_t msg_id, const linkaddr_t *to, uint8_t seqno, rtimer_clock_t timestamp);
void M2DAGNOS_clear_out_msg_queue (M2DAGNOS_node *c_node);

#endif /* M2DaGNoS_H_ */
