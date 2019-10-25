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
 *         Header file for Kang's hybrid node scheduling
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


#ifndef KHS_H_
#define KHS_H_

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
#include "dev/leds.h"
#include "dev/button-sensor.h"

/*
 *
 * PARAMETERS DEFINITION
 *
 */

// States - State machine
#define KHS_ST_SLEEPING                    0x01
#define KHS_ST_INITIALIZING                0x02
#define KHS_ST_WORKING                     0x04
#define KHS_ST_ASSISTING                   0x08
#define KHS_ST_DEAD               		     0x10

// Type of messages
#define KHS_MSG_CLAIM                      0x01
#define KHS_MSG_WORKING                    0x02
#define KHS_MSG_ASSISTING                  0x04
#define KHS_MSG_JOIN_REQ                   0x08
#define KHS_MSG_JOIN_RES                   0x10
#define KHS_MSG_DATA_REPORT                0x20
#define KHS_MSG_ALARM_REPORT               0x40

// Flags
#define KHS_FLAG_QUEUE_COMPLETED           0x01
#define KHS_FLAG_RX_WORKING_MSG            0x02
#define KHS_FLAG_TX_WORKING_MSG            0x04
#define KHS_FLAG_RX_ASSISTING_MSG          0x08
#define KHS_FLAG_RX_CLAIM_MSG              0x10
#define KHS_FLAG_RX_JOIN_MSG  		         0x20
#define KHS_FLAG_RX_REPORT_SINK_MSG		     0x40
#define KHS_FLAG_WAIT_INIT                 0x80
#define KHS_FLAG_TX_ASSISTING_MSG          0x11

// Grid status
#define KHS_STATUS_DEAD                    0x01
#define KHS_STATUS_ALERTING                0x02
#define KHS_STATUS_COLLECTING              0x04
#define KHS_STATUS_WAIT_REPORT             0x08

// PERIODS & TIMEOUTS
//#define KHS_COLLECT_PER                    50 * RTIMER_SECOND
//#define KHS_DATA_REPORT_SINK_PER           10 * RTIMER_SECOND
#define KHS_COLLECT_PER                    50 * CLOCK_SECOND
#define KHS_DATA_REPORT_SINK_PER           5 * CLOCK_SECOND//10 * CLOCK_SECOND
#define KHS_SIGMA_TIMEOUT                  5 * CLOCK_SECOND
#define KHS_TIN                            5 * CLOCK_SECOND//5 * CLOCK_SECOND
#define KHS_EVENT_PER                      60 * CLOCK_SECOND//10* CLOCK_SECOND//900 * CLOCK_SECOND//200 * CLOCK_SECOND//60 * CLOCK_SECOND // 490* CLOCK_SECOND
#define KHS_M_PARAMETER                    1
#define KHS_SAMPLING_PER                   1 * CLOCK_SECOND //ACQUIRING DATA
#define KHS_T_START                        100//60
#define KHS_TIME_OUT_MSG                   0.001
#define KHS_TIME_PRE_OUT_MSG               0.002

// THRESHOLDS & MAXIMUM
#define KHS_EVENT_TH                       100 //degree Celsius (°C)
#define MAX_RETRANSMISSIONS                 10
#define ON                                  1
#define OFF                                 0

// LEDS COLOR INDICATION FOR RE-MOTE ZOLERTIA
#if FW1_ACTIVE
  #define LEDS_ST_WORKING        LEDS_RED//LEDS_RED
  #define LEDS_ST_ASSISTING      LEDS_BLUE///LEDS_BLUE
  #define LEDS_ST_SLEEPING       LEDS_ALL // ALL OFF
  #define LEDS_ST_SAMPLING       LEDS_GREEN
  #define LEDS_REBOOT            LEDS_ALL
  #define LEDS_SINK_NODE         LEDS_ALL
#else
  // CM500O LED COLORS
  #define LEDS_ST_WORKING        LEDS_RED   //&LEDS_ST_MONITORING
  #define LEDS_ST_ASSISTING      LEDS_BLUE
  #define LEDS_ST_SLEEPING       LEDS_ALL // ALL OFF
  #define LEDS_ST_SAMPLING       LEDS_GREEN
  #define LEDS_REBOOT            LEDS_ALL
  #define LEDS_SINK_NODE         LEDS_ALL
#endif
/*
 *
 * STRUCTURES DEFINITION
 *
 */
typedef struct khs_node khs_node;
typedef struct khs_flags khs_flags;
typedef struct khs_node_queue khs_node_queue;
typedef struct khs_working_msg khs_working_msg;
typedef struct khs_broadcast_msg khs_broadcast_msg;
typedef struct khs_join_msg khs_join_msg;
typedef struct khs_report_msg khs_report_msg;
typedef struct khs_neighbor_tier_info khs_neighbor_tier_info;
typedef struct khs_rucast_msgs_list khs_rucast_msgs_list;

// Hybrid node properties
struct khs_node {
    uint8_t state;
    khs_flags *flags;
    uint8_t grid_id;
    uint8_t tier_id;
    uint8_t sensed_value;
    uint8_t batt_level;
    uint16_t Nj;
    uint8_t msg_type;
    uint8_t turn_sche;
    const linkaddr_t *node_addr;
    linkaddr_t current_working_node_addr;
    linkaddr_t next_node_queue_addr;
    list_t p_out_msg_list;
    struct memb *p_out_msg_memb;

    khs_node_queue *node_list;
    khs_neighbor_tier_info *khs_nti;
    struct etimer *t_collect;
    clock_time_t interval_collect;
    struct etimer *t_idle;
    clock_time_t interval_idle;
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
    struct etimer *p_collect;
    clock_time_t period_collect;
    struct etimer *p_report_sink;
    clock_time_t period_report_sink;
    struct collect_conn *collect_app;
    struct etimer *p_scheduling;
    clock_time_t period_scheduling;
    struct etimer *t_init_sche;
    struct etimer *t_wait;
    clock_time_t interval_wait;
    struct etimer *p_sampling;
    clock_time_t period_sampling;
};

struct khs_flags {
	uint8_t flag_node_queue;
	uint8_t flag_process;
	uint8_t flag_grid_alert;
	uint8_t flag_grid_collect;
	uint8_t flag_grid_assist_msg;
	uint8_t flag_grid_join_msg_res;
  uint8_t flag_grid_join_msg_req;
  uint8_t flag_tx_node_queue_info;
  uint8_t flag_rx_node_queue_info;
  uint8_t flag_set_tout_node_queue_sending;
  uint8_t flag_set_sigma_tout;
  uint8_t flag_still_rcast_msg_to_send;
  uint8_t flag_sim; // for simulation purpose
  uint8_t flag_set_fire_dataset;
};

// Node queue inside a grid
struct khs_node_queue {
	struct khs_node_queue *next;
	linkaddr_t addr; //join node addr
	uint8_t state;   //state of join node
    uint8_t sqno;   //sequence number (position in the node queue)
};

// Node queue inside a grid
struct khs_out_msg_queue {
    struct khs_out_msg_queue *next;
    uint8_t msg_id;
    linkaddr_t dst_addr;
    uint8_t seqno;
    uint8_t sensed_value;
};

// Working Message (runicast)
struct khs_working_msg {
	uint8_t msg_id;
	uint8_t grid_id;
  uint8_t state;
  linkaddr_t src; //current_working_node_addr
	const linkaddr_t *dst; //next_working_node_addr
	uint8_t last_seqno;
	uint16_t length_nqi;
	char node_queue_info[50]; //as a JSON string {addr_node_1, state_node_1,addr_node_2, state_node_3}
};

// Claim Message or Assisting Message (Broadcast) //neighbor-discovery.h
struct khs_broadcast_msg {
  uint8_t msg_id; // KHS_MSG_CLAIM or KHS_MSG_ASSISTING
  uint8_t grid_id;
  const linkaddr_t *src; //working_node_addr
  uint8_t seqno;
};

// Join Message (runicast)
struct khs_join_msg {
  	uint8_t msg_id;
  	uint8_t grid_id;
  	uint8_t state;
  	const linkaddr_t *src; //join_node_addr
  	uint8_t last_seqno;
  	uint8_t turn; //for response msg
};

// Report message (runicast)
struct khs_report_msg
{ //rime/rucb.h
  	uint8_t msg_id; // KHS_MSG_DATA_REPORT or KHS_MSG_ALARM_REPORT
  	uint8_t src_grid_id;
    uint8_t src_tier_id;
  	uint8_t dst_tier_id;
    linkaddr_t src; //working_node_addr
    linkaddr_t dst; //neighbor_lower_tier_addr
  	uint8_t src_grid_status;
 	  uint8_t last_seqno;
  	uint8_t sensed_value;
};

// Node queue inside a grid
struct khs_rucast_msgs_list {
  struct khs_rucast_msgs *next;
  linkaddr_t addr; //dst addr
  uint8_t msg_type;//type of msg
  uint8_t seqno;
};

// Info about neighbor in the lower tier
struct khs_neighbor_tier_info {
  	uint8_t grid_id;
  	uint8_t tier_id;
  	linkaddr_t neighbor_lower_tier_addr;
  	uint8_t neighbor_lower_tier_status;
  	uint8_t neighbor_lower_tier_state;
};

//Contiki list that holds the node in the grid
//LIST_STRUCT(node_queue_list);

/*
 *
 * EVENTS DEFINITION
 *
 */

process_event_t ev_sleeping;
process_event_t ev_working;
process_event_t ev_assisting;
process_event_t ev_initializing;
process_event_t ev_init_khs;

/*
 *
 * FUNCTIONS DEFINITION
 *
 */

// Initialization procedure by master processs
void khs_init_proc (khs_node *c_node, uint8_t state, khs_flags *flags, uint8_t grid_id, uint8_t tier_id, uint8_t msg_type,
                     uint8_t turn_sche, uint8_t sensed_value, uint8_t batt_level, uint16_t Nj,  const linkaddr_t *node_addr,
                     const linkaddr_t *next_node_queue_addr, const linkaddr_t *current_working_node_addr,
                     khs_node_queue *node_list, khs_neighbor_tier_info *khs_nti, struct etimer *t_collect,
                     struct etimer *t_idle, struct etimer *t_sleep, struct etimer *t_work, struct etimer *t_start,
                     struct etimer *t_event, struct etimer *t_sigma, struct etimer *p_collect, struct etimer *p_report_sink,
                     struct collect_conn *collect_app, struct etimer *p_scheduling, struct etimer *t_init_sche, struct etimer *p_sampling,
                     struct etimer *t_wait, uint8_t flag_dataset, uint8_t flag_sim);

// Acquiring or updating information about the neighbor in the lower tier
void khs_update_neighbor_lower_tier_info_proc (khs_neighbor_tier_info *c_info, uint8_t grid_id, uint8_t tier_id,
											   const linkaddr_t *neighbor_lower_tier_addr,
											   uint8_t neighbor_lower_tier_status, uint8_t neighbor_lower_tier_state);

// Node exploring procedure inside the grid
void khs_exploring_proc (khs_node *c_node, struct runicast_conn *rucast, struct broadcast_conn *bcast);

// Node sleeping scheduling inside the grid
void khs_sleeping_proc (khs_node *c_node, struct runicast_conn *rucast, struct broadcast_conn *bcast);

// Node wakeup scheduling inside the grid
void khs_wakeup_proc (khs_node *c_node, struct runicast_conn *rucast, struct broadcast_conn *bcast);

// Collecting data procedure
void khs_collecting_data_proc (khs_node *c_node, khs_neighbor_tier_info *c_tier, struct runicast_conn *rucast,
                               struct broadcast_conn *bcast);

//Store or get sensed value
void khs_store_sensed_value_queue_proc (khs_node *c_node, uint8_t sensed_value);
uint8_t khs_get_sensed_value_queue_proc (khs_node *c_node);

// Building node queue procedure
void khs_builiding_queue_proc (khs_node *c_node, const linkaddr_t *from, uint8_t state);

// Node queue scheduling cycle
const linkaddr_t* khs_node_queue_scheduling (khs_node *c_node, const linkaddr_t* current_addr); //function not clear in the paper.

// Setting & updating sleep timeout
void khs_set_sleep_timeout (khs_node *c_node, uint8_t m);

// Setting & updating work timeout
void khs_set_work_timeout (khs_node *c_node, uint8_t m);
//void khs_restart_work_timeout ();

// Restart timers
void khs_restart_start_timeout (khs_node *c_node);
void khs_set_event_timeout (khs_node *c_node);
void khs_restart_event_timeout (khs_node *c_node);
void khs_restart_sigma_timeout (khs_node *c_node);
void khs_set_collect_timeout (khs_node *c_node);
void khs_restart_collect_timeout (khs_node *c_node);
void khs_restart_idle_timeout (khs_node *c_node);
void khs_set_collect_period (khs_node *c_node);
uint8_t khs_check_collect_period (khs_node *c_node);
void khs_set_data_report_period (khs_node *c_node);
uint8_t khs_check_data_report_period (khs_node *c_node);
uint8_t khs_check_scheduling_period (khs_node *c_node);
uint8_t khs_check_wait_time (khs_node *c_node);
void khs_set_wait_time (khs_node *c_node, clock_time_t interval_wait);
void khs_update_scheduling_period (khs_node *c_node, uint8_t m);
uint8_t khs_check_sampling (khs_node *c_node);

// Receiving procedure of reliable unicast messages
void khs_rucast_recv (khs_node *c_node, struct runicast_conn *rucast, void *msg,
					  const linkaddr_t *from, uint8_t seqno);

// Sending procedure of reliable unicast messages
void khs_rucast_send (khs_node *c_node, struct runicast_conn *rucast, const linkaddr_t *to, uint8_t seqno,
					            uint8_t retransmissions);

// Timeout procedure of reliable unicast messages
void khs_rucast_timedout (khs_node *c_node, struct runicast_conn *rucast,
	                      const linkaddr_t *to, uint8_t retransmissions);

// Receiving procedure of broadcast messages
void khs_bcast_recv (khs_node *c_node, struct broadcast_conn *bcast,
				     khs_broadcast_msg *msg, const linkaddr_t *sender);

// Sending procedure of broadcast messages
void khs_bcast_send (khs_node *c_node, struct broadcast_conn *bcast);

// Helper C function
void update_parent_collect_app (khs_node *c_node); //Update parent node in collect-app.

uint8_t khs_fire_dataset (khs_node *c_node);
void khs_store_msg (khs_node *c_node,  uint8_t msg_id, const linkaddr_t *to, uint8_t seqno);
void khs_clear_out_msg_queue (khs_node *c_node);

#endif /* KANG_HYBRID_SWITCHING_H_ */
