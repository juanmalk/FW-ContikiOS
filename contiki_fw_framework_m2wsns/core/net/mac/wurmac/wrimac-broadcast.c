/**
 * RI-MAC for broadcast Mode 11 April 2018
 * Sends a single broadcast wake-up signal to collect data
 */
 /**
   * \Modified file
   *        Add support to FeuerWhere nodes (FW-node)
   * \author
   *         Juan Aranda <juan-aranda@javeriana.edu.co>
   *         Date: 27/08/2019
 */


#include "net/mac/mac-sequence.h"
#include "net/mac/wurmac/wrimac-broadcast.h"
#include "dev/cc2520/cc2520.h"
#include "dev/button-sensor.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/netstack.h"
#include "wur.h"
#include "net/rime/rimestats.h"
#include <string.h>
#include <stdio.h>
#include "dev/leds.h"
#include "sys/rtimer.h"
#include "node-id.h"

#if CONTIKI_TARGET_ZOUL
#include "dev/gpio.h"
#endif


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

#ifdef NULLRDC_CONF_ADDRESS_FILTER
#define NULLRDC_ADDRESS_FILTER NULLRDC_CONF_ADDRESS_FILTER
#else
#define NULLRDC_ADDRESS_FILTER 1
#endif /* NULLRDC_CONF_ADDRESS_FILTER */

#ifndef NULLRDC_802154_AUTOACK
#ifdef NULLRDC_CONF_802154_AUTOACK
#define NULLRDC_802154_AUTOACK NULLRDC_CONF_802154_AUTOACK
#else
#define NULLRDC_802154_AUTOACK 0
#endif /* NULLRDC_CONF_802154_AUTOACK */
#endif /* NULLRDC_802154_AUTOACK */

#ifndef NULLRDC_802154_AUTOACK_HW
#ifdef NULLRDC_CONF_802154_AUTOACK_HW
#define NULLRDC_802154_AUTOACK_HW NULLRDC_CONF_802154_AUTOACK_HW
#else
#define NULLRDC_802154_AUTOACK_HW 0
#endif /* NULLRDC_CONF_802154_AUTOACK_HW */
#endif /* NULLRDC_802154_AUTOACK_HW */

#if NULLRDC_802154_AUTOACK
#include "sys/rtimer.h"
#include "dev/watchdog.h"

#ifdef NULLRDC_CONF_ACK_WAIT_TIME
#define ACK_WAIT_TIME NULLRDC_CONF_ACK_WAIT_TIME
#else /* NU#if TIMESYNCH_CONF_ENABLEDLLRDC_CONF_ACK_WAIT_TIME */
#define ACK_WAIT_TIME                      RTIMER_SECOND / 2500
#endif /* NULLRDC_CONF_ACK_WAIT_TIME */
#ifdef NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME
#define AFTER_ACK_DETECTED_WAIT_TIME NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME
#else /* NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME */
#define AFTER_ACK_DETECTED_WAIT_TIME       RTIMER_SECOND / 1500
#endif /* NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME */
#endif /* NULLRDC_802154_AUTOACK */

#ifdef NULLRDC_CONF_SEND_802154_ACK
#define NULLRDC_SEND_802154_ACK NULLRDC_CONF_SEND_802154_A MAC layerCK
#else /* NULLRDC_CONF_SEND_802154_ACK */
#define NULLRDC_SEND_802154_ACK 0
#endif /* NULLRDC_CONF_SEND_802154_ACK */

#if NULLRDC_SEND_802154_ACK
#include "net/mac/frame802154.h"
#endif /* NULLRDC_SEND_802154_ACK */

#define ACK_LEN 3
//#define ROOT_ADDR 1

/*---------------------------------------------------------------------------*/
PROCESS(wur_broadcast_process, "Wake-up signal handler process");
/*---------------------------------------------------------------------------*/
static volatile uint8_t flag_WUPSIG = 0;
static struct etimer timer_wait_just_sent;
//static volatile uint8_t radio_is_on = 0;
/*---------------------------------------------------------------------------*/
static void
on(void)
{
    NETSTACK_RADIO.on();
    //radio_is_on = 1;
    //LOG_DBG("WRIRDC: Turn ON the radio module!\n");
}
/*---------------------------------------------------------------------------*/
static void
off(void)
{
    NETSTACK_RADIO.off();
    //radio_is_on = 0;
    //LOG_DBG("WRIRDC: Turn OFF the radio module!!\n");
}
/*---------------------------------------------------------------------------*/
static void wur_trigger_tx() {
    LOG_DBG("WRIRDC: Triggering Signal to MR!\n");
    #if CONTIKI_TARGET_ZOUL
      GPIO_SET_PIN(EXAMPLE_PORT_BASE, EXAMPLE_PIN_MASK);
      leds_on(LEDS_RED);
      clock_delay(1000);
      leds_off(LEDS_RED);
      GPIO_CLR_PIN(EXAMPLE_PORT_BASE, EXAMPLE_PIN_MASK);
    #else
      wur_set_tx();
      leds_on(LEDS_RED);
      //cc2520_off();
      clock_delay(1000);
      //clock_delay(100);
      //cc2520_on();
      leds_off(LEDS_RED);
      wur_clear_tx();
    #endif
}

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
wrimac_interrupt(void)
{
  process_poll(&wur_broadcast_process);
  PRINTF("***INTERRUPT : WRIRDC - Trigger signal received by MR!\n");
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
send_one_packet(mac_callback_t sent, void *ptr)
{
  on();
  int ret;
  int last_sent_ok = 0;

  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &linkaddr_node_addr);

#if NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW
  //packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 0);
#endif /* NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW */

  if(NETSTACK_FRAMER.create() < 0) {
    /* Failed to allocate space for headers */
    LOG_DBG("WRIRDC: Send failed, too large header!\n");
    ret = MAC_TX_ERR_FATAL;
  } else {
#if NULLRDC_802154_AUTOACK
    int is_broadcast;

    NETSTACK_RADIO.prepare(packetbuf_hdrptr(), packetbuf_totlen());

    is_broadcast = packetbuf_holds_broadcast();

    if(NETSTACK_RADIO.receiving_packet() ||
       (!is_broadcast && NETSTACK_RADIO.pending_packet())) {
      ret = MAC_TX_COLLISION;
      off();/*****************************/
      LOG_DBG("WRIRDC: MAC_TX_COLLISION, receiving!\n");
    } else {
      if(!is_broadcast) {
        RIMESTATS_ADD(reliabletx);
      }

      switch(NETSTACK_RADIO.transmit(packetbuf_totlen())) {
      case RADIO_TX_OK:
        if(is_broadcast) {
          off();
          ret = MAC_TX_OK;
          LOG_DBG("+WRIRDC: MAC_TX_OK! %u\n", etimer_expired(&timer_wait_just_sent));
          etimer_restart(&timer_wait_just_sent);
          LOG_DBG("-WRIRDC: MAC_TX_OK! %u\n", etimer_expired(&timer_wait_just_sent));
        } else {
          off(); /*this turns off the radio right after sending the packet without waiting for ACK*/
          ret = MAC_TX_NOACK;
          LOG_DBG("WRIRDC: MAC_TX_NOACK!\n");
        }
        break;
      case RADIO_TX_COLLISION:
        ret = MAC_TX_COLLISION;
        off();
        LOG_DBG("WRIRDC: MAC_TX_COLLISION, transmitting!\n");
        break;
      default:
        ret = MAC_TX_ERR;
        off();
        LOG_DBG("WRIRDC: MAC_TX_ERR!\n");
        break;
      }
    }
#else /* ! NULLRDC_802154_AUTOACK */

    switch(NETSTACK_RADIO.send(packetbuf_hdrptr(), packetbuf_totlen())) {
    case RADIO_TX_OK:
      ret = MAC_TX_OK;
      break;
    case RADIO_TX_COLLISION:
      ret = MAC_TX_COLLISION;
      break;
    case RADIO_TX_NOACK:
      ret = MAC_TX_NOACK;
      break;
    default:
      ret = MAC_TX_ERR;
      break;
    }
#endif /* ! NULLRDC_802154_AUTOACK */
  }
  if(ret == MAC_TX_OK) {
    last_sent_ok = 1;
  }
  mac_call_sent_callback(sent, ptr, ret, 1);

  return last_sent_ok;
}
/*---------------------------------------------------------------------------*/
static void
send_packet(mac_callback_t sent, void *ptr)
{
  #if WURX_CONF_ENABLED
    // Do nothing just WuRx
  #else
      send_one_packet(sent, ptr);
  #endif /* WURX_CONF_ENABLED */
}

/*---------------------------------------------------------------------------*/
static void
send_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
  #if WURX_CONF_ENABLED
    // Do nothing just WuRx
  #else
    LOG_DBG("WRIRDC: Send a packet list!\n");
    while(buf_list != NULL) {
      /* We backup the next pointer, as it may be nullified by
       * mac_call_sent_callback() */
      struct rdc_buf_list *next = buf_list->next;
      int last_sent_ok;

      queuebuf_to_packetbuf(buf_list->buf);
      last_sent_ok = send_one_packet(sent, ptr);

      /* If packet transmission was not successful, we should back off and let
       * upper layers retransmit, rather than potentially sending out-of-order
       * packet fragments. */
      if(!last_sent_ok) {
        return;
      }
      buf_list = next;
    }
  #endif /* WURX_CONF_ENABLED */
}

/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{
  LOG_DBG("WRIRDC: Callback for getting notified of incoming packet!\n");

  #if WURX_CONF_ENABLED
    wur_trigger_tx(); // trigger the WURPSIG to the MR's MCU
  #else

  #if NULLRDC_SEND_802154_ACK
    int original_datalen;
    uint8_t *original_dataptr;

    original_datalen = packetbuf_datalen();
    original_dataptr = packetbuf_dataptr();
  #endif

  #if NULLRDC_802154_AUTOACK
    if(packetbuf_datalen() == ACK_LEN) {
    	    /* Ignore ack packets */
        LOG_DBG("WRIRDC: Ignored ack\n");
    } else
  #endif /* NULLRDC_802154_AUTOACK */
    if(NETSTACK_FRAMER.parse() < 0) {
      LOG_DBG("WRIRDC: Failed to parse %u\n", packetbuf_datalen());
  #if NULLRDC_ADDRESS_FILTER
    } else if(!linkaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                                           &linkaddr_node_addr) &&
              !packetbuf_holds_broadcast()) {
      LOG_DBG("WRIRDC: Not for us\n");
  #endif /* NULLRDC_ADDRESS_FILTER */
    } else {
      int duplicate = 0;

  #if NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW
  #if RDC_WITH_DUPLICATE_DETECTION
      /* Check for duplicate packet. */
      duplicate = mac_sequence_is_duplicate();
      if(duplicate) {
        /* Drop the packet. */
        LOG_DBG("WRIRDC: Drop duplicate link layer packet %u\n",
               packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO));
      } else {
        mac_sequence_register_seqno();
      }
  #endif /* RDC_WITH_DUPLICATE_DETECTION */
  #endif /* NULLRDC_802154_AUTOACK */

      if(!duplicate) {
        //on();
        NETSTACK_MAC.input();
        LOG_DBG("WRIRDC: NOT duplicate packet received!\n");
      }
    }
  #endif /* WURX_CONF_ENABLED */

}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
  #if WURX_CONF_ENABLED
    PRINTF("WRIRDC: Initialized as WuRx! \n");
    #if CONTIKI_TARGET_ZOUL
      printf("TARGET_ZOUL1!!!\n");
      /* The masks below converts the Port number and Pin number to base and mask values */
      #define EXAMPLE_PORT_BASE  GPIO_PORT_TO_BASE(GPIO_A_NUM)
      #define EXAMPLE_PIN_MASK   GPIO_PIN_MASK(5)

      /* We tell the system the application will drive the pin */
      GPIO_SOFTWARE_CONTROL(EXAMPLE_PORT_BASE, EXAMPLE_PIN_MASK);

      /* And set as output, starting low */
      GPIO_SET_OUTPUT(EXAMPLE_PORT_BASE, EXAMPLE_PIN_MASK);
      GPIO_SET_PIN(EXAMPLE_PORT_BASE, EXAMPLE_PIN_MASK);
    #else
    wur_init();
    #endif
  #else
    PRINTF("WRIRDC: Initialized, %d!\n", CLOCK_SECOND);
    process_start(&wur_broadcast_process, NULL);
  #endif /* WURX_CONF_ENABLED */
  on();
}
/*---------------------------------------------------------------------------*/
static int
turn_on(void)
{
  LOG_DBG("WRIRDC: Turn the Radio ON t_on!\n");
  return NETSTACK_RADIO.on();
}
/*---------------------------------------------------------------------------*/
static int
turn_off(int keep_radio_on)
{
  if(keep_radio_on) {
    return NETSTACK_RADIO.on();
  } else {
    LOG_DBG("WRIRDC: Turn the Radio OFF t_off!\n");
    return NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static unsigned short
channel_check_interval(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver wrimac_broadcast_driver = {
  "wrimac-broadcast",
  init,
  send_packet,
  send_list,
  packet_input,
  turn_on,
  turn_off,
  channel_check_interval,
};

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(wur_broadcast_process, ev, data)
{
  static struct etimer timeout;

  PROCESS_BEGIN();

  LOG_DBG("WRIRDC: wur_broadcast_process began! \n");
  SENSORS_ACTIVATE(wur_sensor); //for enabling wake-up signal on MR by jmalk
  LOG_DBG("WRIRDC: Sensors activated! \n");

  /*Wait a time for activiting the MCU-WuRx hardware interface*/
  etimer_set(&timeout, WAKE_UP_PERIOD);
  etimer_set(&timer_wait_just_sent, WAKE_UP_PERIOD / WAKE_UP_PERIOD);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timeout));
  off();

  while(1)
  {
    /* Wait to receive WURPSIG (interrupt signal via GPIO) from WuRx HW */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL && etimer_expired(&timer_wait_just_sent) == 1);
    /* Turn on radio to receive the data packet from other motes */
      //flag_WUPSIG = 1;
      PRINTF("**WRIRDC: Radio ON to request data packet! \n");
      on();
      etimer_restart(&timeout); // in average (20) 160ms radio ON window time
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timeout));
      off();
      //flag_WUPSIG = 0;
      PRINTF("**WRIRDC: Radio OFF to receive a data packet! \n");

  };

  PROCESS_END();
}
