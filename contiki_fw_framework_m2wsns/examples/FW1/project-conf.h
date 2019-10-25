/**
  * \file
  *        Configuration for M2DaGNoS with Low-duty-cycling and WuRx support
  * \author
  *         Juan Aranda <juan-aranda@javeriana.edu.co>
  *         Date: 25/02/2019
*/

#ifndef __PROJECT_CONF_H__
#define __PROJECT_CONF_H__

  /*MultiModal Switching Mechanism*/
  #define M2DaGNoS 1
  #define eHNS 0

  /*WuRx support*/
  #ifndef WURX_CONF_ENABLED
    #define WURX_CONF_ENABLED 0 //ON, only if the FW works as WuRx
  #endif
  #ifndef    WURx
    #define  WURx 1 //ON, only if the FW has attached a WuRx
  #endif

  /*FW support*/
  #ifndef FW1_ACTIVE
    #define FW1_ACTIVE 1
  #endif

  /*FW MULTIHOPS support*/
  #ifndef FW1_MULTIHOPS
    #define FW1_MULTIHOPS 1
  #endif

  /*For Delay measurement using triggering signals and oscilloscope*/
  #ifndef TRIGGERING
    #define TRIGGERING 1
  #endif

  /*For Power consumption measurement using PowerTrace module*/
  #ifndef POWER_TRACE
    #define POWER_TRACE 0
  #endif

  /* Netstack layers */
  #undef NETSTACK_CONF_RDC
  #if M2DaGNoS
    #if WURx
      #define NETSTACK_CONF_RDC      wrimac_broadcast_driver //For 2R-MAC  // nullrdc_driver //for de sink node
      #if WURX_CONF_ENABLED
        #define NETSTACK_CONF_MAC     nullmac_driver
      #else
        #define NETSTACK_CONF_MAC     csma_driver
      #endif
    #else
      #ifndef CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION
        #define CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION 1 //ONLY for sink ContikiMAC
      #endif
      #ifndef WITH_FAST_SLEEP
        #define WITH_FAST_SLEEP              1 //ONLY for sink ContikiMAC
      #endif
      #define NETSTACK_CONF_RDC     contikimac_driver
      #define NETSTACK_CONF_MAC     csma_driver
      #define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE 8 //Change the rate at which the node check for radio activity in the channel
      //#define CC2520_CONF_CHANNEL 26 //RF_CHANNEL = Radio Fequency Channel
    #endif
  #elif eHNS
    #define NETSTACK_CONF_RDC     contikimac_driver
    #define NETSTACK_CONF_MAC     csma_driver
    #define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE 8 //Change the rate at which the node check for radio activity in the channel
    //#define CC2520_CONF_CHANNEL 26 //RF_CHANNEL = Radio Fequency Channel
  #else //HNS
    #define NETSTACK_CONF_RDC     nullrdc_driver
    #define NETSTACK_CONF_MAC    csma_driver//nullmac_driver
  #endif

  /*Implicit Network Time Synchronization */
  #undef TIMESYNCH_CONF_ENABLED
  #if M2DaGNoS
    #if WURX_CONF_ENABLED
      #define TIMESYNCH_CONF_ENABLED 0
    #else
      #define TIMESYNCH_CONF_ENABLED 1
    #endif
  #elif eHNS
    #define TIMESYNCH_CONF_ENABLED 0
  #else //HNS
    #define TIMESYNCH_CONF_ENABLED 0
  #endif

#endif /* __PROJECT_CONF_H__ */
