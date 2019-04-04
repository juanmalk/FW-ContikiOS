#ifndef __PROJECT_CONF_H__
#define __PROJECT_CONF_H__

/* Netstack layers */
#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC     contikimac_driver //nullrdc_driver
#define NETSTACK_CONF_MAC     csma_driver //nullmac_driver
//#define NETSTACK_CONF_RDC nullrdc_driver
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE 128 //Change the rate at which the node check for radio activity in the channel
//#define CC2520_CONF_CHANNEL 26 //RF_CHANNEL = Radio Fequency Channel

#undef TIMESYNCH_CONF_ENABLED  // TO ENABLE THE Implicit Network Time Synchronization
#define TIMESYNCH_CONF_ENABLED 1 // TO ENABLE THE Implicit Network Time Synchronization

#endif /* __PROJECT_CONF_H__ */
