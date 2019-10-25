/**
 * RI-MAC for broadcast Mode 11 April 2018
 * Sends a single broadcast wake-up signal to collect data
 */
 /**
   * \Modified file
   *        Add support to FeuerWhere nodes (FW-node)
   * \author
   *         Juan Aranda <juan-aranda@javeriana.edu.co>
   *         Date: 26/02/2019
 */

#ifndef WRIMAC_BROADCAST_H_
#define WRIMAC_BROADCAST_H_

#include "sys/rtimer.h"
#include "net/mac/rdc.h"
#include "dev/radio.h"

#define WAKE_UP_PERIOD   26//75//60//120//0.1 * CLOCK_SECOND //60//40// //5 // 0.001 * CLOCK_SECOND (1ms)

extern const struct rdc_driver wrimac_broadcast_driver;

int wrimac_interrupt(void);

#endif /* WMAC_H_ */
