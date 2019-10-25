#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#include "contiki.h"
#include "dev/leds.h"
#include "net/rime/rime.h"
#include "dev/button-sensor.h"

#define HEADER "RTST"
#define PACKET_SIZE 20
PROCESS(cp_init_process, NULL);
PROCESS(button_process, NULL);

AUTOSTART_PROCESSES(&cp_init_process, &button_process);

static const struct abc_callbacks abc_call = {};
struct etimer e;
static struct abc_conn abc;
PROCESS_THREAD(cp_init_process, ev, data){
    PROCESS_BEGIN();
    etimer_set(&e, CLOCK_SECOND);
    uint8_t dat = 0xAA;
  abc_open(&abc, 9345, &abc_call);
    while(1){
        PROCESS_WAIT_EVENT();
        if(data == &e){
        packetbuf_copyfrom(HEADER, sizeof(HEADER));
        ((char *)packetbuf_dataptr())[sizeof(HEADER)] = dat;
        /* send arbitrary data to fill the packet size */
        packetbuf_set_datalen(PACKET_SIZE);
            abc_send(&abc);
            PRINTF("HALLO WELT\n");
            etimer_reset(&e);
            /*leds_toggle(LEDS_ALL);*/
        }
    }
    PROCESS_END();
}

PROCESS_THREAD(button_process, ev, data){
    PROCESS_BEGIN();
    SENSORS_ACTIVATE(button_sensor);
    SENSORS_ACTIVATE(button_sensor2);
    while(1){
    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event);
        if(data == &button_sensor){
            PRINTF("Button\n");
            leds_toggle(LEDS_GREEN);
        }else if (data == &button_sensor2){
            PRINTF("Button2\n");
            leds_toggle(LEDS_RED);
        }else{
            PRINTF("OTHER SENSOR\n");
        }
    }
    PROCESS_END();
}
