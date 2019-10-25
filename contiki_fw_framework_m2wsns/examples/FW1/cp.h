#ifndef __CP_PLATFORM_H__
#define __CP_PLATFORM_H__

#define PF_PORT_ARRAY \
    {&P1IN, &P1OUT, &P1DIR, &P1SEL, &P1IES, &P1IE, &P1IFG},\
    {&P2IN, &P2OUT, &P2DIR, &P2SEL, &P2IES, &P2IE, &P2IFG}, \
    {&P3IN, &P3OUT, &P3DIR, NULL, NULL, NULL, NULL},\
    {&P4IN, &P4OUT, &P4DIR, NULL, NULL, NULL, NULL},\
    {&P5IN, &P5OUT, &P5DIR, NULL, NULL, NULL, NULL},\
    {&P6IN, &P6OUT, &P6DIR, NULL, NULL, NULL, NULL},\
    {&P7IN, &P7OUT, &P7DIR, NULL, NULL, NULL, NULL},\
    {&P8IN, &P8OUT, &P8DIR, NULL, NULL, NULL, NULL},\
    {&P9IN, &P9OUT, &P9DIR, NULL, NULL, NULL, NULL},\
    {&P10IN, &P10OUT, &P10DIR, NULL, NULL, NULL, NULL},\
    {&P11IN, &P11OUT, &P11DIR, NULL, NULL, NULL, NULL}

#define PF_RADIO_COUNT 3
#define PF_RADIO_0 cc2520_driver

#define PF_ID_CHIP_COUNT 1

#define PF_UART_COUNT 1

#define PF_BATTERY_COUNT 0
#define PF_BATTERY_REFERENCE 2.5
#define PF_BATTERY_RESOLUTION 4096

#define PF_BUTTON_COUNT 2
#define PF_BUTTON_RESOLUTION 100
#define PF_BUTTON_ARRAY {&button_sensor,&P2IN,6,0}, {&button_sensor2, &P2IN,7,0}

#define PF_LED_COUNT 8
#define PF_LED_ARRAY {8, 1, 0}, {8, 3, 0}, {8, 0, 0}, {8, 2, 0}, {8, 4, 0}, {8, 5, 0}, {8, 6, 0}, {8, 7, 0} 

#define PF_TIMER_COUNT PF_BUTTON_COUNT

#define PF_GPIO_COUNT 24
#define PF_GPIO_ARRAY \
    {1,0}, {1,1}, {1,2}, {1,3}, {1,4}, {1,5}, {1,6}, {1,7},\
    {2,0}, {2,1}, {2,2}, {2,3}, {2,4}, {2,5}, {2,6}, {2,7},\
    {4,0}, {4,1}, {4,2}, {4,3}, {4,4}, {4,5}, {4,6},\
    {7,2}

#endif
