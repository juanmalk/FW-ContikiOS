#ifndef DS2411_CONF_H
#define DS2411_CONF_H

/* 1-wire is at p2.5 */
#define PIN BV(5)

#define PIN_INIT() {\
  P2DIR &= ~PIN;		/* p2.5 in, resistor pull high */\
  P2OUT &= ~PIN;		/* p2.5 == 0 but still input */\
}

/* Set 1-Wire low or high. */
#define OUTP_0() (P2DIR |=  PIN) /* output and p2.5 == 0 from above */
#define OUTP_1() (P2DIR &= ~PIN) /* p2.5 in, external resistor pull high */

/* Read one bit. */
#define INP()    (P2IN & PIN)

/*
 * Delay for u microseconds on a MSP430 at 2.4756MHz.
 *
 * The loop in clock_delay consists of one add and one jnz, i.e 3
 * cycles.
 *
 * 3 cycles at 2.4756MHz ==> 1.2us = 6/5us.
 *
 * Call overhead is roughly 7 cycles and the loop 3 cycles, to
 * compensate for call overheads we make 7/3=14/6 fewer laps in the
 * loop.
 *
 * This macro will loose badly if not passed a constant argument, it
 * relies on the compiler doing the arithmetic during compile time!!
 * TODO: Fix above comment to be correct - below code is modified for 4Mhz
 */
#define udelay(u) clock_delay((u*8 - 14)/6)

/*
 * Where call overhead dominates, use a macro!
 * Note: modified for 4 Mhz
 */
#define udelay_6() { _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); }

#endif /* DS2411_CONF_H */
