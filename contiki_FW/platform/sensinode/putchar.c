/**
 * \file
 *         hardware-specific putchar() routine for sensinode motes
 *
 * \author
 *         George Oikonomou - <oikonomou@users.sourceforge.net>
 */

#include "contiki-conf.h"
#include "dev/uart1.h"

/*---------------------------------------------------------------------------*/
void
putchar(char c)
{
#if SLIP_ARCH_CONF_ENABLE
#define SLIP_END     0300
  static char debug_frame = 0;

  if(!debug_frame) {            /* Start of debug output */
    uart1_writeb(SLIP_END);
    uart1_writeb('\r');     /* Type debug line == '\r' */
    debug_frame = 1;
  }
#endif

  uart1_writeb((char)c);

#if SLIP_ARCH_CONF_ENABLE
  /*
   * Line buffered output, a newline marks the end of debug output and
   * implicitly flushes debug output.
   */
  if(c == '\n') {
    uart1_writeb(SLIP_END);
    debug_frame = 0;
  }
#endif
}
