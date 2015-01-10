/****************************************************************************
 * config/omega/src/nsh.c
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <string.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mmcsd.h>

#include "chip/sam_pmc.h"
#include "chip/sam_rstc.h"
#include "sam_periphclks.h"

#include "arch/board/board.h"
#include "arch/board/metrology.h"

#include "up_arch.h"
#include "omega.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PORT and SLOT number probably depend on the board configuration */

#ifdef CONFIG_ARCH_BOARD_OMEGA
#  undef NSH_HAVEUSBDEV
#else
#  error "Unrecognized board"
#  undef NSH_HAVEUSBDEV
#endif

/* Can't support USB features if USB is not enabled */

#ifndef CONFIG_USBDEV
#  undef NSH_HAVEUSBDEV
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#  else
#    define message printf
#  endif
#endif

#ifdef CONFIG_NET_SLIP
#  define SLIP_DEVNO 0
#  define NET_DEVNAME "sl0"

#  ifndef CONFIG_NET_SLIPTTY
#    define CONFIG_NET_SLIPTTY "/dev/ttyS1"
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int nsh_archinitialize(void)
{
  return OK;
}
