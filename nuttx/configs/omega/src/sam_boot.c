/************************************************************************************
 * configs/omega/src/sam_boot.c
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mmcsd.h>
#include <sys/mount.h>

#include "chip/sam_pmc.h"
#include "chip/sam_rstc.h"
#include "chip/sam_gpbr.h"
#include "sam_periphclks.h"

#include "arch/board/board.h"
#include "arch/board/metrology.h"

#include "up_arch.h"
#include "omega.h"
#include "sam_gpio.h"


/************************************************************************************
 * Definitions
 ************************************************************************************/

#ifdef CONFIG_DEBUG
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define message(...) lldbg(__VA_ARGS__)
#  else
#    define message lldbg
#  endif
#else
#  define message(...)
#endif

#ifdef CONFIG_NET_SLIP
#  define SLIP_DEVNO 0
#  define NET_DEVNAME "sl0"

#  ifndef CONFIG_NET_SLIPTTY
#    define CONFIG_NET_SLIPTTY "/dev/ttyS1"
#  endif
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_boardinitialize
 *
 * Description:
 *   All SAM3CM architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void sam_boardinitialize(void)
{
  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak function
   * sam_spiinitialize() has been brought into the link.
   */

#ifdef CONFIG_SAM34_SPI0
  if (sam_spiinitialize)
    {
      sam_spiinitialize();
    }
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_led_initialize();
#endif
}

void board_initialize(void)
{
#ifdef CONFIG_SAM34_SLCDC
  {
    int ret = sam_slcdc_initialize();
    if (ret < 0)
      {
        message("Failed to initialize the LCD: %d\n", errno);
      }
  }
#endif

#ifdef CONFIG_SAM34_METROLOGY
  {
    int ret = sam_metrology_initialize();
    if (ret < 0)
      {
        message("Failed to initialize the Metrology: %d\n", errno);
      }
  }
#endif	

#if defined(CONFIG_SAM34_SPI0) && defined(CONFIG_MTD_AT25)
  {
    int ret = sam_at25_initialize();
    if (ret == 0)
      {
        ret = mount("/dev/mtdblock0", "/flash0", "fixfs", 0, 0);
        if (ret != 0)
          {
            message("Fail to mount AT25 to /flash0: %d\n", errno);
          }
      }
    else
      {
        message("AT25 initialization failed: %d\n", errno);
      }
  }
#endif

#if defined(CONFIG_MTD_AT24XX)
  {
    int ret = sam_at24_initialize();
    if (ret != 0)
      {
        message("AT24 initialization failed: %d\n", errno);
      }

    ret = bchdev_register("/dev/mtdblock1", "/dev/eeprom0", 0);
    if (ret != 0)
      {
        message("Fail to bind /dev/eeprom0 to /dev/mtdbloc1: %d\n", errno);
      }
  }
#endif

#ifdef CONFIG_NET_SLIP
  {
    int ret = slip_initialize(SLIP_DEVNO, CONFIG_NET_SLIPTTY);
    if (ret < 0)
      {
        message("SLIP initialization failed: %d\n", errno);
      }
  }
#endif

#ifdef CONFIG_OMEGA_RF
  {
    int ret = sam_rf_initialize();
    if (ret < 0)
      {
        message("OMEGA RF initialization failed: %d\n", errno);
      }
  }
#endif
}

void sam_reset(void)
{
	putreg32(RSTC_CR_PROCRST | RSTC_CR_KEY, SAM_RSTC_CR);
}
