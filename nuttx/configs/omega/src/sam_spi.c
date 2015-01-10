/************************************************************************************
 * configs/omega/src/up_spi.c
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "sam_gpio.h"
#include "sam_spi.h"
#include "omega.h"

#if defined(CONFIG_SAM34_SPI0)

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG too) */

#undef SPI_DEBUG   /* Define to enable debug */
#undef SPI_VERBOSE /* Define to enable verbose debug */

#ifdef SPI_DEBUG
#  define spidbg  lldbg
#  ifdef SPI_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  undef SPI_VERBOSE
#  define spidbg(x...)
#  define spivdbg(x...)
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

FAR struct spi_dev_s *g_spi = 0;

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select PIO pins for the omega board.
 *
 ************************************************************************************/

void sam_spiinitialize(void)
{
#ifdef CONFIG_SAM34_SPI0
  if (g_spi != 0)
    {
      return;
    }

#ifdef CONFIG_MTD_AT25
  /* The AT25 serial FLASH connects using NPCS1 */

  sam_configgpio(GPIO_AT25_NPCS1);
#endif

#ifdef CONFIG_OMEGA_RF
  /* The RF connects using NPCS2 */

  sam_configgpio(GPIO_OMEGA_RF_NPCS2);
#endif

  g_spi = up_spiinitialize(0);
  if (!g_spi)
    {
      fdbg("ERROR: Failed to initialize SPI port 0\n");
    }
#endif
}

/****************************************************************************
 * Name:  sam_spi[0|1]select, sam_spi[0|1]status, and sam_spi[0|1]cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They
 *   include:
 *
 *   o sam_spi[0|1]select is a functions tomanage the board-specific chip selects
 *   o sam_spi[0|1]status and sam_spi[0|1]cmddata:  Implementations of the status
 *     and cmddata methods of the SPI interface defined by struct spi_ops_
 *     (see include/nuttx/spi/spi.h). All other methods including
 *     up_spiinitialize()) are provided by common SAM3/4 logic.
 *
 *  To use this common SPI logic on your board:
 *
 *   1. Provide logic in sam_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide sam_spi[0|1]select() and sam_spi[0|1]status() functions in your board-
 *      specific logic.  These functions will perform chip selection and
 *      status operations using PIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      sam_spi[0|1]cmddata() functions in your board-specific logic.  This
 *      function will perform cmd/data selection operations using PIOs in
 *      the way your board is configured.
 *   3. Add a call to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

/****************************************************************************
 * Name: sam_spi[0|1]select
 *
 * Description:
 *   PIO chip select pins may be programmed by the board specific logic in
 *   one of two different ways.  First, the pins may be programmed as SPI
 *   peripherals.  In that case, the pins are completely controlled by the
 *   SPI driver.  This method still needs to be provided, but it may be only
 *   a stub.
 *
 *   An alternative way to program the PIO chip select pins is as a normal
 *   PIO output.  In that case, the automatic control of the CS pins is
 *   bypassed and this function must provide control of the chip select.
 *   NOTE:  In this case, the PIO output pin does *not* have to be the
 *   same as the NPCS pin normal associated with the chip select number.
 *
 * Input Parameters:
 *   devid - Identifies the (logical) device
 *   selected - TRUE:Select the device, FALSE:De-select the device
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_SPI0
void sam_spi0select(enum spi_dev_e devid, bool selected)
{
#ifdef CONFIG_MTD_AT25
  /* The AT25 serial FLASH connects using NPCS1 */

  if (devid == SPIDEV_FLASH)
    {
      sam_gpiowrite(GPIO_AT25_NPCS1, !selected);
    }
#endif

#ifdef CONFIG_OMEGA_RF
  /* The OMEGA_RF modem connects using NPCS2 */

  if (devid == SPIDEV_WIRELESS)
    {
      sam_gpiowrite(GPIO_OMEGA_RF_NPCS2, !selected);
    }
#endif
}
#endif

/****************************************************************************
 * Name: sam_spi[0|1]status
 *
 * Description:
 *   Return status information associated with the SPI device.
 *
 * Input Parameters:
 *   devid - Identifies the (logical) device
 *
 * Returned Values:
 *   Bit-encoded SPI status (see include/nuttx/spi/spi.h.
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_SPI0
uint8_t sam_spi0status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  return 0;
}
#endif

#endif /* CONFIG_SAM34_SPI0 */
