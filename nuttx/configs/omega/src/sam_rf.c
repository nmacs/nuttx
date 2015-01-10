/****************************************************************************
 * config/omega/src/sam_rf.c
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/wireless/omega_rf.h>

#include "sam_spi.h"
#include "sam_gpio.h"
#include "arch/board/board.h"
#include "omega.h"

#ifdef CONFIG_OMEGA_RF

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

static const omega_rf_cfg cfg = {
	.irq_gpio  = GPIO_OMEGA_RF_IRQ,
	.rdy_gpio  = GPIO_OMEGA_RF_RDY,
	.nrst_gpio = GPIO_OMEGA_RF_nRST,
	.spidev    = SPIDEV_WIRELESS,
	.rx_irq    = OMEGA_RF_IRQ,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_at25_initialize
 *
 * Description:
 *   Initialize and configure the AT25 serial FLASH
 *
 ****************************************************************************/

int sam_rf_initialize(void)
{
  int ret;

  sam_spiinitialize();

  if (!g_spi)
    {
      fdbg("ERROR: SPI not initialized\n");
      return -ENODEV;
    }

  /* Now bind the SPI interface to the AT25 SPI FLASH driver */

  ret = omega_rf_initialize(g_spi, &cfg);
  if (ret)
    {
      fdbg("ERROR: Failed to bind SPI port %d to the AT25 FLASH driver\n");
      return ret;
    }

  return OK;
}

#endif /* CONFIG_OMEGA_RF */
