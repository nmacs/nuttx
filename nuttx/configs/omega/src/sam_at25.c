/****************************************************************************
 * config/omega/src/sam_at25.c
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
#include <nuttx/mtd/mtd.h>

#include "sam_spi.h"
#include "arch/board/board.h"
#include "omega.h"

#ifdef CONFIG_MTD_AT25

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

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

int sam_at25_initialize(void)
{
  FAR struct mtd_dev_s *mtd;
  int ret;

  sam_spiinitialize();

  if (!g_spi)
    {
      fdbg("ERROR: SPI not initialized\n");
      return -ENODEV;
    }

  /* Now bind the SPI interface to the AT25 SPI FLASH driver */

  mtd = at25_initialize(g_spi);
  if (!mtd)
    {
      fdbg("ERROR: Failed to bind SPI port %d to the AT25 FLASH driver\n");
      return -ENODEV;
    }

  /* And finally, use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(AT25_MINOR, mtd);
  if (ret < 0)
    {
      fdbg("ERROR: Failed to initialize the FTL layer: %d\n", ret);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_MTD_AT25 */
