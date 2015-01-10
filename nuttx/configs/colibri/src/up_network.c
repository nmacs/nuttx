/****************************************************************************
 * board/up_network.c
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_KS8851)

#include <debug.h>
#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include "up_arch.h"
#include "up_internal.h"
#include "lm_gpio.h"

extern int ks8851_initialize(int intf, FAR struct spi_dev_s *spi, uint32_t spiid, int irq);

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_netinitialize
 ****************************************************************************/

void up_netinitialize(void)
{
  int ret;
  /* Configure chip select and interrupt pins */
  lm_configgpio(GPIO_SSI_CS_ETH);
  lm_configgpio(GPIO_ETH_INTRN);

  FAR struct spi_dev_s *spi = up_spiinitialize(1);

  if (!spi) {
    nlldbg("Fail to init SPI bus 1\n");
    return;
  }

  ret = ks8851_initialize(0, spi, 0, LM_IRQ_GPIOF_1);
  if (ret) {
    nlldbg("Fail to init ks8851 device\n");
    return;
  }
}

#endif /* CONFIG_NET && CONFIG_NET_DM90x0 */
