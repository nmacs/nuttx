/****************************************************************************
 * configs/lm4f120-launchpad/src/lm4f_leds.c
 * arch/arm/src/board/lm4f_leds.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "lm_gpio.h"
#include "colibri.h"

/* CONFIG_DEBUG_LEDS enables debug output from this file (needs CONFIG_DEBUG
 * with CONFIG_DEBUG_VERBOSE too)
 */

#ifdef CONFIG_DEBUG_LEDS
#  define leddbg  lldbg
#  define ledvdbg llvdbg
#else
#  define leddbg(x...)
#  define ledvdbg(x...)
#endif

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_LEDS
#  define led_dumpgpio(m) lm_dumpgpio(LED_GPIO, m)
#else
#  define led_dumpgpio(m)
#endif

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
 * Name: lm4f_ledinit
 *
 * Description:
 *   Called to initialize the on-board LEDs.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void lm4c_ledinit(void)
{
  leddbg("Initializing\n");
#ifdef LED_IDLE
  lm_configgpio(GPIO_LED_CPU);
#endif
}

/****************************************************************************
 * Name: board_led_on
 ****************************************************************************/

void board_led_on(int led)
{
  switch (led)
    {
      /* All components stay off until the file initialization step */
#ifdef LED_IDLE
      case LED_IDLE:
        lm_gpiowrite(GPIO_LED_CPU, false);
        break;
#endif
    }
}

/****************************************************************************
 * Name: board_led_off
 ****************************************************************************/

void board_led_off(int led)
{
  switch (led)
    {
      /* All components stay off until the file initialization step */
#ifdef LED_IDLE
      case LED_IDLE:
        lm_gpiowrite(GPIO_LED_CPU, true);
        break;
#endif
    }
}

#endif /* CONFIG_ARCH_LEDS */
