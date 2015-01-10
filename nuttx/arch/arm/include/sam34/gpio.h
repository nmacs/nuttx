/****************************************************************************************
 * arch/arm/include/sam34/gpio.h
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_SAM34_GPIO_H
#define __ARCH_ARM_INCLUDE_SAM34_GPIO_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/


/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

#define GPIO_CONFIG      sam_configgpio
#define GPIO_WRITE       sam_gpiowrite
#define GPIO_READ        sam_gpioread
#define GPIO_IRQ         sam_gpioirq
#define GPIO_IRQ_ENABLE  sam_gpioirqenable
#define GPIO_IRQ_DISABLE sam_gpioirqdisable

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************************
 * Public Function Prototypes
 ****************************************************************************************/

int sam_configgpio(uint32_t cfgset);
void sam_gpiowrite(uint32_t pinset, bool value);
bool sam_gpioread(uint32_t pinset);
void sam_gpioirq(uint32_t pinset);
void sam_gpioirqenable(int irq);
void sam_gpioirqdisable(int irq);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_SAM34_GPIO_H */
