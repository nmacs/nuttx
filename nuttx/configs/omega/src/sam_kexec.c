/************************************************************************************
 * configs/omega/src/sam_kexec.c
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "chip.h"
#include "omega.h"
#include "up_arch.h"

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static void disable_irqs(void)
{
  int irq;
  for (irq = SAM_IRQ_NMI; irq < NR_IRQS; irq++)
    up_disable_irq(irq);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

typedef void (*reset_t)(void *args);

int kexec(uint32_t address, void *args)
{
  reset_t reset = (reset_t)(*(uint32_t*)(address + 4) | 1);
  uint32_t stack = *(uint32_t*)address;

  __asm__ __volatile__ (
    "cpsid i"
  );
  disable_irqs();

  __asm__ __volatile__ (
    "mov sp, %0\n\t"
    "mov r0, %1\n\t"
    "bx  %2\n\t" // not return
    : // output
    : "r"(stack), "r"(args), "r"(reset) // input
    : // used registers
  );

  errno = EFAULT;
  return -1;
}