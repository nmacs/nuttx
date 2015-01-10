/************************************************************************************
 * configs/omega/src/sam_flash.c
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

#include "chip/sam_gpbr.h"

#include "arch/board/board.h"

#include "up_arch.h"
#include "omega.h"


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
#  define message(...) (void)
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

void gpbr_save(const void *data, size_t size, off_t offset)
{
	uint32_t addr = SAM_GPBR_BASE + offset;
	while (size != 0) {
		putreg32(*(const uint32_t*)data, addr);
		data = (const char*)data + sizeof(uint32_t);
		addr += sizeof(uint32_t);
		size -= sizeof(uint32_t);
	}
}

void gpbr_load(void *data, size_t size, off_t offset)
{
	uint32_t addr = SAM_GPBR_BASE + offset;
	while (size != 0) {
		*(uint32_t*)data = getreg32(addr);
		data = (char*)data + sizeof(uint32_t);
		addr += sizeof(uint32_t);
		size -= sizeof(uint32_t);
	}
}