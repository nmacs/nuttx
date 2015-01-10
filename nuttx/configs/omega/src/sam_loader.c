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

#define SWITCH_PAGE_NUMBER (1024/512 - 1)

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

void loader_enable_switch(void)
{
	uint32_t value = 0x00000000;
	flash_writepage(&value, sizeof(value), SWITCH_PAGE_NUMBER, 0 /* do not erase */);
}

void loader_disable_switch(void)
{
	flash_writepage(0, 0, SWITCH_PAGE_NUMBER, 1 /* erase */);
}