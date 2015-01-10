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

#include "chip/sam_eefc.h"
#include "chip/sam_rstc.h"

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

#define SAM_EEFC_BASE     0x00400000
#define SAM_EEFC_PAGESIZE 512

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

void flash_eraseblock(uint32_t page, size_t npages)
{
	uint32_t farg = page;
	
	switch(npages) {
	case 4:
		farg |= 0;
		break;
	case 8:
		farg |= 1;
		break;
	case 16:
		farg |= 2;
		break;
	case 32:
		farg |= 3;
		break;
	default:
		DEBUGASSERT(0);
		return;
	}

	putreg32(EEFC_FCR_FCMD_EPA | EEFC_FCR_FARG(farg) | EEFC_FCR_FKEY_PASSWD, SAM_EEFC0_FCR);
	while((getreg32(SAM_EEFC0_FSR) & EEFC_FSR_FRDY) == 0) {}
}

void flash_writeblock(const void* data, uint32_t page, size_t npages, int erase)
{
	size_t i;
	for (i = 0; i < npages; i++, page++) {
		flash_writepage(data, SAM_EEFC_PAGESIZE, page, erase);
		data = (const char*)data + SAM_EEFC_PAGESIZE;
	}
}

void flash_writepage(const void* data, size_t size, uint32_t page, int erase)
{
	uint32_t regval;
	uint32_t cmd = EEFC_FCR_FARG(page) | EEFC_FCR_FKEY_PASSWD;
	size_t i;
	if (erase)
		cmd |= EEFC_FCR_FCMD_EWP;
	else
		cmd |= EEFC_FCR_FCMD_WP;
	
	regval = getreg32(SAM_EEFC0_FMR);
	regval &= ~EEFC_FMR_FWS_MASK;
	regval |= EEFC_FMR_FWS(6);
	putreg32(regval, SAM_EEFC0_FMR);
	
	uint32_t *dst = (uint32_t*)(page * SAM_EEFC_PAGESIZE + SAM_EEFC_BASE);
	const uint32_t *src = (const uint32_t*)(data);
	for (i = 0; i < size; i += sizeof(uint32_t), dst++, src++)
		*dst = *src;
	for (     ; i < SAM_EEFC_PAGESIZE; i += sizeof(uint32_t), dst++)
		*dst = 0xFFFFFFFF;
	
	putreg32(cmd, SAM_EEFC0_FCR);
	while((getreg32(SAM_EEFC0_FSR) & EEFC_FSR_FRDY) == 0) {}
}