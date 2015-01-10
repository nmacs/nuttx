/****************************************************************************
 * configs/omega/src/sam_slcdc.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/ascii.h>
#include <nuttx/streams.h>
#include <nuttx/fs/fs.h>
#include <nuttx/lcd/slcd_ioctl.h>
#include <nuttx/lcd/slcd_codec.h>

#include "up_arch.h"
#include "sam_gpio.h"
#include "sam4cm_periphclks.h"
#include "chip/sam4cm_slcdc.h"
#include "chip/sam4cm_pinmap.h"
#include "chip/sam_supc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Define CONFIG_DEBUG_LCD to enable detailed LCD debug output. Verbose debug
 * must also be enabled.
 */

//#ifndef CONFIG_LIB_SLCDCODEC
//#  error This SLCD driver requires CONFIG_LIB_SLCDCODEC
//#endif

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#  undef CONFIG_DEBUG_LCD
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_LCD
#endif

/* The ever-present MIN/MAX macros ******************************************/

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif

#ifndef MAX
#  define MAX(a,b) (a > b ? a : b)
#endif

/* LCD **********************************************************************/
/* LCD characteristics.  The logic in this driver is not portable; it is
 * tailored for the SAM4l Xplained Pro's LED1 module.  However, in an effort
 * to add some reusability to this module, some of the tunable settings are
 * included here as BOARD_ definitions (although they do not appear in the
 * board.h header file.
 */

#define BOARD_SLCD_NCOM        6
#define BOARD_SLCD_SEG_MASK0   0x7F0017F8
#define BOARD_SLCD_SEG_MASK1   0x00000001

/* LCD controller initial contrast setting. */

#define BOARD_INITIAL_CONTRAST (SLCD_MAXCONTRAST / 2)

/* LCD controller timing configuration */

#define BOARD_FRR_PRESC        SLCDC_FRR_PRESC_SCLK_DIV8  /* Clock prescaler */
#define BOARD_FRR_CLOCKDIV     SLCDC_FRR_DIV(4)  /* Clock divider {1..8} */

/* LCD display mode */

#define BOARD_DR_DISPMODE      SLCDC_DR_DISPMODE_NORMAL

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg              dbg
#  define lcdvdbg             vdbg
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* SLCD incoming stream structure */

struct slcd_instream_s
{
  struct lib_instream_s stream;
  FAR const char *buffer;
  ssize_t nbytes;
};

/* Global SLCD state */

struct sam_slcdstate_s
{
  bool initialized;             /* True: Completed initialization sequence */
};

/* Describes one pixel */

struct slcd_pixel_s
{
  uint8_t segment;
  uint8_t com;
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* Character driver methods */

static ssize_t slcd_read(FAR struct file *, FAR char *, size_t);
static ssize_t slcd_write(FAR struct file *, FAR const char *, size_t);
static int slcd_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int slcd_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the driver state structure (there is no retained state information) */

static const struct file_operations g_slcdops =
{
  0,             /* open */
  0,             /* close */
  slcd_read,     /* read */
  slcd_write,    /* write */
  0,             /* seek */
  slcd_ioctl,    /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  slcd_poll,            /* poll */
#endif
};

/* LCD state data */

static struct sam_slcdstate_s g_slcdstate;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char segments[17] = {3, 4, 5, 6, 7, 8, 9, 10, 12, 24, 25, 26, 27, 28, 29, 30, 32};

static void lcd_clear(void)
{
	int i;
	for (i = 0; i < 6; i++) {
		putreg32(0, SAM_SLCDC_LMEMR(i));
		putreg32(0, SAM_SLCDC_MMEMR(i));
	}
}

static void segment_ctrl(int col, int row, int visible)
{
  uint32_t reg, regval;
  int segment = segments[row];

  //printf("segment_ctrl col:%i, row:%i, visible:%i\n", col, row, visible);

  if (segment < 32)
    reg = SAM_SLCDC_LMEMR(col);
  else {
    reg = SAM_SLCDC_MMEMR(col);
    segment -= 32;
  }

  regval = getreg32(reg);
  if (visible)
    regval |= 1 << segment;
  else
    regval &= ~(1 << segment);
  putreg32(regval, reg);
}

// A B C D  E F G H  I J K L  M N O P  Q
static const char* decode_segment(const char* start, const char* end)
{
  const char* iter = start;
  size_t len = end ? end - iter : strlen(iter);
  if (len < 4) return NULL;

  int row = iter[1];
  int col = iter[2];
  int on_off = iter[3];

  if (row < 'A' || row > 'Q' || col < '0' || col > '5' || (on_off != '+' && on_off != '-'))
    return NULL;

  row -= 'A';
  col -= '0';

  segment_ctrl(col, row, on_off == '+' ? 1 : 0);

  return start + 4;
}

static const char digits[] = {
0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71
};

static const char positions[] = {
 47,  79, 111, 143, 112,  48,  80,
 45,  77, 109, 141, 110,  46,  78,
 43,  75, 107, 139, 108,  44,  76,
 41,  73, 105, 137, 106,  42,  74,
 39,  71, 103, 135, 104,  40,  72,
 37,  69, 101, 133, 102,  38,  70,
 96,  64,  32,   0, 174, 172, 173,
 98,  66,  34,   2,  33,  97,  65,
100,  68,  36,   4,  35,  99,  67
};

static void digit_ctrl(int pos, int dig)
{
  const char* position;
  int digit;
  int i;
	//printf("digit_ctrl pos:%i, dig:%i\n", pos, dig);

  position = positions + 7 * pos;
  digit    = dig >= 0 ? digits[dig] : 0;

  for (i = 0; i < 7; i++, position++, digit >>= 1) {
    int p = *position;
    int col = p >> 5;
    int row = p & 0x1F;
    segment_ctrl(col, row, digit & 1);
  }
}

static const char* decode_digit(const char* start, const char* end)
{
  const char* iter = start;
  size_t len = end ? end - iter : strlen(iter);
	//printf("decode_digit %p %p %u\n", start, end, len);
  if (len < 3) return NULL;

  int pos = iter[1];
  int dig = iter[2];

  if (pos < '0' || pos > '9' || ((dig < '0' || dig > '9') && dig != '-'))
    return NULL;

  pos -= '0';
  dig = dig == '-' ? -1 : dig - '0';
  //printf("pos:%i, dig:%i\n", pos, dig);

  digit_ctrl(pos, dig);

  return start + 3;
}

static const char* decode_number(const char *start, const char* end)
{
  const char* iter = start;
  const char* digit = end - 1;
  size_t len = end ? end - iter : strlen(iter);
  int i, count, pos;
  if (len < 2) return NULL;
  len -= 2;

  int line = iter[1];

  if (line < '0' || line > '1')
    return 0;

  line -= '0';

  while((*digit < '0' || *digit > '9') && len > 0) {
    digit--;
    len--;
  }
  if (len > 6)
    return 0;
  else if (line == 1 && len > 3)
    return 0;

  count = line == 0 ? 6 : 3;
  pos = line == 0 ? 5 : 8;
  for (i = 0; i < count; i++) {
    if (len > 0) {
      char dig = *digit;
      if (dig < '0' || dig > '9')
        digit_ctrl(pos, -1);
      else
        digit_ctrl(pos, dig - '0');
      len--;
      digit--;
    }
    else {
      digit_ctrl(pos, -1);
    }
    pos--;
  }
  return end;
}

static const char* decode(const char* start, const char* end)
{
  //printf("decode start:%i\n", *start);
  switch (*start) {
  case 'S':
    return decode_segment(start, end);
  case 'D':
    return decode_digit(start, end);
  case 'N':
    return decode_number(start, end);
  case 'C':
    lcd_clear();
    return start + 1;
  case '\t':
  case ' ':
  case '\r':
  case '\n':
    return start + 1;
  default:
    return NULL;
  }
}

/****************************************************************************
 * Name: slcd_read
 ****************************************************************************/

static ssize_t slcd_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  return len;
}

/****************************************************************************
 * Name: slcd_write
 ****************************************************************************/

static ssize_t slcd_write(FAR struct file *filep,
                          FAR const char *buffer, size_t len)
{
  const char *start = buffer;
  const char *end = buffer + len;
  while (start != end) {
    start = decode(start, end);
    if (start == 0) return (ssize_t)len;
  }
  return (ssize_t)len;
}

/****************************************************************************
 * Name: slcd_ioctl
 ****************************************************************************/

static int slcd_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  return 0;
}

/****************************************************************************
 * Name: slcd_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int slcd_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  if (setup)
    {
      /* Data is always avaialble to be read / Data can always be written */

      fds->revents |= (fds->events & (POLLIN|POLLOUT));
      if (fds->revents != 0)
        {
          sem_post(fds->sem);
        }
    }

  return OK;
}
#endif

int sam_slcdc_initialize(void)
{
  uint32_t regval;
  int ret = OK;

  /* Only initialize the driver once. */

  if (!g_slcdstate.initialized)
    {
      lcdvdbg("Initializing\n");

      /* Enable clock for SLCDC */

      sam_slcdc_enableclk();

      regval = supc_get_slcd_power_mode();
      if (regval != SUPC_MR_LCDMODE_LCDOFF)
        supc_set_slcd_power_mode(SUPC_MR_LCDMODE_LCDOFF);
      supc_set_slcd_ldo_output(SUPC_MR_LCDVROUT_2p92V);
      supc_set_slcd_power_mode(SUPC_MR_LCDMODE_LCDON_INVR);

      /* Software reset the LCD controller */

      putreg32(SLCDC_CR_SWRST, SAM_SLCDC_CR);

      /* Configure LCD controller clock */

      putreg32(BOARD_FRR_PRESC | BOARD_FRR_CLOCKDIV, SAM_SLCDC_FRR);

      /* Setup the LCD controller configuration */

      regval = SLCDC_MR_COMSEL(BOARD_SLCD_NCOM) |
               SLCDC_MR_BUFFTIME_X64_SCLK |
               SLCDC_MR_BIAS_1_3;
      putreg32(regval, SAM_SLCDC_MR);

      putreg32(BOARD_SLCD_SEG_MASK0, SAM_SLCDC_SMR0);
      putreg32(BOARD_SLCD_SEG_MASK1, SAM_SLCDC_SMR1);

      putreg32(SLCDC_DR_DISPMODE_NORMAL, SAM_SLCDC_DR);

      /* Enable the display controller */

      putreg32(SLCDC_CR_LCDEN, SAM_SLCDC_CR);

      /* Register the LCD device driver */

      ret = register_driver("/dev/slcd", &g_slcdops, 0644, &g_slcdstate);
      g_slcdstate.initialized = true;
    }

  return ret;
}