/************************************************************************************
 * configs/colibri/include/board.h
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
 ************************************************************************************/

#ifndef __CONFIGS_COLIBRI_INCLUDE_BOARD_H
#define __CONFIGS_COLIBRI_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

#define XTAL_FREQUENCY       12000000

/* Oscillator source is the main oscillator */

#define OSCSRC_FREQUENCY     XTAL_FREQUENCY
#define SYSCLK_FREQUENCY     120000000  /* 120MHz */

#if OSCSRC_FREQUENCY != 12000000
#  error "Unknown value of CONFIG_XTAL_FREQUENCY"
#endif

#if SYSCLK_FREQUENCY == 60000000
#  define PLLFREQ0  (SYSCON_PLLFREQ0_MINT_SET(40) | \
                     SYSCON_PLLFREQ0_MFRAC_SET(0) | \
                     SYSCON_PLLFREQ0_PLLPWR)
#  define PLLFREQ1  (SYSCON_PLLFREQ1_N_SET(0) | \
                     SYSCON_PLLFREQ1_Q_SET(0))
#  define RSCLKCFG  (SYSCON_RSCLKCFG_PSYSDIV_SET(7) | \
                     SYSCON_RSCLKCFG_USEPLL | \
                     SYSCON_RSCLKCFG_MEMTIMU | \
                     SYSCON_RSCLKCFG_PLLSRC_MOSC | SYSCON_RSCLKCFG_NEWFREQ)
#  define MEMTIME0  (SYSCON_MEMTIM0_FBCHT_2_0  | \
                     SYSCON_MEMTIM0_FWS_SET(3) | \
                     SYSCON_MEMTIM0_FBCE_FALL  | \
                     SYSCON_MEMTIM0_EBCHT_2_0  | \
                     SYSCON_MEMTIM0_EWS_SET(3) | \
                     SYSCON_MEMTIM0_EBCE_FALL)
#  define MEMTIME0_MASK (SYSCON_MEMTIM0_FBCHT_MASK | \
                         SYSCON_MEMTIM0_FWS_MASK | \
                         SYSCON_MEMTIM0_FBCE_MASK | \
                         SYSCON_MEMTIM0_EBCHT_MASK | \
                         SYSCON_MEMTIM0_EWS_MASK | \
                         SYSCON_MEMTIM0_EBCE_MASK)
#elif SYSCLK_FREQUENCY == 120000000
#  define PLLFREQ0  (SYSCON_PLLFREQ0_MINT_SET(40) | \
                     SYSCON_PLLFREQ0_MFRAC_SET(0) | \
                     SYSCON_PLLFREQ0_PLLPWR)
#  define PLLFREQ1  (SYSCON_PLLFREQ1_N_SET(0) | \
                     SYSCON_PLLFREQ1_Q_SET(0))
#  define RSCLKCFG  (SYSCON_RSCLKCFG_PSYSDIV_SET(3) | \
                     SYSCON_RSCLKCFG_USEPLL | \
                     SYSCON_RSCLKCFG_MEMTIMU | \
                     SYSCON_RSCLKCFG_PLLSRC_MOSC | SYSCON_RSCLKCFG_NEWFREQ)
#  define MEMTIME0  (SYSCON_MEMTIM0_FBCHT_3_5  | \
                     SYSCON_MEMTIM0_FWS_SET(6) | \
                     SYSCON_MEMTIM0_FBCE_FALL  | \
                     SYSCON_MEMTIM0_EBCHT_3_5  | \
                     SYSCON_MEMTIM0_EWS_SET(3) | \
                     SYSCON_MEMTIM0_EBCE_FALL)
#  define MEMTIME0_MASK (SYSCON_MEMTIM0_FBCHT_MASK | \
                         SYSCON_MEMTIM0_FWS_MASK | \
                         SYSCON_MEMTIM0_FBCE_MASK | \
                         SYSCON_MEMTIM0_EBCHT_MASK | \
                         SYSCON_MEMTIM0_EWS_MASK | \
                         SYSCON_MEMTIM0_EBCE_MASK)
#else
#  error "Unknown value of SYSCLK_FREQUENCY"
#endif

/* Board specific pin configuration *************************************************/

#define GPIO_LED_CPU (GPIO_FUNC_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTE | 0)

#define GPIO_SSI1_CLK    (GPIO_FUNC_PFIO      | GPIO_PORTB | GPIO_ALT_15 | 5)       /* PA2: SSI1 clock (SSI1Clk) */
#define GPIO_SSI1_TX     (GPIO_FUNC_PFOUTPUT  | GPIO_PORTE | GPIO_ALT_15 | 4)       /* PA4: SSI1 transmit (SSI1Tx) */
#define GPIO_SSI1_RX     (GPIO_FUNC_PFINPUT   | GPIO_PORTE | GPIO_ALT_15 | 5)       /* PA5: SSI1 receive (SSI1Rx) */

#define GPIO_SSI_CS_SF   (GPIO_FUNC_OUTPUT    | GPIO_PORTF | 3 | GPIO_VALUE_ONE)    /* PA7: SSI0 Serial Flash chip select */
#define GPIO_SSI_CS_EE   (GPIO_FUNC_OUTPUT    | GPIO_PORTF | 2 | GPIO_VALUE_ONE)    /* PA6: SSI0 EEPROM chip select */
#define GPIO_SSI_CS_ETH  (GPIO_FUNC_OUTPUT    | GPIO_PORTB | 4 | GPIO_VALUE_ONE)    /* Use SSI1 Fss signal instead of CS */

#define GPIO_ETH_INTRN   (GPIO_FUNC_INTERRUPT | GPIO_PORTF | 1 | GPIO_INT_LOWLEVEL)/* PG5: ETH chip interrupt */

/* Pin Multiplexing Disambiguation **************************************************/

#define GPIO_UART1_CTS    GPIO_UART1_CTS_1
#define GPIO_UART1_RTS    GPIO_UART1_RTS_1
#define GPIO_UART1_RX     GPIO_UART1_RX_1
#define GPIO_UART1_TX     GPIO_UART1_TX_1

/* LEDS ****************************************************************************/

#define LED_IDLE 0

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Name: lm_boardinitialize
 *
 * Description:
 *   All Stellaris architectures must provide the following entry point.  This entry
 *   point is called early in the intitialization -- after all memory has been
 *   configured and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void lm_boardinitialize(void);

#  define TTYS2_DEV           g_uart2port /* UART2 is ttyS2 */
#  define UART2_ASSIGNED      1

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIGS_COLIBRI_INCLUDE_BOARD_H */
