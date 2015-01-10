/************************************************************************************
 * configs/omega/include/board.h
 ************************************************************************************/

#ifndef __CONFIGS_OMEGA_INCLUDE_BOARD_H
#define __CONFIGS_OMEGA_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  ifdef CONFIG_GPIO_IRQ
#    include <arch/irq.h>
#  endif
#  include <sys/types.h>
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* After power-on reset, the sam4cm device is running on a 4MHz internal RC.
 */

/* Main oscillator register settings */

#define BOARD_CKGR_MOR_MOSCXTST    (63 << PMC_CKGR_MOR_MOSCXTST_SHIFT) /* Start-up Time */

/* PLLA configuration:
 *
 * Source: 12MHz crystall at 12MHz
 * PLLdiv: 10
 * PLLmul: 1 (bypassed)
 * Fpll:   (12MHz * 10) / 1 = 120MHz
 */

#define BOARD_MAINOSC_FREQUENCY    (8192000)

#define BOARD_CKGR_PLLAR_MUL       ((250-1) << PMC_CKGR_PLLAR_MUL_SHIFT)
#define BOARD_CKGR_PLLAR_DIV       (1 << PMC_CKGR_PLLAR_DIV_SHIFT)
#define BOARD_CKGR_PLLAR_COUNT     (63 << PMC_CKGR_PLLAR_COUNT_SHIFT)

#define BOARD_CKGR_PLLBR_DIV       (PMC_CKGR_PLLBR_DIV(2))
#define BOARD_CKGR_PLLBR_MUL       ((25 - 1) << PMC_CKGR_PLLBR_MUL_SHIFT)
#define BOARD_CKGR_PLLBR_COUNT     (63 << PMC_CKGR_PLLBR_COUNT_SHIFT)
#define BOARD_CKGR_PLLBR_SRCB      (PMC_CKGR_PLLBR_SRCB_MAIN)
#define BOARD_PLLB_FREQUENCY       (25 * BOARD_MAINOSC_FREQUENCY / 2)

/* PMC master clock register settings */

#define BOARD_PMC_MCKR_CSS         PMC_MCKR_CSS_PLLB
#define BOARD_PMC_MCKR_PRES        PMC_MCKR_PRES_DIV1
#define BOARD_MCK_FREQUENCY        (BOARD_PLLB_FREQUENCY/1)
#define BOARD_CPU_FREQUENCY        (BOARD_PLLB_FREQUENCY/1)

/* USB UTMI PLL start-up time */

#define BOARD_CKGR_UCKR_UPLLCOUNT  (3 << PMC_CKGR_UCKR_UPLLCOUNT_SHIFT)

/* FLASH wait states:
 *
 * DC Characteristics
 *
 * Parameter              Min   Typ  Max
 * ---------------------- ----- ----- ----
 * Vddcore DC Supply Core 1.08V 1.2V 1.32V
 * Vvddio  DC Supply I/Os 1.62V 3.3V 3.6V
 *
 *                     Wait   Maximum
 * Vddcore   Vvddio   States Frequency (MHz)
 * ------- ---------- ------ ---------------
 * 1.08V   1.62-3.6V    0        16
 * "   "   "  "-"  "    1        33
 * "   "   "  "-"  "    2        50
 * "   "   "  "-"  "    3        67
 * "   "   "  "-"  "    4        84
 * "   "   "  "-"  "    5       100
 * 1.08V   2.7-3.6V     0        20
 * "   "   " "-"  "     1        40
 * "   "   " "-"  "     2        60
 * "   "   " "-"  "     3        80
 * "   "   " "-"  "     4       100
 * 1.2V    1.62-3.6V    0        17
 * "  "    " "-"  "     1        34
 * "  "    " "-"  "     2        52
 * "  "    " "-"  "     3        69
 * "  "    " "-"  "     4        87
 * "  "    " "-"  "     5       104
 * "  "    " "-"  "     6       121
 * 1.2V    2.7-3.6V     0        21
 * "  "    " "-"  "     1        42
 * "  "    " "-"  "     2        63
 * "  "    " "-"  "     3        84
 * "  "    " "-"  "     4       105
 * "  "    " "-"  "     5       123 << SELECTION
 */

#define BOARD_FWS                  5

/* Board specific pin configuration *************************************************/

#define GPIO_UART1_SWITCH    (GPIO_OUTPUT | GPIO_OUTPUT_SET    | GPIO_PORT_PIOC | GPIO_PIN6)
#define GPIO_AT25_NPCS1      (GPIO_OUTPUT | GPIO_OUTPUT_SET    | GPIO_PORT_PIOA | GPIO_PIN21)

#define GPIO_OMEGA_RF_NPCS2  (GPIO_OUTPUT | GPIO_OUTPUT_SET    | GPIO_PORT_PIOA | GPIO_PIN22)

#define GPIO_OMEGA_RF_IRQ    (GPIO_INPUT  | GPIO_INT_LOWLEVEL  | GPIO_PORT_PIOA | GPIO_PIN24 | GPIO_CFG_PULLUP)
#define GPIO_OMEGA_RF_RDY    (GPIO_INPUT  | GPIO_INT_LOWLEVEL  | GPIO_PORT_PIOB | GPIO_PIN13 | GPIO_CFG_PULLUP)
#define GPIO_OMEGA_RF_nRST   (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR  | GPIO_PORT_PIOA | GPIO_PIN25)

#define OMEGA_RF_IRQ         (SAM_IRQ_PA24)

/* Pin Multiplexing Disambiguation **************************************************/

/* LEDS ****************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

extern FAR struct spi_dev_s *g_spi;

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

void sam_reset(void);

void sam_boardinitialize(void);
#ifdef CONFIG_ARCH_LEDS
void board_led_initialize(void);
#endif
#ifdef CONFIG_SAM34_METROLOGY
int sam_metrology_initialize(void);
#endif
#ifdef CONFIG_SAM_KEXEC
int kexec(uint32_t address, void *args);
#endif
int sam_slcdc_initialize(void);

/************************************************************************************
 * Name: sam_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the omega board.
 *
 ************************************************************************************/
void sam_spiinitialize(void);

int sam_at25_initialize(void);
int sam_at24_initialize(void);

void flash_eraseblock(uint32_t page, size_t npages);
void flash_writeblock(const void* data, uint32_t page, size_t npages, int erase);
void flash_writepage(const void* data, size_t size, uint32_t page, int erase);

void loader_enable_switch(void);
void loader_disable_switch(void);

void gpbr_save(const void *data, size_t size, off_t offset);
void gpbr_load(void *data, size_t size, off_t offset);

#ifndef __ASSEMBLY__


#endif /* __ASSEMBLY__ */
#endif  /* __CONFIGS_COLIBRI_INCLUDE_BOARD_H */
