/************************************************************************************
 * configs/omega/src/metrology.c
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
#include "chip/sam_pmc.h"
#include "chip/sam_rstc.h"
#include "chip/sam4cm_ipc.h"
#include "sam_periphclks.h"
#include "sam_gpio.h"

#include "arch/board/metrology.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

#define IPC_INIT_IRQ          (0x1u << 20)
#define IPC_STATUS_IRQ        (0x1U << 16)
#define IPC_HALF_CYCLE_IRQ    (0x1U << 5)
#define IPC_FULL_CYCLE_IRQ    (0x1U << 4)
#define IPC_INTEGRATION_IRQ   (0x1U << 0)

//====================SHARED MEMORY SETTINGS=========================
#define mem_reg_in (0x20100000)
#define mem_reg_out (mem_reg_in + 55*4)//DSP_CONTROL_SIZE*4)
#define mem_acc_out (mem_reg_out + 35*4)//DSP_ST_SIZE*4)
#define mem_har_out (mem_acc_out + 55*8)//DSP_ACC_SIZE*8)

#define DSP_READINGS_BUFFER_SIZE 4

#define enable_ipc_irq()  up_enable_irq(SAM_IRQ_IPC0)
#define disable_ipc_irq() up_disable_irq(SAM_IRQ_IPC0)

#define GPIO_ATSENSE_CLOCK (GPIO_PERIPHA | GPIO_CFG_DEFAULT | GPIO_PORT_PIOA | GPIO_PIN29)

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

static int ipc0_interrupt(int irq, void *context);

static int metrology_reset(void);
static int metrology_set_configuration(unsigned long arg);
static int metrology_apply_configuration(void);
static int metrology_set_acc(unsigned long arg);
static int metrology_get_acc(unsigned long arg);
static int metrology_shutdown(void);

/* Character driver methods */
static ssize_t metrology_read(FAR struct file *, FAR char *, size_t);
static int metrology_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int metrology_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the driver state structure (there is no retained state information) */

static const struct file_operations g_metrologyops =
{
  0,                  /* open */
  0,                  /* close */
  metrology_read,     /* read */
  0,                  /* write */
  0,                  /* seek */
  metrology_ioctl,    /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  metrology_poll,     /* poll */
#endif
};

static DSP_CTRL_TYPE    configuration;
static ACC_Tx_TYPE      acc;
static DSP_READING_TYPE readings[DSP_READINGS_BUFFER_SIZE];
static unsigned first_reading, last_reading, readings_count, readings_overflow;
#ifndef CONFIG_DISABLE_POLL
FAR struct pollfd *poll_wait;
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static void reset_core1(void)
{
#if !defined(CONFIG_SAM34_UART1)
  putreg32(RSTC_CPMR_CPKEY, SAM_RSTC_CPMR); // Reset Core1 CPU and peripheral
#else
  putreg32(RSTC_CPMR_CPEREN | RSTC_CPMR_CPKEY, SAM_RSTC_CPMR); // Reset Core1 CPU; leave peripheral running
#endif
}

static void enable_core1(void)
{
#if !defined(CONFIG_SAM34_UART1)
  uint32_t regval;

  putreg32(PMC_CPBMCK | PMC_CPCK | PMC_CPKEY, SAM_PMC_SCER);

  regval = getreg32(SAM_PMC_MCKR);
  regval &= ~PMC_MCKR_CPPRES_MASK;
  regval |= PMC_MCKR_CPPRES(1) | PMC_MCKR_CCPSS_PLLB;
  putreg32(regval, SAM_PMC_MCKR);

  // Release Core1 peripheral reset
  regval = getreg32(SAM_RSTC_CPMR);
  regval |= RSTC_CPMR_CPEREN | RSTC_CPMR_CPKEY;
  putreg32(regval, SAM_RSTC_CPMR);
#endif

  sam_sram_enableclk();
}

static void disable_core1(void)
{
  sam_sram_disableclk();
#if !defined(CONFIG_SAM34_UART1)
  putreg32(PMC_CPBMCK | PMC_CPCK | PMC_CPKEY, SAM_PMC_SCDR);
#else
  putreg32(PMC_CPCK | PMC_CPKEY, SAM_PMC_SCDR);
#endif
}

static void load_core1_firmware(void)
{
  extern uint32_t __smetrology;
  extern uint32_t __emetrology;

  // Copy metrology firmware to SRAM1 (Core 1) @ 0x20080000
  memcpy((void*)0x20080000, &__smetrology, (char*)&__emetrology - (char*)&__smetrology);
  //assert(memcmp((void*)0x20080000, &__smetrology, (char*)&__emetrology - (char*)&__smetrology) == 0);
}

static void start_core1(void)
{
  // Releset Core1 CPU reset
  uint32_t regval = getreg32(SAM_RSTC_CPMR);
  regval |= RSTC_CPMR_CPROCEN | RSTC_CPMR_CPKEY;
  putreg32(regval, SAM_RSTC_CPMR);
}

static void enable_atsense_clock(void)
{
  sam_pmc_enableclk();
  sam_pioa_enableclk();
  sam_configgpio(GPIO_ATSENSE_CLOCK);

  // set PCK1 = PLLA clock / 2
  putreg32(PMC_PCK_CSS_PLLA | PMC_PCK_PRES_DIV2, SAM_PMC_PCK1);
  // enable PCK1
  putreg32(PMC_PCK1, SAM_PMC_SCER);
}

static void disable_atsense_clock(void)
{
  putreg32(PMC_PCK1, SAM_PMC_SCDR);
}

static int init_metrology_ipc(void)
{
  int res;

  //Enable IPC0, IPC1 clock
  sam_ipc0_enableclk();
  sam_ipc1_enableclk();

  res = irq_attach(SAM_IRQ_IPC0, ipc0_interrupt);
  if (res) return res;

  enable_ipc_irq();

  // Enable Metrology IPC interrupts
  putreg32(/*IPC_INIT_IRQ | IPC_STATUS_IRQ | IPC_HALF_CYCLE_IRQ | IPC_FULL_CYCLE_IRQ |*/ IPC_INTEGRATION_IRQ, SAM_IPC0_IECR);

  return 0;
}

static void read_dsp_data(DSP_READING_TYPE *reading)
{
  memcpy(&reading->STATUS, (void*)mem_reg_out, sizeof(reading->STATUS));
	memcpy(&reading->ACC, (void*)mem_acc_out, sizeof(reading->ACC));
}

static void Set_DSP_CTRL_ST_CTRL( DSP_CTRL_ST_CTRL_TYPE iSStatus )
{
    //VMetrology.DSP_CTRL.STATE_CTRL.BIT.ST_CTRL = iSStatus;
    ( ( DSP_CTRL_TYPE * )( mem_reg_in ) )->STATE_CTRL.BIT.ST_CTRL = iSStatus;
}

static void init_dsp(void)
{
  int i;

  //----1---set DSP_CTRL structure---except ST_CTRL--------
  Set_DSP_CTRL_ST_CTRL( IsReset );

  //---2---wait for DSP Reset------------------------------
  for( i=0; i<200; i++ ) {
    if( ( ( DSP_ST_TYPE * )( mem_reg_out ) )->STATUS.BIT.ST == DSP_ST_Reset ) {
      break;
    }
    usleep( 1000 );
  }

  //----3---set Metrolgoy Configure structure--------------
  for ( i = 1; i < ( DSP_CONTROL_SIZE ); i++ ) {
    *( ( uint32_t * )( mem_reg_in ) + i ) = *( ( uint32_t * )( &configuration ) + i );
  }

  //----4---set DSP_CTRL_ST_CTRL =Init---------------------
  Set_DSP_CTRL_ST_CTRL( IsInit );

  //---5---wait for DSP Ready------------------------------
  for( i=0;i<200;i++ ) {
    if( ( ( DSP_ST_TYPE * )( mem_reg_out ) )->STATUS.BIT.ST == DSP_ST_Ready ) {
      break;
    }
     usleep( 1000 );
  }

  //----6---set DSP_CTRL_ST_CTRL =Run----------------------
  Set_DSP_CTRL_ST_CTRL( IsRun );

  //---7---wait for DSP Running----------------------------
  for( i=0;i<200;i++ ) {
    if( ( ( DSP_ST_TYPE * )( mem_reg_out ) )->STATUS.BIT.ST == DSP_ST_DSP_Running )
      break;
    usleep( 1000 );
  }

  ( ( DSP_ACC_TYPE * )( mem_acc_out ) )->ACC_T0 = acc.ACC_T0;
  ( ( DSP_ACC_TYPE * )( mem_acc_out ) )->ACC_T1 = acc.ACC_T1;
  ( ( DSP_ACC_TYPE * )( mem_acc_out ) )->ACC_T2 = acc.ACC_T2;
}

static int metrology_reset(void)
{
  reset_core1();
  enable_core1();
  load_core1_firmware();
  return OK;
}

static int metrology_set_configuration(unsigned long arg)
{
  memcpy(&configuration, (void*)arg, sizeof(configuration));
  return OK;
}

static int metrology_apply_configuration(void)
{
  int i;
  for ( i = 1; i < ( DSP_CONTROL_SIZE ); i++ ) {
    *( ( uint32_t * )( mem_reg_in ) + i ) = *( ( uint32_t * )( &configuration ) + i );
  }
  return OK;
}

static int metrology_run(void)
{
  enable_atsense_clock();
  start_core1();
  init_dsp();
  return OK;
}

static int metrology_set_acc(unsigned long arg)
{
  disable_ipc_irq();
  memcpy(&acc, (void*)arg, sizeof(acc));
  enable_ipc_irq();
  return OK;
}

static int metrology_get_acc(unsigned long arg)
{
  disable_ipc_irq();
  memcpy((void*)arg, &acc, sizeof(acc));
  enable_ipc_irq();
  return OK;
}

static int metrology_shutdown()
{
  reset_core1();
  disable_atsense_clock();
  disable_core1();
  return OK;
}

static int ipc0_interrupt(int irq, void *context)
{
  uint32_t ints = getreg32(SAM_IPC0_IPR);
  // Ack interrupts
  putreg32(ints, SAM_IPC0_ICCR);

  if (ints & IPC_INTEGRATION_IRQ) {
    if (readings_count < DSP_READINGS_BUFFER_SIZE) {
      read_dsp_data(readings + last_reading);
      readings_count++;
      last_reading = (last_reading + 1) & (DSP_READINGS_BUFFER_SIZE - 1);
    }
    else
      readings_overflow = 1;

    acc.ACC_T0 = ( ( DSP_ACC_TYPE * )( mem_acc_out ) )->ACC_T0;
    acc.ACC_T1 = ( ( DSP_ACC_TYPE * )( mem_acc_out ) )->ACC_T1;
    acc.ACC_T2 = ( ( DSP_ACC_TYPE * )( mem_acc_out ) )->ACC_T2;

#ifndef CONFIG_DISABLE_POLL
    if (poll_wait) {
      poll_wait->revents = POLLIN;
      sem_post(poll_wait->sem);
      poll_wait = 0;
    }
#endif
  }

  return OK;
}

static ssize_t metrology_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  if ((filep->f_oflags & O_NONBLOCK) == 0)
    return -EINVAL;

  if (len < sizeof(DSP_READING_TYPE))
    return -EINVAL;

  disable_ipc_irq();
  if (readings_count == 0) {
    enable_ipc_irq();
    return -EAGAIN;
  }

  memcpy(buffer, readings + first_reading, sizeof(DSP_READING_TYPE));
  first_reading = (first_reading + 1) & (DSP_READINGS_BUFFER_SIZE - 1);
  readings_count--;

  enable_ipc_irq();
  return sizeof(DSP_READING_TYPE);
}

static int metrology_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  switch(cmd) {
	case METROLOGY_IOCTL_RESET:
		return metrology_reset();
	case METROLOGY_IOCTL_SETCFG:
		return metrology_set_configuration(arg);
	case METROLOGY_IOCTL_APPLYCFG:
		return metrology_apply_configuration();
	case METROLOGY_IOCTL_GETACC:
		return metrology_get_acc(arg);
	case METROLOGY_IOCTL_SETACC:
		return metrology_set_acc(arg);
	case METROLOGY_IOCTL_RUN:
		return metrology_run();
	case METROLOGY_IOCTL_SHUTDOWN:
		return metrology_shutdown();
	default:
		return -EINVAL;
	}
}

#ifndef CONFIG_DISABLE_POLL
static int metrology_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup)
{
  if (setup) {
    if (fds->events != POLLIN)
      return -EINVAL;

    disable_ipc_irq();
    if (poll_wait) {
      enable_ipc_irq();
      return -EBUSY;
    }
    poll_wait = fds;
    enable_ipc_irq();
  }
  else
  {
    disable_ipc_irq();
    poll_wait = 0;
    enable_ipc_irq();
  }

  return OK;
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

int sam_metrology_initialize(void)
{
  int ret;
  reset_core1();
  ret = init_metrology_ipc();
  if (ret) return ret;
  return register_driver("/dev/metrology", &g_metrologyops, 0444, 0);;
}
