/************************************************************************************
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>

#include <nuttx/net/netdev.h>

#include <arch/chip/gpio.h>

#include <nuttx/wireless/omega_rf.h>

#include "spi-uip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define RF_WDDELAY   (1*CLK_TCK)
#define RF_POLLHSEC  (1*2)

#define TX_QUEUE_THRESHOLD 6

/* TX timeout = 10 sec */

#define RF_TXTIMEOUT (10*CLK_TCK)

#define UIP_IP_BUF    ((struct uip_ip_hdr *)&dev->d_buf[UIP_LLH_LEN])
#define UIP_LL_BUF    ((struct omega_rf_hdr *)&dev->d_buf[0])

/* Configuration ********************************************************************/

#ifndef CONFIG_OMEGA_RF_SPIMODE
#  define CONFIG_OMEGA_RF_SPIMODE SPIDEV_MODE0
#endif

#ifndef CONFIG_OMEGA_RF_SPIFREQUENCY
#  define CONFIG_OMEGA_RF_SPIFREQUENCY 1000000
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/

struct rf_dev_s
{
	const FAR omega_rf_cfg *cfg;
	FAR struct spi_dev_s   *spi;           /* Saved SPI interface instance */
	struct uip_driver_s     dev;           /* Interface understood by uIP */
	int ifup;

	WDOG_ID                 txpoll;        /* TX poll timer */
	WDOG_ID                 txtimeout;     /* TX timeout timer */

	struct work_s           pollwork;      /* Poll timeout work queue support */
	struct work_s           irqwork;       /* IRQ work queue support */
	struct work_s           rstwork;        /* Tx timeout work queue support */
	
	struct spi_status_t     status;
	
	int                     ready_phase;
};

static void rf_polltimer(int argc, uint32_t arg, ...);
static int rf_uiptxpoll(struct uip_driver_s *dev);
static int rf_receive(FAR struct rf_dev_s *priv);
static int rf_transmit(FAR struct rf_dev_s *priv);
static void rf_sched_reset(FAR struct rf_dev_s *priv);
static void rf_out(struct uip_driver_s *dev);

static FAR struct rf_dev_s *g_priv;

/************************************************************************************
 * Name: rf_is_ready
 ************************************************************************************/

static int rf_is_ready(FAR struct rf_dev_s *priv)
{
	return GPIO_READ(priv->cfg->rdy_gpio) == priv->ready_phase;
}

static int rf_can_transimt(FAR struct rf_dev_s *priv)
{
	return rf_is_ready(priv) && priv->status.tx_queue_freespace >= TX_QUEUE_THRESHOLD;
}

/************************************************************************************
 * Name: rf_wait_for_ready
 ************************************************************************************/

static int rf_wait_for_ready(FAR struct rf_dev_s *priv, int timeout)
{
	while (!rf_is_ready(priv)) {
		/*if (timeout > 0) {
			usleep(1000);
			timeout--;
		}
		else if (timeout == 0) {
			return 0;
		}*/
	}

	return 1;
}

static void rf_toggle_ready_phase(FAR struct rf_dev_s *priv)
{
	if (priv->ready_phase == 0)
		priv->ready_phase = 1;
	else
		priv->ready_phase = 0;
}

static int rf_ack_irq(FAR struct rf_dev_s *priv)
{
	struct spi_command_t cmd;
	struct spi_status_t status;

	cmd.magic = SPI_CMD_MAGIC;
	cmd.cmd = SPI_CMD_IRQACK;

	SPI_SELECT(priv->spi, priv->cfg->spidev, true);
	SPI_EXCHANGE(priv->spi, &cmd, &status, sizeof(cmd));
	SPI_SELECT(priv->spi, priv->cfg->spidev, false);
	rf_toggle_ready_phase(priv);

	nllvdbg("status: tx_queue_freespace:%u rx_packet_size:%u\n",
	        priv->status.tx_queue_freespace, priv->status.rx_packet_size);
	
	if (status.magic != SPI_STATUS_MAGIC) {
		rf_sched_reset(priv);
		return -1;
	}

	priv->status = status;

	return priv->status.pending_irqs;
}

/************************************************************************************
 * Name: rf_lock
 ************************************************************************************/

static void rf_lock(FAR struct rf_dev_s *priv)
{
  /* On SPI busses where there are multiple devices, it will be necessary to
   * lock SPI to have exclusive access to the busses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusiv access to
   * the SPI buss.  We will retain that exclusive access until the bus is unlocked.
   */

  (void)SPI_LOCK(priv->spi, true);

  /* After locking the SPI bus, the we also need call the setfrequency, setbits, and
   * setmode methods to make sure that the SPI is properly configured for the device.
   * If the SPI buss is being shared, then it may have been left in an incompatible
   * state.
   */

  SPI_SETMODE(priv->spi, CONFIG_OMEGA_RF_SPIMODE);
  SPI_SETBITS(priv->spi, 8);
  (void)SPI_SETFREQUENCY(priv->spi, 10000);
}

/************************************************************************************
 * Name: rf_unlock
 ************************************************************************************/

static inline void rf_unlock(FAR struct rf_dev_s *priv)
{
  (void)SPI_LOCK(priv->spi, false);
}

/************************************************************************************
 * Name: rf_set_hw_addr
 ************************************************************************************/

static void rf_set_hw_addr(FAR struct rf_dev_s *priv, uint8_t *addr)
{
	struct spi_command_t cmd;
	struct spi_status_t status;

	cmd.magic = SPI_CMD_MAGIC;
	cmd.cmd = SPI_CMD_SETHWADDR; // set hw address

	cmd.setaddr.addr[0] = 0x02;
	cmd.setaddr.addr[1] = 0x00;
	memcpy(&cmd.setaddr.addr[2], addr, 6);

	/* Get exclusive access to the SPI bus. */

	rf_lock(priv);

	SPI_SELECT(priv->spi, priv->cfg->spidev, true);
	SPI_EXCHANGE(priv->spi, &cmd, &status, sizeof(cmd));
	SPI_SELECT(priv->spi, priv->cfg->spidev, false);
	rf_toggle_ready_phase(priv);

	if (status.magic != SPI_STATUS_MAGIC)
		rf_sched_reset(priv);
	else
		priv->status = status;

	/* Release lock on the SPI bus */

	rf_unlock(priv);
}

/************************************************************************************
 * Name: rf_ifup
 ************************************************************************************/

static int rf_ifup(struct uip_driver_s *dev)
{
	nllvdbg("up\n");

	FAR struct rf_dev_s *priv = (FAR struct rf_dev_s *)dev->d_private;
	if (priv->ifup)
		return OK;

	priv->ready_phase = 0;
	GPIO_WRITE(priv->cfg->nrst_gpio, 1);
	if (!rf_wait_for_ready(priv, 2000)) {
		GPIO_WRITE(priv->cfg->nrst_gpio, 0);
		priv->ready_phase = 0;
		return -1;
	}

	rf_set_hw_addr(priv, dev->d_mac.ether_addr_octet);

	(void)wd_start(priv->txpoll, RF_WDDELAY, rf_polltimer, 1, (uint32_t)priv);
	up_enable_irq(priv->cfg->rx_irq);

	priv->ifup = 1;

	return OK;
}

/************************************************************************************
 * Name: rf_ifdown
 ************************************************************************************/

static int rf_ifdown(struct uip_driver_s *dev)
{
	nllvdbg("down\n");

	FAR struct rf_dev_s *priv = (FAR struct rf_dev_s *)dev->d_private;
	if (!priv->ifup)
		return 0;

	GPIO_WRITE(priv->cfg->nrst_gpio, 0);

	up_disable_irq(priv->cfg->rx_irq);

	wd_cancel(priv->txpoll);
	wd_cancel(priv->txtimeout);

	priv->ifup = 0;
	priv->ready_phase = 0;

	return 0;
}

/****************************************************************************
 * Function: enc_rxdispatch
 *
 * Description:
 *   Give the newly received packet to uIP.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Interrupts are enabled but the caller holds the uIP lock.
 *
 ****************************************************************************/

static void rf_rxdispatch(FAR struct rf_dev_s *priv)
{
	uip_input(&priv->dev);

	/* If the above function invocation resulted in data that should be
	 * sent out on the network, the field  d_len will set to a value > 0.
	 */

	if (priv->dev.d_len > 0) {
		rf_out(&priv->dev);
		rf_transmit(priv);
	}
}

/****************************************************************************
 * Function: rf_pollworker
 *
 * Description:
 *   Periodic timer handler continuation.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void rf_pollworker(FAR void *arg)
{
	FAR struct rf_dev_s *priv = (FAR struct rf_dev_s *)arg;
	uip_lock_t lock;

	DEBUGASSERT(priv);

	/* Get exclusive access to both uIP and the SPI bus. */

	lock = uip_lock();
	rf_lock(priv);

	if (rf_can_transimt(priv)) {
		(void)uip_timer(&priv->dev, rf_uiptxpoll, RF_POLLHSEC);
	}

	/* Release lock on the SPI bus and uIP */

	rf_unlock(priv);
	uip_unlock(lock);

	/* Setup the watchdog poll timer again */

	(void)wd_start(priv->txpoll, RF_WDDELAY, rf_polltimer, 1, arg);
}

static void rf_tx_done(FAR struct rf_dev_s *priv)
{
	/* If no further xmits are pending, then cancel the TX timeout */

	wd_cancel(priv->txtimeout);

	/* Then poll uIP for new XMIT data */

	if (rf_can_transimt(priv)) {
		(void)uip_poll(&priv->dev, rf_uiptxpoll);
	}
}

static void rf_rx_done(FAR struct rf_dev_s *priv)
{
	if (rf_receive(priv) == OK) {
		rf_rxdispatch(priv);
	}
}

/****************************************************************************
 * Function: rf_irqworker
 *
 * Description:
 *   Perform interrupt handling logic outside of the interrupt handler (on
 *   the work queue thread).
 *
 * Parameters:
 *   arg     - The reference to the driver structure (case to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void rf_irqworker(FAR void *arg)
{
	FAR struct rf_dev_s *priv = (FAR struct rf_dev_s *)arg;
	uip_lock_t lock;
	uint8_t pending_irqs;

	DEBUGASSERT(priv);

	/* Get exclusive access to both uIP and the SPI bus. */

	lock = uip_lock();
	rf_lock(priv);

	pending_irqs = rf_ack_irq(priv);
	
	if (pending_irqs & SPI_IRQ_RX) {
		rf_rx_done(priv);
	}
	
	if (pending_irqs & SPI_IRQ_TX) {
		rf_tx_done(priv);
	}

	/* Release lock on the SPI bus and uIP */

	rf_unlock(priv);
	uip_unlock(lock);

	usleep(100);
	up_enable_irq(priv->cfg->rx_irq);
}

/****************************************************************************
 * Function: rf_rstworker
 *
 * Description:
 *   Our TX watchdog timed out.  This is the worker thread continuation of
 *   the watchdog timer interrupt.  Reset the hardware and start again.
 *
 * Parameters:
 *   arg     - The reference to the driver structure (case to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void rf_rstworker(FAR void *arg)
{
  FAR struct rf_dev_s *priv = (FAR struct rf_dev_s *)arg;
  uip_lock_t lock;
  int ret;

  nlldbg("reset\n");
  DEBUGASSERT(priv);

  /* Get exclusive access to uIP */

  lock = uip_lock();

  /* Then reset the hardware: Take the interface down, then bring it
   * back up
   */

  ret = rf_ifdown(&priv->dev);
  DEBUGASSERT(ret == OK);
  ret = rf_ifup(&priv->dev);
  DEBUGASSERT(ret == OK);

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&priv->dev, rf_uiptxpoll);

  /* Release lock on uIP */

  uip_unlock(lock);
}

static void rf_sched_reset(FAR struct rf_dev_s *priv)
{
	int ret;
	if (priv && work_available(&priv->rstwork)) {
		ret = work_queue(HPWORK, &priv->rstwork, rf_rstworker, (FAR void *)priv, 0);
		DEBUGASSERT(ret == OK);
	}
}

/****************************************************************************
 * Function: rf_txtimeout
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Perform work on the worker thread.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void rf_txtimeout(int argc, uint32_t arg, ...)
{
  FAR struct rf_dev_s *priv = (FAR struct rf_dev_s *)arg;

  /* In complex environments, we cannot do SPI transfers from the timout
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  DEBUGASSERT(priv);
  rf_sched_reset(priv);
}

/****************************************************************************
 * Function: rf_polltimer
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void rf_polltimer(int argc, uint32_t arg, ...)
{
  FAR struct rf_dev_s *priv = (FAR struct rf_dev_s *)arg;
  int ret;

  /* In complex environments, we cannot do SPI transfers from the timout
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  DEBUGASSERT(priv);

  if (work_available(&priv->pollwork))
  {
	/* Notice that poll watchdog is not active so further poll timeouts can
	* occur until we restart the poll timeout watchdog.
	*/

	ret = work_queue(HPWORK, &priv->pollwork, rf_pollworker, (FAR void *)priv, 0);
	DEBUGASSERT(ret == OK);
  }
}

static void rf_out(struct uip_driver_s *dev)
{
	uip_ipaddr_t destipaddr;
	uip_ipaddr_t ipaddr;

	/* Check if the destination address is on the local network. */

	uip_ipaddr_copy(destipaddr, UIP_IP_BUF->destipaddr);
	if (!uip_ipaddr_maskcmp(destipaddr, dev->d_ipaddr, dev->d_netmask))
	{
		/* Destination address is not on the local network */

#ifdef CONFIG_NET_ROUTE
		/* We have a routing table.. find the correct router to use in
		 * this case (or, as a fall-back, use the device's default router
		 * address).  We will use the router IP address instead of the
		 * destination address when determining the MAC address.
		 */

		netdev_router(dev, destipaddr, &ipaddr);
#else
	/* Use the device's default router IP address instead of the
	* destination address when determining the MAC address.
	*/

		uip_ipaddr_copy(ipaddr, dev->d_draddr);
#endif
	}
	else
	{
		/* Else, we use the destination IP address. */

		uip_ipaddr_copy(ipaddr, destipaddr);
	}

	UIP_LL_BUF->destaddr[0] = 0x02;
	UIP_LL_BUF->destaddr[1] = 0x00;
	memcpy(&UIP_LL_BUF->destaddr[2], &((uint8_t*)ipaddr)[10], 6);
}

/****************************************************************************
 * Function: rf_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from:
 *
 *   -  pkif interrupt when an application responds to the receipt of data
 *      by trying to send something, or
 *   -  From watchdog based polling.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int rf_transmit(FAR struct rf_dev_s *priv)
{
	struct spi_command_t cmd;
	struct spi_status_t status;
	struct uip_driver_s *dev = &priv->dev;

	cmd.magic = SPI_CMD_MAGIC;
	cmd.cmd = SPI_CMD_WRITE;

	cmd.write.size = dev->d_len + UIP_LLH_LEN; // packet length

	//rf_unlock(priv);
	if (!rf_wait_for_ready(priv, 1000)) {
		rf_sched_reset(priv);
		return -1;
	}
	//rf_lock(priv);

	SPI_SELECT(priv->spi, priv->cfg->spidev, true);
	SPI_EXCHANGE(priv->spi, &cmd, &status, sizeof(cmd));
	SPI_SELECT(priv->spi, priv->cfg->spidev, false);
	rf_toggle_ready_phase(priv);

	if (status.magic != SPI_STATUS_MAGIC) {
		rf_sched_reset(priv);
		return -1;
	}

	//rf_unlock(priv);
	if (!rf_wait_for_ready(priv, 100)) {
		rf_sched_reset(priv);
		return -1;
	}
	//rf_lock(priv);
	
	SPI_SELECT(priv->spi, priv->cfg->spidev, true);
	SPI_SNDBLOCK(priv->spi, dev->d_buf, dev->d_len + UIP_LLH_LEN);
	SPI_SELECT(priv->spi, priv->cfg->spidev, false);
	rf_toggle_ready_phase(priv);

	(void)wd_start(priv->txtimeout, RF_TXTIMEOUT, rf_txtimeout, 1, (uint32_t)priv);

	return OK;
}

/****************************************************************************
 * Function: rf_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from:
 *
 *   -  pkif interrupt when an application responds to the receipt of data
 *      by trying to send something, or
 *   -  From watchdog based polling.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int rf_receive(FAR struct rf_dev_s *priv)
{
	struct spi_command_t cmd;
	struct spi_status_t status;
	struct uip_driver_s *dev = &priv->dev;
	uint16_t pkt_len = priv->status.rx_packet_size;

	nllvdbg("receive len:%u\n", pkt_len);

	cmd.magic = SPI_CMD_MAGIC;
	cmd.cmd = SPI_CMD_READ;

	cmd.read.size = pkt_len; // packet length

	rf_unlock(priv);
	if (!rf_wait_for_ready(priv, 100)) {
		rf_sched_reset(priv);
		return -1;
	}
	rf_lock(priv);

	SPI_SELECT(priv->spi, priv->cfg->spidev, true);
	SPI_EXCHANGE(priv->spi, &cmd, &status, sizeof(cmd));
	SPI_SELECT(priv->spi, priv->cfg->spidev, false);
	rf_toggle_ready_phase(priv);
	
	if (status.magic != SPI_STATUS_MAGIC) {
		rf_sched_reset(priv);
		return -1;
	}

	rf_unlock(priv);
	if (!rf_wait_for_ready(priv, 100)) {
		rf_sched_reset(priv);
		return -1;
	}
	rf_lock(priv);
	
	SPI_SELECT(priv->spi, priv->cfg->spidev, true);
	SPI_RECVBLOCK(priv->spi, dev->d_buf, pkt_len);
	SPI_SELECT(priv->spi, priv->cfg->spidev, false);
	rf_toggle_ready_phase(priv);

	dev->d_len = pkt_len;

	return OK;
}

/****************************************************************************
 * Function: rf_uiptxpoll
 *
 * Description:
 *   The transmitter is available, check if uIP has any outgoing packets ready
 *   to send.  This is a callback from uip_poll().  uip_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   Interrupts are enabled but the caller holds the uIP lock.
 *
 ****************************************************************************/

static int rf_uiptxpoll(struct uip_driver_s *dev)
{
	FAR struct rf_dev_s *priv = (FAR struct rf_dev_s *)dev->d_private;
	uint16_t len = priv->dev.d_len;

	/* If the polling resulted in data that should be sent out on the network,
	 * the field d_len is set to a value > 0.
	 */

	nllvdbg("Poll result: d_len=%d\n", len);
	if (len > 0) {
		uint8_t packets_expected;
		if (len <= (61 - 23))
			packets_expected = 1;
		else
			packets_expected = 1 + ((len - 38) + 29) / 30;

		rf_out(dev);
		rf_transmit(priv);

		if (priv->status.tx_queue_freespace >= packets_expected)
			priv->status.tx_queue_freespace -= packets_expected;
		else
			priv->status.tx_queue_freespace = 0;

		if (priv->status.tx_queue_freespace < TX_QUEUE_THRESHOLD)
			return -1;
	}

	/* If zero is returned, the polling will continue until all connections have
	 * been examined.
	 */

	return OK;
}

/************************************************************************************
 * Name: rf_txavail
 ************************************************************************************/

static int rf_txavail(struct uip_driver_s *dev)
{
	FAR struct rf_dev_s *priv = (FAR struct rf_dev_s *)dev->d_private;
	irqstate_t flags;

	/* Lock the SPI bus so that we have exclusive access */

	rf_lock(priv);

	/* Ignore the notification if the interface is not yet up */

	flags = irqsave();
	if (priv->ifup) {
		/* Check if the hardware is ready to send another packet */

		if (rf_can_transimt(priv)) {
			/* The interface is up and TX is idle; poll uIP for new XMIT data */

			(void)uip_poll(&priv->dev, rf_uiptxpoll);
		}
	}

	/* Un-lock the SPI bus */

	irqrestore(flags);
	rf_unlock(priv);
	return OK;
}

static int rf_interrupt(int irq, void *context)
{
	FAR struct rf_dev_s *priv = g_priv;

	up_disable_irq(priv->cfg->rx_irq);
	return work_queue(HPWORK, &priv->irqwork, rf_irqworker, (FAR void *)priv, 0);
}

/************************************************************************************
 * Name: omega_rf_initialize
 ************************************************************************************/

int omega_rf_initialize(FAR struct spi_dev_s *spi, const FAR omega_rf_cfg *cfg)
{
  FAR struct rf_dev_s *priv;

  nllvdbg("init omega_rf\n");

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per SPI
   * device (only because of the SPIDEV_WIRELESS definition) and so would have
   * to be extended to handle multiple FLASH parts on the same SPI bus.
   */

  priv = (FAR struct rf_dev_s *)kzalloc(sizeof(struct rf_dev_s));
  if (priv)
    {
      GPIO_CONFIG(cfg->nrst_gpio);
      GPIO_CONFIG(cfg->irq_gpio);
      GPIO_CONFIG(cfg->rdy_gpio);
      
      g_priv = priv;

      GPIO_IRQ(cfg->irq_gpio);
      irq_attach(cfg->rx_irq, rf_interrupt);

      priv->cfg = cfg;
      priv->status.tx_queue_freespace = 0;
      priv->ready_phase   = 0;

      priv->dev.d_ifup    = rf_ifup;     /* I/F down callback */
      priv->dev.d_ifdown  = rf_ifdown;   /* I/F up (new IP address) callback */
      priv->dev.d_txavail = rf_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
      priv->dev.d_addmac  = NULL;        /* Add multicast MAC address */
      priv->dev.d_rmmac   = NULL;        /* Remove multicast MAC address */
#endif
      priv->dev.d_private = priv;         /* Used to recover private state from dev */

      priv->txpoll        = wd_create();
      priv->txtimeout     = wd_create();  /* Create TX timeout timer */

      priv->spi           = spi;

      memset(priv->dev.d_mac.ether_addr_octet, 0, IFHWADDRLEN);

      /* Deselect the RF */
      SPI_SELECT(spi, priv->cfg->spidev, false);

      return netdev_register(&priv->dev);
    }

  return -1;
}