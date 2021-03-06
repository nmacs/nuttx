/****************************************************************************
 * net/udp/udp_send.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Adapted for NuttX from logic in uIP which also has a BSD-like license:
 *
 *   Original author Adam Dunkels <adam@dunkels.com>
 *   Copyright () 2001-2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP)

#include <string.h>
#include <debug.h>

#include <arpa/inet.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/udp.h>

#include "devif/devif.h"
#include "utils/utils.h"
#include "udp/udp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UDPBUF ((struct udp_iphdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_send
 *
 * Description:
 *   Set-up to send a UDP packet
 *
 * Parameters:
 *   dev  - The device driver structure to use in the send operation
 *   conn - The UDP "connection" structure holding port information
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

void udp_send(struct net_driver_s *dev, struct udp_conn_s *conn)
{
  FAR struct udp_iphdr_s *pudpbuf = UDPBUF;

  if (dev->d_sndlen > 0)
    {
      /* The total length to send is the size of the application data plus
       * the IP and UDP headers (and, eventually, the Ethernet header)
       */

      dev->d_len = dev->d_sndlen + IPUDP_HDRLEN;

      /* Initialize the IP header.  Note that for IPv6, the IP length field
       * does not include the IPv6 IP header length.
       */

#ifdef CONFIG_NET_IPv6

      pudpbuf->vtc         = 0x60;
      pudpbuf->tcf         = 0x00;
      pudpbuf->flow        = 0x00;
      pudpbuf->len[0]      = ((dev->d_len - IP_HDRLEN) >> 8);
      pudpbuf->len[1]      = ((dev->d_len - IP_HDRLEN) & 0xff);
      pudpbuf->proto       = IP_PROTO_UDP;
      pudpbuf->ttl         = conn->ttl;

      net_ipaddr_copy(pudpbuf->srcipaddr, dev->d_ipaddr);
      net_ipaddr_copy(pudpbuf->destipaddr, conn->ripaddr);

#else /* CONFIG_NET_IPv6 */

      pudpbuf->vhl         = 0x45;
      pudpbuf->tos         = 0;
      pudpbuf->len[0]      = (dev->d_len >> 8);
      pudpbuf->len[1]      = (dev->d_len & 0xff);
      ++g_ipid;
      pudpbuf->ipid[0]     = g_ipid >> 8;
      pudpbuf->ipid[1]     = g_ipid & 0xff;
      pudpbuf->ipoffset[0] = 0;
      pudpbuf->ipoffset[1] = 0;
      pudpbuf->ttl         = conn->ttl;
      pudpbuf->proto       = IP_PROTO_UDP;

      net_ipaddr_hdrcopy(pudpbuf->srcipaddr, &dev->d_ipaddr);
      net_ipaddr_hdrcopy(pudpbuf->destipaddr, &conn->ripaddr);

      /* Calculate IP checksum. */

      pudpbuf->ipchksum    = 0;
      pudpbuf->ipchksum    = ~(ip_chksum(dev));

#endif /* CONFIG_NET_IPv6 */

      /* Initialize the UDP header */

      pudpbuf->srcport     = conn->lport;
      pudpbuf->destport    = conn->rport;
      pudpbuf->udplen      = HTONS(dev->d_sndlen + UDP_HDRLEN);

#ifdef CONFIG_NET_UDP_CHECKSUMS
      /* Calculate UDP checksum. */

      pudpbuf->udpchksum   = 0;
      pudpbuf->udpchksum   = ~(udp_chksum(dev));
      if (pudpbuf->udpchksum == 0)
        {
          pudpbuf->udpchksum = 0xffff;
        }
#else
      pudpbuf->udpchksum   = 0;
#endif

      nllvdbg("Outgoing UDP packet length: %d (%d)\n",
              dev->d_len, (pudpbuf->len[0] << 8) | pudpbuf->len[1]);

#ifdef CONFIG_NET_STATISTICS
      g_netstats.udp.sent++;
      g_netstats.ip.sent++;
#endif
    }
}

#endif /* CONFIG_NET && CONFIG_NET_UDP */
