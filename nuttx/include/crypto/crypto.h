/****************************************************************************
 * include/crypto/crypto.h
 *
 *   Copyright (C) 2008, 2009, 2011-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_CRYPTO_CRYPTO_H
#define __INCLUDE_CRYPTO_CRYPTO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <debug.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#if defined(CONFIG_CRYPTO_AES)
#  define AES_MODE_MIN 1

#  define AES_MODE_ECB 1
#  define AES_MODE_CBC 2
#  define AES_MODE_CTR 3

#  define AES_MODE_MAX 3
#endif

#define CYPHER_ENCRYPT 1
#define CYPHER_DECRYPT 0

#ifdef CONFIG_DEBUG_CRYPTO
#  define cryptdbg lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define cryptvdbg lldbg
#  else
#    define cryptvdbg(x...)
#  endif
#else
#  define cryptdbg(x...)
#  define cryptvdbg(x...)
#endif


#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#if defined(CONFIG_CRYPTO_AES)
int up_aesinitialize(void);
int aes_cypher(FAR void *out, FAR const void *in, uint32_t size, FAR const void *iv, FAR const void *key, uint32_t keysize, int mode, int encrypt);
#endif

#if defined(CONFIG_CRYPTO_XEX)
int crypto_xex(FAR void *out, FAR const void *in, uint32_t size, FAR const void *key, uint32_t keysize, uint32_t sector, uint32_t offset, int encrypt);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __INCLUDE_CRYPTO_CRYPTO_H */