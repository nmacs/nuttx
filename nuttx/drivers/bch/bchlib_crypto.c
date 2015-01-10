/****************************************************************************
 * drivers/bch/bchlib_crypto.c
 *
 *   Copyright (C) 2014 Nekludov Max. All rights reserved.
 *   Author: Nekludov Max <macscomp@gmail.com>
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
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <crypto/crypto.h>

#include "bch_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bchlib_encryptcache(FAR struct bchlib_s *bch)
{
  return crypto_xex(bch->buffer, bch->buffer, bch->sectsize, bch->key,
                    CONFIG_BCH_ENCRIPTION_KEY_SIZE, bch->sector, CYPHER_ENCRYPT);
}

int bchlib_decryptcache(FAR struct bchlib_s *bch)
{
  return crypto_xex(bch->buffer, bch->buffer, bch->sectsize, bch->key, 
                    CONFIG_BCH_ENCRIPTION_KEY_SIZE, bch->sector, CYPHER_DECRYPT);
}

int bchlib_encrypt(FAR struct bchlib_s *bch, void *out, void *in, uint32_t sector)
{
  return crypto_xex(out, in, bch->sectsize, bch->key,
                    CONFIG_BCH_ENCRIPTION_KEY_SIZE, sector, CYPHER_ENCRYPT);
}

int bchlib_decrypt(FAR struct bchlib_s *bch, void *out, void *in, uint32_t sector)
{
  return crypto_xex(out, in, bch->sectsize, bch->key, 
                    CONFIG_BCH_ENCRIPTION_KEY_SIZE, sector, CYPHER_DECRYPT);
}