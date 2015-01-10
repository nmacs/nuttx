/****************************************************************************
 * crypto/crypto.c
 *
 *   Copyright (C) 2014 Max Nekludov. All rights reserved.
 *   Author: Max Nekludov <macscomp@gmail.com>
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
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <nuttx/fs/fs.h>
#include <crypto/crypto.h>
#include <crypto/cryptodev.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void xor(uint32_t *R, uint32_t *A, uint32_t *B)
{
  R[0] = A[0] ^ B[0];
  R[1] = A[1] ^ B[1];
  R[2] = A[2] ^ B[2];
  R[3] = A[3] ^ B[3];
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int crypto_xex(FAR void *out, FAR const void *in, uint32_t size, FAR const void *key, 
               uint32_t keysize, uint32_t sector, uint32_t offset, int encrypt)
{
  uint32_t blocks = size / AES_BLOCK_LEN;
  uint32_t *in_buffer = (uint32_t*)in;
  uint32_t *out_buffer = (uint32_t*)out;
  uint32_t i;
  int ret;

  offset = offset / AES_BLOCK_LEN;

  for (i = 0; i < blocks; i++, in_buffer += AES_BLOCK_LEN / sizeof(uint32_t), out_buffer += AES_BLOCK_LEN / sizeof(uint32_t) )
    {
      uint32_t T[4];
      uint32_t X[4] = {sector, 0, 0, i + offset};

      ret = aes_cypher(X, X, AES_BLOCK_LEN, NULL, key, keysize, AES_MODE_ECB, CYPHER_ENCRYPT);
      if (ret)
        return ret;

      // Xor-Encrypt-Xor
      xor(T, X, in_buffer);

      ret = aes_cypher(T, T, AES_BLOCK_LEN, NULL, key, keysize, AES_MODE_ECB, encrypt);
      if (ret)
        return ret;

      xor(out_buffer, X, T);
    }

  return OK;
}