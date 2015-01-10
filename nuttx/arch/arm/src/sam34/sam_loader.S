/************************************************************************************************
 * arch/arm/src/sam34/sam_loader.S
 *
 *   Copyright (C) 2014, Max Nekludov. All rights reserved.
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
 ************************************************************************************************/

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************************
 * Preprocessor Definitions
 ************************************************************************************************/

#define LOADER_STACK (CONFIG_RAM_START + CONFIG_RAM_SIZE)
#define UNUSED       (0xFFFFFFFF)

/************************************************************************************************
 * Global Symbols
 ************************************************************************************************/

	.syntax		unified
	.thumb
	.file		"sam_loader.S"

/* Check if common ARMv7 interrupt vectoring is used (see arch/arm/src/armv7-m/up_vectors.S) */

#ifdef CONFIG_ARMV7M_CMNVECTOR
#  error "CONFIG_ARMV7M_CMNVECTOR must not be defined"
#endif

/************************************************************************************************
 * Vectors
 ************************************************************************************************/

	.section	.loader, "ax"
	.code		16
	.align		2
	.globl		_vectors_loader
	.type		_vectors_loader, function

_vectors_loader:

/* Processor Exceptions */

	.word	LOADER_STACK		/* Vector  0: Reset stack pointer */
	.word	loader			/* Vector  1: Reset vector */
	.word	UNUSED			/* Vector  2: Non-Maskable Interrupt (NMI) */
	.word	UNUSED	/* Vector  3: Hard fault */
	.word	UNUSED			/* Vector  4: Memory management (MPU) */
	.word	UNUSED	/* Vector  5: Bus fault */
	.word	UNUSED	/* Vector  6: Usage fault */
	.word	UNUSED	/* Vector  7: Reserved */
	.word	UNUSED	/* Vector  8: Reserved */
	.word	UNUSED	/* Vector  9: Reserved */
	.word	UNUSED	/* Vector 10: Reserved */
	.word	UNUSED		/* Vector 11: SVC call */
	.word	UNUSED	/* Vector 12: Debug monitor */
	.word	UNUSED	/* Vector 13: Reserved */
	.word	UNUSED		/* Vector 14: Pendable system service request */
	.word	UNUSED		/* Vector 15: System tick */

	.size	_vectors_loader, .-_vectors_loader

/************************************************************************************************
 * .loader
 ************************************************************************************************/

#ifdef CONFIG_SAM_LOADER
#define CONFIG_SAM_LOADER_SWITCH_TO 0x000F0000
#define CONFIG_SAM_LOADER_MAIN      0x00000400

	.section	.loader
	.type	loader, function
	.thumb_func
loader:
	ldr r2, =_loader_switch_to
	ldr r1, [r2]
	ldr r2, =0xFFFFFFFF /* check if switch was set */
	cmp r1, r2
	bne 1f
	ldr r1, =CONFIG_SAM_LOADER_MAIN
	b   2f
1:
	ldr r1, =CONFIG_SAM_LOADER_SWITCH_TO
2:
	ldmia r1, {r2, r3}
	mov sp, r2
	mov pc, r3
#endif

	.end
