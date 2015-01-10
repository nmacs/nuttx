/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include "up_internal.h"
#include "up_arch.h"

#ifdef __thumb2__
.thumb
.syntax unified
#define IT(t, cond) i##t cond
#else
/* XXX: This can be removed if/when we require an assembler that supports
   unified assembly syntax.  */
#define IT(t, cond)
/* Code to return from a thumb function stub.  */
#ifdef __ARM_ARCH_4T__
#define POP_RET pop	{r2, pc}
#else
#define POP_RET pop	{r2, r3}; bx	r3
#endif
#endif

#if defined(__ARM_ARCH_6M__)
/* Force arm mode to flush out errors on M profile cores.  */
#undef IT
#define THUMB1_ONLY 1
#endif

  .globl setjmp
  .type setjmp,%function
setjmp:
  mov ip, r0
  /* Save all the callee-preserved registers into the jump buffer.  */
#if defined(__thumb2__)
  stmia   ip!, { v1-v6, sl, fp }
  mov     r2, sp
  stmia   ip!, { r2, lr }
#else
  stmia	ip!, { v1-v6, sl, fp, sp, lr }
#endif

  /* When setting up the jump buffer return 0.  */
  mov   r0, #0

  mov pc, lr
  .size setjmp, .-setjmp

/* --------------------------------------------------------------------
    volatile void longjmp (jmp_buf, int);
   -------------------------------------------------------------------- */

  .globl longjmp
  .type longjmp,%function
longjmp:
  mov     ip, r0 /* save jmp_buf pointer */

  movs    r0, r1 /* get the return value in place */
  IT(t, eq)
  moveq   r0, #1 /* can't let setjmp() return zero! */

  /* Restore the registers, retrieving the state when setjmp() was called.  */
#if defined(__thumb2__)
  /* Thumb-2 does not allow loading sp with ldm.  */
  ldmia   ip!,  { v1-v6, sl, fp }
  ldr     sp, [ip], #4
  ldr     lr, [ip], #4
#else
  ldmia   ip!,  { v1-v6, sl, fp, sp, lr }
#endif

  mov pc, lr
  .size longjmp, .-longjmp

  .end