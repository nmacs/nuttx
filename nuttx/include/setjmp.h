#ifndef	_SETJMP_H
#define	_SETJMP_H	1

#include "bits_setjmp.h"

typedef struct __jmp_buf_tag
  {
    __jmp_buf __jmpbuf;
  } jmp_buf[1];

extern int setjmp (jmp_buf __env);
extern void longjmp (jmp_buf __env, int __val) __attribute__ ((__noreturn__));

#define _setjmp setjmp
#define _longjmp longjmp

#endif /* setjmp.h  */
