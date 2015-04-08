/*

2.4 kbps MELP Proposed Federal Standard speech coder

version 1.2

Copyright (c) 1996, Texas Instruments, Inc.  

Texas Instruments has intellectual property rights on the MELP
algorithm.  The Texas Instruments contact for licensing issues for
commercial and non-government use is William Gordon, Director,
Government Contracts, Texas Instruments Incorporated, Semiconductor
Group (phone 972 480 7442).


*/

/*
   spbstd.h   SPB standard header file.

   Copyright (c) 1995 by Texas Instruments, Inc.  All rights reserved.
*/

#ifndef _spbstd_h
#define _spbstd_h

/*
** Needed include files.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef _WIN32
#include <cmsis_os.h>
#endif
/* OSTYPE-dependent definitions/macros. */

/*
** Constant definitions.
*/
#ifndef TRUE
#define TRUE            1
#endif
#ifndef FALSE
#define FALSE           0
#endif
#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

/*
** Macros.
*/
#ifndef SQR
#define SQR(x)          ((x)*(x))
#endif

/* Generic memory allocation/deallocation macros. */
#define MEM_ALLOC(alloc_routine, v, n, type)   (v) = (type*) alloc_routine((n) * sizeof(type))
#define MEM_2ALLOC(alloc_routine,v,n,k,type) \
	do { \
		v = (type**)alloc_routine((n) * sizeof(type*)); v[0]=(type*) alloc_routine((n)*(k)*sizeof(type));\
		{int u__i; for(u__i=1; u__i < n; u__i++) v[u__i] = &v[u__i-1][k]; } \
	} while(0)

#define MEM_FREE(free_routine, v) free_routine(v)
#define MEM_2FREE(free_routine, v) do { free_routine((v)[0]); free_routine(v); } while(0)

#ifdef _WIN32
#define MALLOC(n)   malloc((unsigned)(n))
#define FREE(v)     free((void*)(v))
#else
#define MALLOC(n)   osAlloc((unsigned)(n))
#define FREE(v)     osFree((void*)(v))
#endif

#endif /* #ifndef _spbstd_h */
