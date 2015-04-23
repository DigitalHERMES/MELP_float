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

#ifdef _WIN32
#define MALLOC(n)   malloc((unsigned)(n))
#define FREE(v)     free((void*)(v))
#else
#define MALLOC(n)   osAlloc((unsigned)(n))
#define FREE(v)     osFree((void*)(v))
#endif

#endif /* #ifndef _spbstd_h */
