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
   mat.h     Matrix include file.
             (Low level matrix and vector functions.)

   Copyright (c) 1995 by Texas Instruments, Inc.  All rights reserved.
*/
#ifndef  __MAT_H__
#define  __MAT_H__

#ifndef _MSC_VER
#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif
#define __TARGET_FPU_VFP 1
#define __FPU_PRESENT 1
#include <stdint.h>
#endif

#include "arm_math.h"
#include "arm_const_structs.h"

static __INLINE float v_inner(float *v1,float *v2,int n)
{
	float innerprod;
	arm_dot_prod_f32(v1, v2, n, &innerprod);
	return innerprod;
}

static __INLINE float v_magsq(float *v,int n)
{
	float magsq;
	arm_power_f32(v, n, &magsq);
	return magsq;
}

static __INLINE int findmax(float input[], int npts)
{
	uint32_t	maxloc;
	float   maxval;
	arm_max_f32(input, npts, &maxval, &maxloc);
	return (maxloc);
}

static __INLINE float arm_sqrt(float x)
{
	float32_t  res;
	arm_sqrt_f32(x, &res);
	return res;
}

static __INLINE float arm_sin(float x)
{
	float32_t  res;
	res = arm_sin_f32(x);
	return res;
}

static __INLINE float arm_cos(float x)
{
	float32_t  res;
	res = arm_cos_f32(x);
	return res;
}

//#define arm_sqrt		sqrtf
//#define arm_sin		sinf
//#define arm_cos		cosf

#define window(inp,cof,outp,n)		arm_mult_f32((float32_t *)inp, (float32_t *)cof, (float32_t *)outp, n)
#define v_zap(v,n)					arm_fill_f32(0.0f, (float32_t *)v, n)
#define v_fill(v,val,n)				arm_fill_f32(val, (float32_t *)v, n)
#define v_equ(v1,v2,n) 				arm_copy_f32((float32_t *)v2, (float32_t *)v1, n)
#define v_sub(v1,v2,n) 				arm_sub_f32((float32_t *)v1, (float32_t *)v2, (float32_t *)v1, n)
#define v_add(v1,v2,n) 				arm_add_f32((float32_t *)v1, (float32_t *)v2, (float32_t *)v1, n)
#define v_scale(v,scale,n)		    arm_scale_f32((float32_t *)v, scale, v, n)

#define v_zap_int(v,n)				arm_fill_q31(0, (q31_t *)v, n)
#define v_equ_int(v1,v2,n) 		    arm_copy_q31((q31_t *)v2, (q31_t *)v1, n)

#endif	//  __MAT_H__
