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

  mat_lib.c: Matrix and vector manipulation library

*/

#include "spbstd.h"
#include "mat.h"


///* V_ADD- vector addition */
//float *v_add(float *v1,float *v2,int n)
//{
//	arm_add_f32(v1, v2, v1, n);
//	return v1;
//}
//
///* V_EQU- vector equate */
//float *v_equ(float *v1,float *v2,int n)
//{
//	arm_copy_f32(v2, v1, n);
//	return v1;
//}
//int *v_equ_int(int *v1,int *v2,int n)
//{
//	arm_copy_q31(v2, v1, n);
//	return v1;
//}
//
///* V_INNER- inner product */
//float v_inner(float *v1,float *v2,int n)
//{
//    float innerprod;
//	arm_dot_prod_f32(v1, v2, n, &innerprod);
//	return innerprod;
//}
//
///* v_magsq - sum of squares */
//float v_magsq(float *v,int n)
//{
//    float magsq;
//
//	arm_power_f32(v, n, &magsq);
//	return magsq;
//} /* V_MAGSQ */
//
///* V_SCALE- vector scale */
//float *v_scale(float *v,float scale,int n)
//{
//	arm_scale_f32(v, scale, v, n);
//	return (v);
//}
//
///* V_SUB- vector difference */
//float *v_sub(float *v1,float *v2,int n)
//{
//	arm_sub_f32(v1, v2, v1, n);
//	return v1;
//}
//
///* v_zap - clear vector */
//
//float *v_zap(float *v,int n)
//{
//	arm_fill_f32(0.0f, v, n);
//	return v;
//} /* V_ZAP */
//
//int *v_zap_int(int *v,int n)
//{
//	arm_fill_q31(0, v, n);
//	return v;
//} /* V_ZAP */
