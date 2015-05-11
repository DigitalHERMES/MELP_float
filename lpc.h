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
   lpc.h     LPC include file.
*/


/* bandwidth expansion function */
int lpc_bw_expand(float *a, float *aw, float gamma, int p);

/* sort LSPs and ensure minimum separation */
int lpc_clamp(float *w, float delta, int p);

/* lpc conversion routines */
/* convert predictor parameters to LSPs */
int lpc_pred2lsp(float *a,float *w,int p);
/* convert predictor parameters to reflection coefficients */
int lpc_pred2refl(float *a,float *k,int p);
/* convert LSPs to predictor parameters */
int lpc_lsp2pred(float *w,float *a,int p);
/* convert reflection coefficients to predictor parameters */
int lpc_refl2pred(float *k,float *a,int p);

/* schur recursion */
float lpc_schur(float *r, float *a, int p);

/* evaluation of |A(e^jw)|^2 at a single point (using Horner's method) */
float lpc_aejw(float *a,float w,int p);


