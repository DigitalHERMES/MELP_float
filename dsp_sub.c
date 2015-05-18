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

  dsp_sub.c: general subroutines.

*/

/*  compiler include files  */
#include	<stdio.h>
#include	<stdlib.h>
#include	<math.h>
#include	"dsp_sub.h"
#include	"spbstd.h"
#include	"mat.h"

#define MAXSORT 5
static float sorted[MAXSORT];

/*								*/
/*	Subroutine autocorr: calculate autocorrelations         */
/*								*/
void autocorr(float input[], float r[], int order, int npts)
{
    int i;

    for (i = 0; i <= order; i++ )
	{
      r[i] = v_inner(&input[0],&input[i],(npts-i));
	}
	if (r[0] < 1.0F) r[0] = 1.0F;
}

/*								*/
/*	Subroutine envelope: calculate time envelope of signal. */
/*      Note: the delay history requires one previous sample    */
/*      of the input signal and two previous output samples.    */
/*								*/
#define C2 (-0.9409F)
#define C1 1.9266F

void envelope(float input[], float prev_in, float output[], int npts)
{
    int i;
    float curr_abs, prev_abs;
    prev_abs = fabsf(prev_in);
    for (i = 0; i < npts; i++) {
		curr_abs = fabsf(input[i]);
		output[i] = curr_abs - prev_abs + C2*output[i-2] + C1*output[i-1];
		prev_abs = curr_abs;
    }
}


/*								*/
/*	Subroutine interp_array: interpolate array              */
/*                                                              */
void interp_array(float prev[],float curr[],float out[],float ifact,int npts)
{
    int i;
    float ifact2;

    ifact2 = 1.0F - ifact;
    for (i = 0; i < npts; i++)
      out[i] = ifact*curr[i] + ifact2*prev[i];
}

/*								*/
/*	Subroutine median: calculate median value               */
/*								*/

float median(float input[], int npts)
{
    int i,j,loc;
    float insert_val;


    /* sort data in temporary array */
    v_equ(sorted,input,npts);
    for (i = 1; i < npts; i++) {
		/* for each data point */
		for (j = 0; j < i; j++) {
			/* find location in current sorted list */
			if (sorted[i] < sorted[j])
			  break;
		}

		/* insert new value */
		loc = j;
		insert_val = sorted[i];
		for (j = i; j > loc; j--)
		  sorted[j] = sorted[j-1];
		sorted[loc] = insert_val;
    }
    return(sorted[npts/2]);
}

/*								*/
/*	Subroutine PACK_CODE: Pack bit code into channel.	*/
/*								*/
void pack_code(int code,unsigned int **p_ch_beg,int *p_ch_bit, int numbits, int wsize)
{
    int	i,ch_bit;
    unsigned int *ch_word;

	ch_bit = *p_ch_bit;
	ch_word = *p_ch_beg;

	for (i = 0; i < numbits; i++) {
		/* Mask in bit from code to channel word	*/
		if (ch_bit == 0)
		  *ch_word = ((code & (1<<i)) >> i);
		else
		  *ch_word |= (((code & (1<<i)) >> i) << ch_bit);

		/* Check for full channel word			*/
		if (++ch_bit >= wsize) {
			ch_bit = 0;
			(*p_ch_beg)++ ;
			ch_word++ ;
		}
	}

	/* Save updated bit counter	*/
	*p_ch_bit = ch_bit;
}

/*								*/
/*	Subroutine peakiness: estimate peakiness of input       */
/*      signal using ratio of L2 to L1 norms.                   */
/*								*/
float peakiness(float input[], int npts)
{
    int i;
    float sum_abs, peak_fact;

    sum_abs = 0.0;
    for (i = 0; i < npts; i++)
      sum_abs += fabsf(input[i]);

    if (sum_abs > 0.01F)
      peak_fact = arm_sqrt(npts*v_magsq(input,npts)) / sum_abs;
    else
      peak_fact = 0.0;

    return(peak_fact);
}

/*								*/
/*	Subroutine QUANT_U: quantize positive input value with 	*/
/*	symmetrical uniform quantizer over given positive	*/
/*	input range.						*/
/*								*/
void quant_u(float *p_data, int *p_index, float qmin, float qmax, int nlev)
{
	register int	i, j;
	register float	step, qbnd, *p_in;

	p_in = p_data;

	/*  Define symmetrical quantizer step-size	*/
	step = (qmax - qmin) / (nlev - 1);

	/*  Search quantizer boundaries			*/
	qbnd = qmin + (0.5f * step);
	j = nlev - 1;
	for (i = 0; i < j; i ++ ) {
		if (*p_in < qbnd)
			break;
		else
			qbnd += step;
	}

	/*  Quantize input to correct level		*/
	*p_in = qmin + (i * step);
	*p_index = i;
}

/*								*/
/*	Subroutine QUANT_U_DEC: decode uniformly quantized	*/
/*	value.							*/
/*								*/
void quant_u_dec(int index, float *p_data,float qmin, float qmax, int nlev)
{
	register float	step;

	/*  Define symmetrical quantizer stepsize	*/
	step = (qmax - qmin) / (nlev - 1);

	/*  Decode quantized level			*/
	*p_data = qmin + (index * step);
}

/*								*/
/*	Subroutine rand_num: generate random numbers to fill    */
/*      array using system random number generator.             */
/*                                                              */
void	rand_num(float output[], float amplitude, int npts)
{
    int i;

    for (i = 0; i < npts; i++ ) {
		/* use system random number generator from -1 to +1 */
		output[i] = (amplitude*2.0f) * ((float) rand()*(1.0f/RAND_MAX) - 0.5f);
    }
}


/*								*/
/*	Subroutine UNPACK_CODE: Unpack bit code from channel.	*/
/*      Return 1 if erasure, otherwise 0.                       */
/*								*/
int unpack_code(unsigned int **p_ch_beg, int *p_ch_bit, int *p_code, int numbits, int wsize, unsigned int ERASE_MASK)
{
    int ret_code;
    int	i,ch_bit;
    unsigned int *ch_word;

	ch_bit = *p_ch_bit;
	ch_word = *p_ch_beg;
	*p_code = 0;
        ret_code = *ch_word & ERASE_MASK;

	for (i = 0; i < numbits; i++) {
		/* Mask in bit from channel word to code	*/
		*p_code |= (((*ch_word & (1<<ch_bit)) >> ch_bit) << i);

		/* Check for end of channel word		*/
		if (++ch_bit >= wsize) {
			ch_bit = 0;
			(*p_ch_beg)++ ;
			ch_word++ ;
		}
	}

	/*  Save updated bit counter	*/
	*p_ch_bit = ch_bit;

    /* Catch erasure in new word if read */
    if (ch_bit != 0)
      ret_code |= *ch_word & ERASE_MASK;

    return(ret_code);
}


/*								*/
/*	Subroutine polflt: all pole (IIR) filter.		*/
/*	Note: The filter coefficients represent the		*/
/*	denominator only, and the leading coefficient		*/
/*	is assumed to be 1.					*/
/*      The output array can overlay the input.                 */
/*								*/
void polflt(float input[], const float coeff[], float output[], int order,int npts)
{
//	int i,j;
//	float accum;
	arm_iirflt_f32(input, coeff, output, order, npts);
//	for (i = 0; i < npts; i++ ) {
//		accum = input[i];
//		for (j = 1; j <= order; j++ )
//			accum -= output[i-j] * coeff[j];
//		output[i] = accum;
//	}
}

/*								*/
/*	Subroutine iirflt: all pole (IIR) filter.		*/
/*	Note: The filter coefficients represent the		*/
/*	denominator only, and the leading coefficient		*/
/*	is assumed to be 1.					*/
/*      The output array can overlay the input.                 */
/*								*/
float  testInput[300];
float  testOutput[300];

void iirflt(float input[], const float coeff[], float output[], float delay[], int order,int npts)
{
	int i,j;
	float accum;

	v_equ(testInput, input, npts);
	v_equ(&testOutput[0], delay, order);

	v_equ(&output[-order], delay, order);
	for (i = 0; i < npts; i++ ) {
		accum = input[i];
		for (j = 1; j <= order; j++ )
			accum -= output[i-j] * coeff[j];
		output[i] = accum;
	}

	arm_iirflt_f32(testInput, coeff, &testOutput[order], order, npts);

	v_equ(delay,&output[npts - order], order);
}


/*								*/
/*	Subroutine firflt: all zero (FIR) filter.		*/
/*      Note: the output array can overlay the input.           */
/*								*/
void firflt(float input[], const float coeff[], float output[], float delay[], int order, int npts)
{
	v_equ(&input[-order], delay, order);
	v_equ(delay, &input[npts - order], order);
	arm_firflt_f32(input,coeff, output, order, npts);

//	for (i = npts-1; i >= 0; i-- ) {
//		accum = 0.0;
//		for (j = 0; j <= order; j++ )
//			accum += input[i-j] * coeff[j];
//		output[i] = accum;
//	}
}

void zerflt(float input[], const float coeff[], float output[], int order, int npts)
{
	arm_firflt_f32(input,coeff, output, order, npts);
//    for (i = npts-1; i >= 0; i-- ) {
//		accum = 0.0;
//		for (j = 0; j <= order; j++ )
//			accum += input[i-j] * coeff[j];
//		output[i] = accum;
//    }
}

void arm_firflt_f32(float *pSrc, const float *pCoeffs, float *pDst, int order, int npts)
{
   const float *px, *pb;                            /* Temporary pointers for state and coefficient buffers */
   float acc0, acc1, acc2, acc3;			  /* Accumulators */
   float x0, x1, x2, x3, c0;				  /* Temporary variables to hold state and coefficient values */
   uint32_t numTaps, tapCnt, blkCnt;          /* Loop counters */
   float p0,p1,p2,p3;						  /* Temporary product values */

   pSrc += (npts - 1u);
   pDst += (npts - 1u);
   numTaps = order + 1;	


   /* Apply loop unrolling and compute 4 output values simultaneously.  
    * The variables acc0 ... acc3 hold output values that are being computed:  
   */
   blkCnt = npts >> 2;

   /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.  
   ** a second loop below computes the remaining 1 to 3 samples. */
   while(blkCnt > 0u)
   {
      /* Set all accumulators to zero */
      acc0 = 0.0f;
      acc1 = 0.0f;
      acc2 = 0.0f;
      acc3 = 0.0f;

	  /* Initialize state pointer */
      px = pSrc;

      /* Initialize coeff pointer */
      pb = (pCoeffs);		
   
      /* Read the first three samples from the state buffer */
      x0 = *px--;
      x1 = *px--;
      x2 = *px--;

      /* Loop unrolling.  Process 4 taps at a time. */
      tapCnt = numTaps >> 2u;
      
      /* Loop over the number of taps.  Unroll by a factor of 4.  
       ** Repeat until we've computed numTaps-4 coefficients. */
      while(tapCnt > 0u)
      {
         /* Read the b[0] coefficient */
         c0 = *(pb++);

         x3 = *(px--);

         p0 = x0 * c0;
         p1 = x1 * c0;
         p2 = x2 * c0;
         p3 = x3 * c0;

         /* Read the b[1] coefficient */
         c0 = *(pb++);
		 x0 = *(px--);
         
         acc0 += p0;
         acc1 += p1;
         acc2 += p2;
         acc3 += p3;

         /* Perform the multiply-accumulate */
         p0 = x1 * c0;
         p1 = x2 * c0;   
         p2 = x3 * c0;   
         p3 = x0 * c0;   
         
         /* Read the b[2] coefficient */
         c0 = *(pb++);
         x1 = *(px--);
         
         acc0 += p0;
         acc1 += p1;
         acc2 += p2;
         acc3 += p3;

         /* Perform the multiply-accumulates */      
         p0 = x2 * c0;
         p1 = x3 * c0;   
         p2 = x0 * c0;   
         p3 = x1 * c0;   

		 /* Read the b[3] coefficient */
         c0 = *(pb++);
         x2 = *(px--);
         
         acc0 += p0;
         acc1 += p1;
         acc2 += p2;
         acc3 += p3;

         /* Perform the multiply-accumulates */      
         p0 = x3 * c0;
         p1 = x0 * c0;   
         p2 = x1 * c0;   
         p3 = x2 * c0;   

         acc0 += p0;
         acc1 += p1;
         acc2 += p2;
         acc3 += p3;

		 tapCnt--;
     }

      /* If the filter length is not a multiple of 4, compute the remaining filter taps */
      tapCnt = numTaps % 0x4u;

      while(tapCnt > 0u)
      {
         /* Read coefficients */
         c0 = *(pb++);

         /* Fetch 1 state variable */
         x3 = *(px--);

         /* Perform the multiply-accumulates */      
         p0 = x0 * c0;
         p1 = x1 * c0;   
         p2 = x2 * c0;   
         p3 = x3 * c0;   

         /* Reuse the present sample states for next sample */
         x0 = x1;
         x1 = x2;
         x2 = x3;
         
         acc0 += p0;
         acc1 += p1;
         acc2 += p2;
         acc3 += p3;
         /* Decrement the loop counter */
         tapCnt--;
      }

      /* Advance the state pointer by 4 to process the next group of 4 samples */
      pSrc = pSrc - 4;

      /* The results in the 4 accumulators, store in the destination buffer. */
      *pDst-- = acc0;
      *pDst-- = acc1;
      *pDst-- = acc2;
      *pDst-- = acc3;

      blkCnt--;
   }

   /* If the blockSize is not a multiple of 4, compute any remaining output samples here.  
   ** No loop unrolling is used. */
   blkCnt = npts % 0x4u;
   while(blkCnt > 0u)
   {
      /* Set the accumulator to zero */
      acc0 = 0.0f;
      /* Initialize state pointer */
      px = pSrc;
	  /* Initialize Coefficient pointer */
      pb = (pCoeffs);
      tapCnt = numTaps;

	  /* Perform the multiply-accumulates */
      do
      {
         acc0 += *px-- * *pb++;
         tapCnt--;
      } while(tapCnt > 0u);

      /* The result is store in the destination buffer. */
      *pDst-- = acc0;

      /* Advance state pointer by 1 for the next sample */
      pSrc--;
      blkCnt--;
   }
}


void arm_iirflt_f32(float *pSrc, const float *pCoeffs, float *pDst, int order, int npts)
{
   const float *py, *pa;                      /* Temporary pointers for state and coefficient buffers */
   float acc0, acc1, acc2, acc3;			  /* Accumulators */
   float y1, y2, y3, y4;					  /* Temporary variables to hold state and coefficient values */
   int32_t numTaps, tapCnt, blkCnt;			  /* Loop counters */
   float a1, a2, a3, a4;					  /* Temporary coefficients */

   numTaps = order ;						  /* The number of used previous outputs  */


   /* Apply loop unrolling and compute 4 output values simultaneously.  
    * The variables acc0 ... acc3 hold output values that are being computed:  
   */
   blkCnt = npts >> 2;
   /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.  
   ** a second block below computes the remaining 1 to 3 samples. */
   while(blkCnt > 0u)
   {
      /* Set all accumulators to x[0], x[1] x[2] x[3] */
      acc0 = *pSrc++;
      acc1 = *pSrc++;
      acc2 = *pSrc++;
      acc3 = *pSrc++;

	  /* Initialize state pointer */
      py = pDst - numTaps;
      /* Initialize coeff pointer */
      pa = pCoeffs + numTaps;		
   
      /* Read the first four samples from the state buffer */
      y1 = *py++;
      y2 = *py++;
      y3 = *py++;
      y4 = *py++;

      /* Loop unrolling.  Process 4 taps at a time. */
      tapCnt = numTaps;
      
      /* Loop over the number of taps.  Unroll by a factor of 4.  
       ** Repeat until we've computed numTaps-4 coefficients. */
      while(tapCnt > 4)
      {
         /* Read the b[nTaps] coefficient */
         a4 = *(pa--);

         acc0 -= y1 * a4;
         acc1 -= y2 * a4;
         acc2 -= y3 * a4;
         acc3 -= y4 * a4;

		 /* Read the b[nTaps - 1] coefficient */
         a3 = *(pa--);
		 y1 = *(py++);

		 /* Perform the multiply-accumulate */
         acc0 -= y2 * a3;
         acc1 -= y3 * a3;
         acc2 -= y4 * a3;
         acc3 -= y1 * a3;

         /* Read the b[nTaps - 2] coefficient */
         a2 = *(pa--);
         y2 = *(py++);
         
         /* Perform the multiply-accumulates */      
         acc0 -= y3 * a2;
         acc1 -= y4 * a2;
         acc2 -= y1 * a2;
         acc3 -= y2 * a2;

		 /* Read the b[nTaps - 3] coefficient */
         a1 = *(pa--);
         y3 = *(py++);

         /* Perform the multiply-accumulates */      
         acc0 -= y4 * a1;
         acc1 -= y1 * a1;
         acc2 -= y2 * a1;
         acc3 -= y3 * a1;

		 y4 = *(py++);
		 tapCnt -= 4;
     }

      /* If the filter length is not a multiple of 4, compute the remaining filter taps */
      switch(tapCnt)
      {
	  case 1:
         a1 = *(pa--);
		 acc0 -= y4 * a1;
		 acc1 -= acc0 * a1;
		 acc2 -= acc1 * a1;
		 acc3 -= acc3 * a1;
		 break;

	  case 2:
         a2 = *(pa--);
         a1 = *(pa--);
		 acc0 -= (y4 * a1 + y3 * a2) ;
		 acc1 -= (acc0 * a1 + y4 * a2);
		 acc2 -= (acc1 * a1 + acc0 * a2);
		 acc3 -= (acc2 * a1 + acc1 * a2);
		 break;

	  case 3:
         a3 = *(pa--);
         a2 = *(pa--);
         a1 = *(pa--);
		 acc0 -= (y4 * a1 + y3 * a2 + y2 * a3) ;
		 acc1 -= (acc0 * a1 + y4 * a2 + y3 * a3);
		 acc2 -= (acc1 * a1 + acc0 * a2 + y4 * a3);
		 acc3 -= (acc2 * a1 + acc1 * a2 + acc0 * a3);
		 break;

	  case 4:
         a4 = *(pa--);
         a3 = *(pa--);
         a2 = *(pa--);
         a1 = *(pa--);
		 acc0 -= (y4 * a1 + y3 * a2 + y2 * a3 + y1 * a4) ;
		 acc1 -= (acc0 * a1 + y4 * a2 + y3 * a3 + y2 * a4);
		 acc2 -= (acc1 * a1 + acc0 * a2 + y4 * a3 + y3 * a4);
		 acc3 -= (acc2 * a1 + acc1 * a2 + acc0 * a3 + y4 * a4);
		 break;

	  default:
		 break;
      }

      /* The results in the 4 accumulators, store in the destination buffer. */
      *pDst++ = acc0;
      *pDst++ = acc1;
      *pDst++ = acc2;
      *pDst++ = acc3;

      blkCnt--;
   }

   /* If the blockSize is not a multiple of 4, compute any remaining output samples here.  
   ** No loop unrolling is used. */
   blkCnt = npts % 0x4u;
   while(blkCnt > 0u)
   {
      /* Set the accumulator to X[0] */
      acc0 = *pSrc++;
	   /* Initialize state pointer */
      py = pDst - 1;
	  /* Initialize Coefficient pointer */
      pa = pCoeffs + 1;

      tapCnt = numTaps;
      /* Perform the multiply-accumulates */
      do
      {
         acc0 -= (*py-- * *pa++);
         tapCnt--;
      } while(tapCnt > 0u);

      /* The result is stored in the destination buffer. */
      *pDst++ = acc0;
      blkCnt--;
   }
}
