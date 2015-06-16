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
	int numTaps, numBlk;
	float accum0, accum1, accum2, accum3;
	float a1, a2, a3, a4, c0;
	float y1, y2, y3, y4;
	float *pc; 
	float *py;
	a1 = coeff[1]; a2 = coeff[2]; a3 = coeff[3]; a4 = coeff[4]; 
	y1 = output[-1]; y2 = output[-2]; y3 = output[-3]; y4 = output[-4];
	numBlk = npts;
	while (numBlk >= 4) {
		accum0 = *input++ - (y1 * a1) - (y2 * a2) - (y3 * a3) - (y4 * a4);
		accum1 = *input++ - (y1 * a2) - (y2 * a3) - (y3 * a4);
		accum2 = *input++ - (y1 * a3) - (y2 * a4);
		accum3 = *input++ - (y1 * a4);

		pc = &coeff[5];
		py = &output[-5];
		numTaps = order - 4;
		while(numTaps > 0)
		{
			c0 = *pc++;
			y1 = y2;
			y2 = y3;
			y3 = y4;
			y4 = *py--;
			accum0 -= ( y4 * c0);
			accum1 -= ( y3 * c0);
			accum2 -= ( y2 * c0);
			accum3 -= ( y1 * c0);
			numTaps--;
		}
		accum1 -= (accum0 * a1);
		accum2 -= (accum1 * a1) + (accum0 * a2);
		accum3 -= (accum2 * a1) + (accum1 * a2) + (accum0 * a3);

		y4 = *output++ = accum0;
		y3 = *output++ = accum1;
		y2 = *output++ = accum2;
		y1 = *output++ = accum3;
		numBlk -= 4;
	}
	// For the rest (non-multiples of 4) of samples do it normally
	while(numBlk > 0)
	{
		accum0 = *input++;
		pc = &coeff[1];
		py = &output[-1];
		numTaps = order;
		while(numTaps > 0)
		{
			accum0 -= ( (*py--) * (*pc++) );
			numTaps--;
		}
		*output++ = accum0;
		numBlk--;
	}
}


/*								*/
/*	Subroutine iirflt: all pole (IIR) filter.		*/
/*	Note: The filter coefficients represent the		*/
/*	denominator only, and the leading coefficient		*/
/*	is assumed to be 1.					*/
/*      The output array can overlay the input.                 */
/*								*/
void iirflt(float input[], const float coeff[], float output[], float delay[], int order,int npts)
{
	v_equ(&output[-order], delay, order);
	polflt(input, coeff, output, order, npts);
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
	zerflt(input, coeff, output, order, npts);
}

void zerflt(float *pSrc, const float *pCoeffs, float *pDst, int order, int npts)
{
   const float *px, *pb;                      /* Temporary pointers for state and coefficient buffers */
   float acc0, acc1, acc2, acc3;			  /* Accumulators */
   float x0, x1, x2, x3, c0;				  /* Temporary variables to hold state and coefficient values */
   uint32_t numTaps, tapCnt, blkCnt;          /* Loop counters */
   float p0,p1,p2,p3;						  /* Temporary product values */

   pSrc += (npts - 1);
   pDst += (npts - 1);
   numTaps = order + 1;	


   /* Apply loop unrolling and compute 4 output values simultaneously.  
    * The variables acc0 ... acc3 hold output values that are being computed  
   */
   blkCnt = npts;

   /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.  
   ** a second loop below computes the remaining 1 to 3 samples. */
   while(blkCnt >= 4)
   {
      /* Set all accumulators to zero */
      acc0 = 0.0f;
      acc1 = 0.0f;
      acc2 = 0.0f;
      acc3 = 0.0f;

	  /* Initialize state pointer */
      px = pSrc;

      /* Initialize coeff pointer */
      pb = pCoeffs;		
   
      /* Read the first three samples from the state buffer */
      x0 = *px--;
      x1 = *px--;
      x2 = *px--;

      /* Loop unrolling.  Process 4 taps at a time. */
      tapCnt = numTaps;
      
      /* Loop over the number of taps.  Unroll by a factor of 4.  
       ** Repeat until we've computed numTaps-4 coefficients. */
      while(tapCnt >= 4)
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

		 tapCnt -= 4;
     }

      /* If the filter length is not a multiple of 4, compute the remaining filter taps */
      while(tapCnt > 0)
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


      /* The results in the 4 accumulators, store in the destination buffer. */
      *pDst-- = acc0;
      *pDst-- = acc1;
      *pDst-- = acc2;
      *pDst-- = acc3;

      /* Advance the state pointer by 4 to process the next group of 4 samples */
      pSrc -= 4;
      blkCnt -= 4;
   }

   /* If the blockSize is not a multiple of 4, compute any remaining output samples here.  
   ** No loop unrolling is used. */

   while(blkCnt > 0)
   {
      /* Set the accumulator to zero */
      acc0 = 0.0f;
      /* Initialize state pointer */
      px = pSrc;
	  /* Initialize Coefficient pointer */
      pb = pCoeffs;
      tapCnt = numTaps;

	  /* Perform the multiply-accumulates */
      do
      {
         acc0 += *px-- * *pb++;
         tapCnt--;
      } while(tapCnt > 0);

      /* The result is store in the destination buffer. */
      *pDst-- = acc0;

      /* Advance state pointer by 1 for the next sample */
      pSrc--;
      blkCnt--;
   }
}

//void zerflt(float input[], const float coeff[], float output[], int order, int npts)
//{
//    int i,j;
//    float accum;


//	firflt_f32(input, coeff, output, order, npts);
//    for (i = npts-1; i >= 0; i-- ) {
//		accum = 0.0;
//		for (j = 0; j <= order; j++ )
//			accum += input[i-j] * coeff[j];
//		output[i] = accum;
//    }
//}
