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

  fs_lib.c: Fourier series subroutines 

*/

/*  compiler include files  */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "spbstd.h"
#include "melp.h"
#include "mat.h"
#include "fs.h"


/*								*/
/*	Subroutine FIND_HARM: find Fourier coefficients using	*/
/*	FFT of input signal divided into pitch dependent bins.	*/
/*								*/
#define	FFTLENGTH	512
#define DFTMAX		160

/* Memory definition		*/
static float	find_hbuf[2*FFTLENGTH]	CCMRAM;
static float	mag[FFTLENGTH]			CCMRAM;
static float	idftc[DFTMAX]			CCMRAM;


void find_harm(float input[], float fsmag[], float pitch, int num_harm, 
	       int length)
{
    int	i, j, k, iwidth, i2;
    float temp, avg, fwidth;

	v_fill(fsmag, 1.0f, num_harm);
    avg = 0.0;

    /* Perform peak-picking on FFT of input signal */

    /* Calculate FFT of complex signal in scratch buffer	*/
    v_zap(find_hbuf,2*FFTLENGTH);
    for (i = 0; i < 2*length; i+=2)
		find_hbuf[i] = input[i/2];
    fft(find_hbuf,FFTLENGTH,-1);
	
    /* Calculate magnitude squared of coefficients		*/
	arm_cmplx_mag_squared_f32(find_hbuf, mag, FFTLENGTH);
	
    /* Implement pitch dependent staircase function		*/
    fwidth = FFTLENGTH / pitch;	/* Harmonic bin width	*/
    iwidth = (int) fwidth;
    if (iwidth < 2) iwidth = 2;
    i2 = iwidth/2;
    avg = 0.0;
    if (num_harm > 0.25f*pitch)	num_harm = (int)(0.25f*pitch);
    for (k = 0; k < num_harm; k++) {
		i = (int)(((k+1)*fwidth) - i2 + 0.5f); /* Start at peak-i2 */
		j = i + findmax(&mag[i],iwidth);
		fsmag[k] = mag[j];
		avg += mag[j];
    }

    /* Normalize Fourier series values to average magnitude */
    temp = num_harm/(avg + .0001f);
    for (i = 0; i < num_harm; i++) {
		fsmag[i] = arm_sqrt(temp*fsmag[i]);
    }
}

/*	Subroutine FFT: Fast Fourier Transform 		*/
/**************************************************************
* Replaces data by its DFT, if isign is 1, or replaces data   *
* by inverse DFT times nn if isign is -1.  data is a complex  *
* array of length nn, input as a real array of length 2*nn.   *
* nn MUST be an integer power of two.  This is not checked    *
* The real part of the number should be in the zeroeth        *
* of data , and the imaginary part should be in the next      *
* element.  Hence all the real parts should have even indices *
* and the imaginary parts, odd indices.			      *

* Data is passed in an array starting in position 0, but the  *
* code is copied from Fortran so uses an internal pointer     *
* which accesses position 0 as position 1, etc.		      *

* This code uses e+jwt sign convention, so isign should be    *
* reversed for e-jwt.                                         *
***************************************************************/
void fft(float *datam1,int nn,int isign)
{
	arm_cfft_f32(&arm_cfft_sR_f32_len512, datam1, 0, 1);
}

/*								*/
/*	Subroutine IDFT_REAL: take inverse discrete Fourier 	*/
/*	transform of real input coefficients.			*/
/*	Assume real time signal, so reduce computation		*/
/*	using symmetry between lower and upper DFT		*/
/*	coefficients.						*/
/*								*/
void	idft_real(float real[], float signal[], int length)

{
    int	i, j, k, k_inc, length2;
    float	w, accum;

    length2 = (length/2)+1;
    w = 2 * M_PI / length;
    for (i = 0; i < length; i++ ) {
		idftc[i] = arm_cos(w*i);
    }
    real[0] *= (1.0f/length);
    for (i = 1; i < length2-1; i++ ) {
		real[i] *= (2.0f/length);
    }
    if ((i*2) == length)
		real[i] *= (1.0f/length);
    else
		real[i] *= (2.0f/length);

    for (i = 0; i < length; i++ ) {
		accum = real[0];
		k_inc = i;
		k = k_inc;
		for (j = 1; j < length2; j++ ) {
			accum += real[j] * idftc[k];
			k += k_inc;
			if (k >= length)
			k -= length;
		}
		signal[i] = accum; 
    }
}
