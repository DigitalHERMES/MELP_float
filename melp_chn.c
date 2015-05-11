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
Name: melp_chn_write, melp_chn_read
Description: Write/read MELP channel bitstream
Inputs:
MELP parameter structure
Outputs: 
updated MELP parameter structure (channel pointers)
Returns: void
*/

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "melp.h"
#include "vq.h"
#include "melp_sub.h"
#include "dsp_sub.h"

/* Define number of channel bits per frame */
#define NUM_CH_BITS 54
#define ORIGINAL_BIT_ORDER 0  /* flag to use bit order of original version */

/* Define bit buffer */
static unsigned int bit_buffer[NUM_CH_BITS] CCMRAM;
static struct melp_param param_buffer;

#if (ORIGINAL_BIT_ORDER)
/* Original linear order */
static int bit_order[NUM_CH_BITS] RODATA = {
	0,  1,  2,  3,  4,  5,
	6,  7,  8,  9,  10, 11,
	12, 13, 14, 15, 16, 17, 
	18, 19, 20, 21, 22, 23,
	24, 25, 26, 27, 28, 29,
	30, 31, 32, 33, 34, 35,
	36, 37, 38, 39, 40, 41, 
	42, 43, 44, 45, 46, 47, 
	48, 49, 50, 51, 52, 53};
#else
/* Order based on priority of bits */
static int bit_order[NUM_CH_BITS] RODATA = {
	0,  17, 9,  28, 34, 3, 
	4,  39, 1,  2,  13, 38,
	14, 10, 11, 40, 15, 21,
	27, 45, 12, 26, 25, 33,
	20, 24, 23, 32, 44, 46,
	22, 31, 53, 52, 51, 7,
	6,  19, 18, 29, 37, 30,
	36, 35, 43, 42, 16, 41, 
	50, 49, 48, 47, 8,  5
};
#endif

static int sync_bit = 0; /* sync bit */


void melp_chn_write(struct melp_param *par)
{
	int i, bit_cntr;
	unsigned int *bit_ptr; 

	param_buffer.gain_index[1] = par->gain_index[1];
	param_buffer.gain_index[0] =  par->gain_index[0];
	param_buffer.pitch_index =  par->pitch_index;
	param_buffer.jit_index =  par->jit_index;
	param_buffer.bpvc_index =  par->bpvc_index;

	memcpy(param_buffer.msvq_par.indices, par->msvq_par.indices, sizeof(param_buffer.msvq_par.indices));
	memcpy(param_buffer.fsvq_par.indices, par->fsvq_par.indices, sizeof(param_buffer.fsvq_par.indices));
	return;

	/* FEC: code additional information in redundant indices */
	// fec_code(par);

	/*	Fill bit buffer	*/
	bit_ptr = bit_buffer;
	bit_cntr = 0;

	pack_code(par->gain_index[1],&bit_ptr,&bit_cntr,5,1);

	/* Toggle and write sync bit */
	sync_bit ^= 1;
	pack_code(sync_bit,&bit_ptr,&bit_cntr,1,1);
	pack_code(par->gain_index[0],&bit_ptr,&bit_cntr,3,1);
	pack_code(par->pitch_index,&bit_ptr,&bit_cntr,PIT_BITS,1);
	pack_code(par->jit_index,&bit_ptr,&bit_cntr,1,1);
	pack_code(par->bpvc_index,&bit_ptr,&bit_cntr,NUM_BANDS-1,1);

	for (i = 0; i < par->msvq_par.num_stages; i++) { 
		pack_code(par->msvq_par.indices[i],&bit_ptr,&bit_cntr,par->msvq_par.bits[i],1);
	}
	pack_code(par->fsvq_par.indices[0],&bit_ptr,&bit_cntr, FS_BITS,1);
}

int melp_chn_read(struct melp_param *par)
{
	int erase = 0;
	int i, bit_cntr;
	unsigned int *bit_ptr; 

	/*	Read information from  bit buffer	*/
	bit_ptr = bit_buffer;
	bit_cntr = 0;

	par->gain_index[1] = param_buffer.gain_index[1];
	par->gain_index[0] = param_buffer.gain_index[0];
	par->pitch_index = param_buffer.pitch_index;
	par->jit_index = param_buffer.jit_index;
	par->bpvc_index = param_buffer.bpvc_index;
	memcpy(par->msvq_par.indices, param_buffer.msvq_par.indices, sizeof(param_buffer.msvq_par));
	memcpy(par->fsvq_par.indices, param_buffer.fsvq_par.indices, sizeof(param_buffer.fsvq_par));

	/* Clear unvoiced flag */
	par->uv_flag = 0;
	return 0;

	unpack_code(&bit_ptr,&bit_cntr,&par->gain_index[1],5,1,0);

	/* Read sync bit */
	unpack_code(&bit_ptr,&bit_cntr,&i,1,1,0);
	unpack_code(&bit_ptr,&bit_cntr,&par->gain_index[0],3,1,0);
	unpack_code(&bit_ptr,&bit_cntr,&par->pitch_index,PIT_BITS,1,0);

	unpack_code(&bit_ptr,&bit_cntr,&par->jit_index,1,1,0);
	unpack_code(&bit_ptr,&bit_cntr,&par->bpvc_index, NUM_BANDS-1,1,0);

	for (i = 0; i < par->msvq_par.num_stages; i++) {
		unpack_code(&bit_ptr,&bit_cntr,&par->msvq_par.indices[i],par->msvq_par.bits[i],1,0);
	}
	unpack_code(&bit_ptr,&bit_cntr,&par->fsvq_par.indices[0], FS_BITS,1,0);

	/* Clear unvoiced flag */
	par->uv_flag = 0;
	// erase = fec_decode(par,erase);
	/* Return erase flag */
	return(erase);
}
