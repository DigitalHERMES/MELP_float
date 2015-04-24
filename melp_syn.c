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
    Name: melp_syn.c
    Description: MELP synthesis
      This program takes the new parameters for a speech
      frame and synthesizes the output speech.  It keeps
      an internal record of the previous frame parameters
      to use for interpolation.			
    Inputs:
      *par - MELP parameter structure
    Outputs: 
      speech[] - output speech signal
    Returns: void
*/

/* compiler include files */
 
#include <stdio.h>
#include <math.h>
#include "melp.h"
#include "spbstd.h"
#include "lpc.h"
#include "mat.h"
#include "vq.h"
#include "fs.h"
#include "dsp_sub.h"
#include "melp_sub.h"


/* temporary memory */
static float sigbuf[BEGIN+PITCHMAX];
static float sig2[BEGIN+PITCHMAX];
static float fs_real[PITCHMAX];

/* permanent memory */ 
static int firstcall = 1; /* Just used for noise gain init */
static float sigsave[2*PITCHMAX];
static struct melp_param prev_par;
static int syn_begin;
static float prev_scale;
static float noise_gain = MIN_NOISE;
static float pulse_del[MIX_ORD],noise_del[MIX_ORD];
static float lpc_del[LPC_ORD],ase_del[LPC_ORD],tilt_del[TILT_ORD];
static float disp_del[DISP_ORD];
static float w_fs_inv[NUM_HARM];

/* these can be saved or recomputed */
static float prev_pcof[MIX_ORD+1], prev_ncof[MIX_ORD+1];
static float prev_tilt;

// Temporary static vars
static float tilt_cof[TILT_ORD+1];
static float lsf[LPC_ORD+1];
static float lpc[LPC_ORD+1];
static float ase_num[LPC_ORD+1],ase_den[LPC_ORD+1];
static float curr_pcof[MIX_ORD+1],curr_ncof[MIX_ORD+1];
static float pulse_cof[MIX_ORD+1],noise_cof[MIX_ORD+1];
static float w_fs[NUM_HARM];

void melp_syn(struct melp_param *par, float sp_out[])
{

    int i, gaincnt;
    int erase;
    int length;
    float intfact, ifact, ifact_gain;
    float gain,pulse_gain,pitch,jitter;
    float curr_tilt;
    float temp,sig_prob;
    
    /* Copy previous period of processed speech to output array */
    if (syn_begin > 0) {
		if (syn_begin > FRAME) {
			v_equ(&sp_out[0],&sigsave[0],FRAME);
			/* past end: save remainder in sigsave[0] */
			v_equ(&sigsave[0],&sigsave[FRAME],syn_begin-FRAME);
		}
		else 
		  v_equ(&sp_out[0],&sigsave[0],syn_begin);
    }
        
    /*	Read and decode channel input buffer	*/
    erase = melp_chn_read(par);

	/* Decode new frame if no erasures occurred */
	if (erase) 
	{	/* Erasure: frame repeat */
		*par = prev_par; 
		/* Force all subframes to equal last one */
		for (i = 0; i < NUM_GAINFR-1; i++) {
			par->gain[i] = par->gain[NUM_GAINFR-1];
		}
	}else {
		/* Decode line spectrum frequencies	*/
		vq_msd2(msvq_cb,&par->lsf[1],(float*)NULL,(float*)NULL,par->msvq_par.indices,
			par->msvq_par.levels,par->msvq_par.num_stages,LPC_ORD,0);
		i = FS_LEVELS;
		if (par->uv_flag)
		{
			fill(par->fs_mag,1.0F,NUM_HARM);
		}else
		{	
			/* Decode Fourier magnitudes */
			vq_msd2(fsvq_cb,par->fs_mag,(float*)NULL,(float*)NULL,
				par->fsvq_par.indices,&i,1,NUM_HARM,0);
		}

		/* Decode gain terms with uniform log quantizer	*/
		q_gain_dec(par->gain, par->gain_index,GN_QLO,GN_QUP,GN_QLEV);

		/* Fractional pitch: */
		/* Decode logarithmic pitch period */
		if (par->uv_flag)
		{
			par->pitch = UV_PITCH;
		}else 
		{
			quant_u_dec(par->pitch_index,&par->pitch,PIT_QLO,PIT_QUP,
				PIT_QLEV);
			par->pitch = powf(10.0f,par->pitch);
		}

		/* Decode jitter and bandpass voicing */
		quant_u_dec(par->jit_index,&par->jitter,0.0,MAX_JITTER,2);
		q_bpvc_dec(&par->bpvc[0],&par->bpvc_index,par->uv_flag,
			NUM_BANDS);
	}

    if (par->uv_flag != 1 && !erase) { 
		/* Un-weight Fourier magnitudes */
		window(&par->fs_mag[0],w_fs_inv,&par->fs_mag[0],NUM_HARM);
    }

    /* Update adaptive noise level estimate based on last gain	*/
    if (firstcall) {
		firstcall = 0;
		noise_gain = par->gain[NUM_GAINFR-1];
    }else if (!erase) 
	{
		for (i = 0; i < NUM_GAINFR; i++) {
			noise_est(par->gain[i],&noise_gain,UPCONST,DOWNCONST,
				  MIN_NOISE,MAX_NOISE);
			/* Adjust gain based on noise level (noise suppression) */
			noise_sup(&par->gain[i],noise_gain,MAX_NS_SUP,MAX_NS_ATT,NFACT);
		}   
    }
    
    /* Clamp LSP bandwidths to avoid sharp LPC filters */
    lpc_clamp(par->lsf,BWMIN,LPC_ORD);
    
    /* Calculate spectral tilt for current frame for spectral enhancement */
    tilt_cof[0] = 1.0;
    lpc_lsp2pred(par->lsf,lpc,LPC_ORD);
    lpc_pred2refl(lpc,sig2,LPC_ORD);
    if (sig2[1] < 0.0f)
      curr_tilt = 0.5f*sig2[1];
    else
      curr_tilt = 0.0f;
    
    /* Disable pitch interpolation for high-pitched onsets */
    if (par->pitch < 0.5f * prev_par.pitch && 
			par->gain[0] > 6.0f + prev_par.gain[NUM_GAINFR-1]) 
	{ /* copy current pitch into previous */
		prev_par.pitch = par->pitch;
    }
    
    /* Set pulse and noise coefficients based on voicing strengths */
    v_zap(curr_pcof, MIX_ORD + 1);
    v_zap(curr_ncof, MIX_ORD + 1);
    for (i = 0; i < NUM_BANDS; i++) 
	{
		if (par->bpvc[i] > 0.5f)
			v_add(curr_pcof,&bp_cof[i][0],MIX_ORD+1);
		else
			v_add(curr_ncof,&bp_cof[i][0],MIX_ORD+1);
    }
	
    /* Process each pitch period */
    while (syn_begin < FRAME) 
	{
		/* interpolate previous and current parameters */
		ifact = ((float) syn_begin ) / FRAME;
				
		if (syn_begin >= GAINFR) {
			gaincnt = 2;
			ifact_gain = ((float) syn_begin-GAINFR) / GAINFR;
		}else {
			gaincnt = 1;
			ifact_gain = ((float) syn_begin) / GAINFR;
		}
		
		/* interpolate gain */
		if (gaincnt > 1) {
			gain = ifact_gain*par->gain[gaincnt-1] + 
			(1.0f-ifact_gain)*par->gain[gaincnt-2];
		}else {
			gain = ifact_gain*par->gain[gaincnt-1] + 
			(1.0f-ifact_gain)*prev_par.gain[NUM_GAINFR-1];
		}
		
		/* Set overall interpolation path based on gain change */
		temp = par->gain[NUM_GAINFR-1] - prev_par.gain[NUM_GAINFR-1];
		if (fabs(temp) > 6) {
			/* Power surge: use gain adjusted interpolation */
			intfact = (gain - prev_par.gain[NUM_GAINFR-1]) / temp;
			if (intfact > 1.0f)	intfact = 1.0f;
			if (intfact < 0.0f)	intfact = 0.0f;
		}else {    /* Otherwise, linear interpolation */
			intfact = ((float) syn_begin) / FRAME;
		}

		/* interpolate LSF's and convert to LPC filter */
		interp_array(prev_par.lsf,par->lsf,lsf,intfact,LPC_ORD+1);
		lpc_lsp2pred(lsf,lpc,LPC_ORD);
		
		/* Check signal probability for adaptive spectral enhancement filter */
		sig_prob = lin_int_bnd(gain,noise_gain+12.0f,noise_gain+30.0f,
					   0.0f,1.0f);

		/* Calculate adaptive spectral enhancement filter coefficients */
		ase_num[0] = 1.0f;
		lpc_bw_expand(lpc,ase_num,sig_prob*ASE_NUM_BW,LPC_ORD);
		lpc_bw_expand(lpc,ase_den,sig_prob*ASE_DEN_BW,LPC_ORD);
		tilt_cof[1] = sig_prob*(intfact*curr_tilt + 
					(1.0f-intfact)*prev_tilt);
		
		/* interpolate pitch and pulse gain */
		pitch = intfact*par->pitch + (1.0f-intfact)*prev_par.pitch;
		pulse_gain = SYN_GAIN*sqrtf(pitch);
		
		/* interpolate pulse and noise coefficients */
		temp = sqrtf(ifact);
		interp_array(prev_pcof,curr_pcof,pulse_cof,temp,MIX_ORD+1);
		interp_array(prev_ncof,curr_ncof,noise_cof,temp,MIX_ORD+1);
		
		/* interpolate jitter */
		jitter = ifact*par->jitter + (1.0f-ifact)*prev_par.jitter;
		
		/* convert gain to linear */
		gain = powf(10.0f,0.05f*gain);
		
		/* Set period length based on pitch and jitter */
		rand_num(&temp,1.0f,1);
		length = (int) (pitch * (1.0f-jitter*temp) + 0.5f);
		if (length < PITCHMIN)  length = PITCHMIN;
		if (length > PITCHMAX)  length = PITCHMAX;
		
		/* Use inverse DFT for pulse excitation */
		fill(fs_real,1.0f,length);
		fs_real[0] = 0.0f;
		interp_array(prev_par.fs_mag,par->fs_mag,&fs_real[1],intfact, NUM_HARM);
		idft_real(fs_real,&sigbuf[BEGIN],length);
		
		/* Delay overall signal by PDEL samples (circular shift) */
		/* use fs_real as a scratch buffer */
		v_equ(fs_real,&sigbuf[BEGIN],length);
		v_equ(&sigbuf[BEGIN+PDEL],fs_real,length-PDEL);
		v_equ(&sigbuf[BEGIN],&fs_real[length-PDEL],PDEL);
		
		/* Scale by pulse gain */
		v_scale(&sigbuf[BEGIN],pulse_gain,length);	
		
		/* Filter and scale pulse excitation */
		v_equ(&sigbuf[BEGIN-MIX_ORD],pulse_del,MIX_ORD);
		v_equ(pulse_del,&sigbuf[length+BEGIN-MIX_ORD],MIX_ORD);
		zerflt(&sigbuf[BEGIN],pulse_cof,&sigbuf[BEGIN],MIX_ORD,length);
		
		/* Get scaled noise excitation */
		rand_num(&sig2[BEGIN],(1.732f*SYN_GAIN),length);
		
		/* Filter noise excitation */
		v_equ(&sig2[BEGIN-MIX_ORD],noise_del,MIX_ORD);
		v_equ(noise_del,&sig2[length+BEGIN-MIX_ORD],MIX_ORD);
		zerflt(&sig2[BEGIN],noise_cof,&sig2[BEGIN],MIX_ORD,length);
		
		/* Add two excitation signals (mixed excitation) */
		v_add(&sigbuf[BEGIN],&sig2[BEGIN],length);
		
		/* Adaptive spectral enhancement */
		v_equ(&sigbuf[BEGIN-LPC_ORD],ase_del,LPC_ORD);
		lpc_synthesis(&sigbuf[BEGIN],&sigbuf[BEGIN],ase_den,LPC_ORD,length);
		v_equ(ase_del,&sigbuf[BEGIN+length-LPC_ORD],LPC_ORD);
		zerflt(&sigbuf[BEGIN],ase_num,&sigbuf[BEGIN],LPC_ORD,length);
		v_equ(&sigbuf[BEGIN-TILT_ORD],tilt_del,TILT_ORD);
		v_equ(tilt_del,&sigbuf[length+BEGIN-TILT_ORD],TILT_ORD);
		zerflt(&sigbuf[BEGIN],tilt_cof,&sigbuf[BEGIN],TILT_ORD,length);
		
		/* Perform LPC synthesis filtering */
		v_equ(&sigbuf[BEGIN-LPC_ORD],lpc_del,LPC_ORD);
		lpc_synthesis(&sigbuf[BEGIN],&sigbuf[BEGIN],lpc,LPC_ORD,length);
		v_equ(lpc_del,&sigbuf[length+BEGIN-LPC_ORD],LPC_ORD);
			
		/* Adjust scaling of synthetic speech */
		scale_adj(&sigbuf[BEGIN],gain,&prev_scale,length,SCALEOVER);

		/* Implement pulse dispersion filter */
		v_equ(&sigbuf[BEGIN-DISP_ORD],disp_del,DISP_ORD);
		v_equ(disp_del,&sigbuf[length+BEGIN-DISP_ORD],DISP_ORD);
		zerflt(&sigbuf[BEGIN],disp_cof,&sigbuf[BEGIN],DISP_ORD,length);
			
		/* Copy processed speech to output array (not past frame end) */
		if (syn_begin+length > FRAME) {
			v_equ(&sp_out[syn_begin],&sigbuf[BEGIN],FRAME-syn_begin);
					
			/* past end: save remainder in sigsave[0] */
			v_equ(&sigsave[0],&sigbuf[BEGIN+FRAME-syn_begin],
			  length-(FRAME-syn_begin));
		}else {
			v_equ(&sp_out[syn_begin],&sigbuf[BEGIN],length);
		}
		
		/* Update syn_begin for next period */
		syn_begin += length;
    }
		
    /* Save previous pulse and noise filters for next frame */
    v_equ(prev_pcof,curr_pcof,MIX_ORD+1);
    v_equ(prev_ncof,curr_ncof,MIX_ORD+1);
    
    /* Copy current parameters to previous parameters for next time */
    prev_par = *par;
    prev_tilt = curr_tilt;
    
    /* Update syn_begin for next frame */
    syn_begin -= FRAME;
}


/* 
 *
 * Subroutine melp_syn_init(): perform initialization for melp 
 *	synthesis
 *
 */

void melp_syn_init(melp_param_t *par)
{
    int i;
	
    v_zap(prev_par.gain,NUM_GAINFR);
    prev_par.pitch = UV_PITCH;
    prev_par.lsf[0] = 0.0f;
    for (i = 1; i < LPC_ORD+1; i++)
      prev_par.lsf[i] = prev_par.lsf[i-1] + (1.0f/(LPC_ORD+1));
    prev_par.jitter = 0.0f;
    v_zap(&prev_par.bpvc[0],NUM_BANDS);
    prev_tilt=0.0f;
    prev_scale = 0.0f;
    syn_begin = 0;
    noise_gain = MIN_NOISE;
    firstcall = 1;
    v_zap(pulse_del,MIX_ORD);
    v_zap(noise_del,MIX_ORD);
    v_zap(lpc_del,LPC_ORD);
    v_zap(ase_del,LPC_ORD);
    v_zap(tilt_del,TILT_ORD);
    v_zap(disp_del,DISP_ORD);
    v_zap(sig2,BEGIN+PITCHMAX);
    v_zap(sigbuf,BEGIN+PITCHMAX);
    v_zap(sigsave,PITCHMAX);
    v_zap(prev_pcof,MIX_ORD+1);
    v_zap(prev_ncof,MIX_ORD+1);
    prev_ncof[MIX_ORD/2] = 1.0;
	
    fill(prev_par.fs_mag,1.0,NUM_HARM);

    /* 
     * Initialize multi-stage vector quantization (read codebook) 
     */
	par->msvq_par.num_best = MSVQ_M;
    par->msvq_par.num_stages = 4;
    par->msvq_par.dimension = 10;

    par->msvq_par.levels[0] = 128;
    par->msvq_par.levels[1] = 64;
    par->msvq_par.levels[2] = 64;
    par->msvq_par.levels[3] = 64;
	
    par->msvq_par.bits[0] = 7;
    par->msvq_par.bits[1] = 6;
    par->msvq_par.bits[2] = 6;
    par->msvq_par.bits[3] = 6;
	
    par->msvq_par.cb = msvq_cb;
	
    /* Scale codebook to 0 to 1 */
    if (fsvq_weighted == 0)
      v_scale(par->msvq_par.cb,(2.0f/FSAMP),3200);

    /* 
     * Initialize Fourier magnitude vector quantization (read codebook) 
     */
	 
    par->fsvq_par.num_best = 1;
    par->fsvq_par.num_stages = 1;
    par->fsvq_par.dimension = NUM_HARM;

    /* 
     * Allocate memory for number of levels per stage and indices
     * and for number of bits per stage 
     */
	
    par->fsvq_par.levels[0] = FS_LEVELS;
    par->fsvq_par.bits[0] = FS_BITS;
    par->fsvq_par.cb = fsvq_cb;
	
    /* 
     * Initialize fixed MSE weighting and inverse of weighting 
     */
	
    vq_fsw(w_fs, NUM_HARM, 60.0f);
    for (i = 0; i < NUM_HARM; i++)
      w_fs_inv[i] = 1.0f/w_fs[i];

    /* 
     * Pre-weight codebook (assume single stage only) 
     */	
    if (fsvq_weighted == 0)
	{
		fsvq_weighted = 1;
		for (i = 0; i < par->fsvq_par.levels[0]; i++)
		window(&par->fsvq_par.cb[i*NUM_HARM],w_fs,&par->fsvq_par.cb[i*NUM_HARM], NUM_HARM);
	}	
}










