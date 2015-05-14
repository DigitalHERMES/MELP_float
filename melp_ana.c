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
    Name: melp_ana.c
    Description: MELP analysis
    Inputs:
      speech[] - input speech signal
    Outputs: 
      *par - MELP parameter structure
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
#include "pit.h"
#include "melp_sub.h"
#include "dsp_sub.h"

/* memory definitions */
static float sigbuf[SIG_LENGTH]		CCMRAM;
static float speech[IN_BEG+FRAME]	CCMRAM;
static float dcdel[DC_ORD]			CCMRAM;
static float lpfsp_del[LPF_ORD]		CCMRAM;
static float pitch_avg;
static float fpitch[2]				CCMRAM;

static float w_fs[NUM_HARM]			CCMRAM;
static float r[LPC_ORD+1]			CCMRAM, 
			 lpc[LPC_ORD+1]			CCMRAM;
static float weights[LPC_ORD]		CCMRAM;
	
void melp_ana(float sp_in[],struct melp_param *par)
{
    int i;
    float sub_pitch;
    float temp,pcorr,bpthresh;

        
    /* Remove DC from input speech */
    dc_rmv(sp_in,&speech[IN_BEG],dcdel,FRAME);
    
    /* Copy input speech to pitch window and lowpass filter */
    v_equ(&sigbuf[LPF_ORD],&speech[PITCH_BEG],PITCH_FR);
//    v_equ(sigbuf,lpfsp_del,LPF_ORD);
    iirflt(&sigbuf[LPF_ORD],lpf_den,&sigbuf[LPF_ORD], lpfsp_del, LPF_ORD,PITCH_FR);
//    v_equ(lpfsp_del,&sigbuf[FRAME],LPF_ORD);
    zerflt(&sigbuf[LPF_ORD],lpf_num,&sigbuf[LPF_ORD],LPF_ORD,PITCH_FR);
    
    /* Perform global pitch search at frame end on lowpass speech signal */
    /* Note: avoid short pitches due to formant tracking */
    fpitch[1] = find_pitch(&sigbuf[LPF_ORD+(PITCH_FR/2)],&temp,(2*PITCHMIN),PITCHMAX,PITCHMAX);
    
    /* Perform bandpass voicing analysis for end of frame */
    bpvc_ana(&speech[FRAME_END], fpitch, &par->bpvc[0], &sub_pitch);
    
    /* Force jitter if lowest band voicing strength is weak */    
    if (par->bpvc[0] < VJIT)
		par->jitter = MAX_JITTER;
    else
		par->jitter = 0.0;
    
    /* Calculate LPC for end of frame */
    window(&speech[(FRAME_END-(LPC_FRAME/2))],win_cof,sigbuf,LPC_FRAME);
    autocorr(sigbuf,r,LPC_ORD,LPC_FRAME);
    lpc[0] = 1.0;
    lpc_schur(r,lpc,LPC_ORD);
    lpc_bw_expand(lpc,lpc,BWFACT,LPC_ORD);
    
    /* Calculate LPC residual */
    zerflt(&speech[PITCH_BEG],lpc,&sigbuf[LPF_ORD],LPC_ORD,PITCH_FR);
        
    /* Check peakiness of residual signal */
    temp = peakiness(&sigbuf[(LPF_ORD+(PITCHMAX/2))],PITCHMAX);
    
    /* Peakiness: force lowest band to be voiced  */
    if (temp > PEAK_THRESH) {
		par->bpvc[0] = 1.0;
    }
    
    /* Extreme peakiness: force second and third bands to be voiced */
    if (temp > PEAK_THR2) {
		par->bpvc[1] = 1.0;
		par->bpvc[2] = 1.0;
    }
		
    /* Calculate overall frame pitch using lowpass filtered residual */
    par->pitch = pitch_ana(&speech[FRAME_END], &sigbuf[LPF_ORD+PITCHMAX], sub_pitch,pitch_avg,&pcorr);
    bpthresh = BPTHRESH;
    
    /* Calculate gain of input speech for each gain subframe */
    for (i = 0; i < NUM_GAINFR; i++) {
		if (par->bpvc[0] > bpthresh) {
			/* voiced mode: pitch synchronous window length */
			temp = sub_pitch;
			par->gain[i] = gain_ana(&speech[FRAME_BEG+(i+1)*GAINFR], temp,MIN_GAINFR,2*PITCHMAX);
		}else {
			temp = 1.33f*GAINFR - 0.5f;
			par->gain[i] = gain_ana(&speech[FRAME_BEG+(i+1)*GAINFR], temp,0,2*PITCHMAX);
		}
    }
    
    /* Update average pitch value */
    if (par->gain[NUM_GAINFR-1] > SILENCE_DB)
      temp = pcorr;
    else
      temp = 0.0;
    pitch_avg = p_avg_update(par->pitch, temp, VMIN);
    
    /* Calculate Line Spectral Frequencies */
    lpc_pred2lsp(lpc,par->lsf,LPC_ORD);
    
    /* Force minimum LSF bandwidth (separation) */
    lpc_clamp(par->lsf,BWMIN,LPC_ORD);
    
    /* Quantize MELP parameters to 2400 bps and generate bitstream */
    
    /* Quantize LSF's with MSVQ */
    vq_lspw(weights, &par->lsf[1], lpc, LPC_ORD);
    msvq_enc(&par->lsf[1], weights, &par->lsf[1], par->msvq_par);
    
    /* Force minimum LSF bandwidth (separation) */
    lpc_clamp(par->lsf,BWMIN,LPC_ORD);
    
    /* Quantize logarithmic pitch period */
    /* Reserve all zero code for completely unvoiced */
    par->pitch = log10f(par->pitch);
    quant_u(&par->pitch,&par->pitch_index,PIT_QLO,PIT_QUP,PIT_QLEV);
    par->pitch = powf(10.0f,par->pitch);
    
    /* Quantize gain terms with uniform log quantizer	*/
    q_gain(par->gain, par->gain_index,GN_QLO,GN_QUP,GN_QLEV);
    
    /* Quantize jitter and bandpass voicing */
    quant_u(&par->jitter,&par->jit_index,0.0,MAX_JITTER,2);
    par->uv_flag = q_bpvc(&par->bpvc[0],&par->bpvc_index,bpthresh,
			  NUM_BANDS);
    
    /*	Calculate Fourier coefficients of residual signal from quantized LPC */
    v_fill(par->fs_mag,1.0,NUM_HARM);
    if (par->bpvc[0] > bpthresh) {
		lpc_lsp2pred(par->lsf,lpc,LPC_ORD);
		zerflt(&speech[(FRAME_END-(LPC_FRAME/2))],lpc,sigbuf,LPC_ORD,LPC_FRAME);
		window(sigbuf,win_cof,sigbuf,LPC_FRAME);
		find_harm(sigbuf, par->fs_mag, par->pitch, NUM_HARM, LPC_FRAME);
    }
    
    /* quantize Fourier coefficients */
    /* pre-weight vector, then use Euclidean distance */
    window(&par->fs_mag[0],w_fs,&par->fs_mag[0],NUM_HARM);
    fsvq_enc(&par->fs_mag[0], &par->fs_mag[0], par->fsvq_par);
    

    /* Write channel bitstream */
    melp_chn_write(par);

    /* Update delay buffers for next frame */
    v_equ(&speech[0],&speech[FRAME],IN_BEG);
    fpitch[0] = fpitch[1];
}



/* 
 * melp_ana_init: perform initialization 
 */
void melp_ana_init(melp_param_t *par)
{
    int j;

    bpvc_ana_init();
    pitch_ana_init();
    p_avg_init();

    v_zap(speech,IN_BEG+FRAME);
    pitch_avg=DEFAULT_PITCH_;
    v_fill(fpitch,DEFAULT_PITCH_,2);
    v_zap(lpfsp_del,LPF_ORD);
	
    /* Initialize multi-stage vector quantization (read codebook) */
	
    par->msvq_par.num_best = MSVQ_M;
    par->msvq_par.num_stages = 4;
    par->msvq_par.num_dimensions = 10;
	
    par->msvq_par.levels[0] = 128;
    par->msvq_par.levels[1] = 64;
    par->msvq_par.levels[2] = 64;
    par->msvq_par.levels[3] = 64;
	
    par->msvq_par.bits[0] = 7;
    par->msvq_par.bits[1] = 6;
    par->msvq_par.bits[2] = 6;
    par->msvq_par.bits[3] = 6;
	
    par->msvq_par.cb = msvq_cb;
	
    /* Initialize Fourier magnitude vector quantization (read codebook) */
	
    par->fsvq_par.num_best = 1;
    par->fsvq_par.num_stages = 1;
    par->fsvq_par.num_dimensions = NUM_HARM;

    par->fsvq_par.levels[0] = FS_LEVELS;
    par->fsvq_par.bits[0] = FS_BITS;
    par->fsvq_par.cb = fsvq_cb;
	
    /* Initialize fixed MSE weighting  */
    vq_fsw(w_fs, NUM_HARM, 60.0f);
	
    /* Pre-weight codebook (assume single stage only) */	
    if (fsvq_weighted == 0)
	{
		fsvq_weighted = 1;
	    /* Scale codebook to 0 to 1 */
		v_scale(msvq_cb,(2.0f/FSAMP),3200);
		for (j = 0; j < FS_LEVELS; j++)
			window(&fsvq_cb[j*NUM_HARM],w_fs,&fsvq_cb[j*NUM_HARM], NUM_HARM);
	}
}
