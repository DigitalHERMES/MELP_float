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

/*                                                                  */
/*  melp.h: include file for MELP coder                             */
/*                                                                  */

/*  compiler flags */
#define ANA_SYN			0
#define ANALYSIS		1
#define SYNTHESIS		2

#define RATE2400		2400
#define RATE1200		1200


/*  compiler constants */
#define FRAME 180           /* speech frame size */
#define LPC_ORD 10          /* LPC order */
#define NUM_HARM 10         /* number of Fourier magnitudes */
#define NUM_GAINFR 2        /* number of gains per frame */
#define LPC_FRAME 200       /* LPC window size */
#define PITCHMIN 20         /* minimum pitch lag */
#define PITCHMAX 160        /* maximum pitch lag */
#define NUM_BANDS 5         /* number of frequency bands */
#define LPF_ORD 6           /* lowpass filter order */
#define DC_ORD 4            /* DC removal filter order */
#define BPF_ORD 6           /* bandpass analysis filter order */
#define ENV_ORD 2           /* bandpass envelope filter order */
#define MIX_ORD 32          /* mixed excitation filtering order */
#define DISP_ORD 64         /* pulse dispersion filter order */
#define DEFAULT_PITCH_ 50.0f /* default pitch value */
#define UV_PITCH 50         /* unvoiced pitch value */
#define VMIN 0.8F            /* minimum strongly voiced correlation */
#define VJIT 0.5F            /* jitter threshold for correlations */
#define BPTHRESH 0.6F        /* bandpass voicing threshold */
#define MAX_JITTER 0.25F     /* maximum jitter percentage (as a fraction) */
#define ASE_NUM_BW 0.5F      /* adaptive spectral enhancement - numerator */
#define ASE_DEN_BW 0.8F      /* adaptive spectral enhancement - denominator */
#define GAINFR (FRAME/NUM_GAINFR)  /* size of gain frame */
#define MIN_GAINFR 120      /* minimum gain analysis window */
#define MINLENGTH 160       /* minimum correlation length */
#define FSAMP 8000.0F        /* sampling frequency */
#define MSVQ_M 8            /* M-best for MSVQ search */
#define MSVQ_MAXCNT 256     /* maximum inner loop counter for MSVQ search */
#define BWMIN (50.0F*2/FSAMP) /* minimum LSF separation */

/* Noise suppression/estimation parameters */
/* Up by 3 dB/sec (0.5*22.5 ms frame), down by 12 dB/sec */
#define UPCONST 0.0337435f    /* noise estimation up time constant */
#define DOWNCONST -0.135418f  /* noise estimation down time constant */
#define NFACT 3.0f            /* noise floor boost in dB */
#define MAX_NS_ATT 6.0f       /* maximum noise suppression */
#define MAX_NS_SUP 20.0f      /* maximum noise level to use in suppression */
#define MIN_NOISE 10.0f       /* minimum value allowed in noise estimation */
#define MAX_NOISE 80.0f       /* maximum value allowed in noise estimation */

/* Channel I/O constants */
#define	CHWORDSIZE 6         /* number of bits per channel word */
#define ERASE_MASK 0x4000    /* erasure flag mask for channel word */

#define GN_QLO 10.0f          /* minimum gain in dB */
#define GN_QUP 77.0f          /* maximum gain in dB */
#define GN_QLEV 32           /* number of second gain quantization levels */
#define PIT_BITS 7           /* number of bits for pitch coding */
#define PIT_QLEV 99          /* number of pitch levels */
#define PIT_QLO 1.30103f      /* minimum log pitch for quantization */
#define PIT_QUP 2.20412f      /* maximum log pitch for quantization */
#define FS_BITS 8            /* number of bits for Fourier magnitudes */
#define FS_LEVELS (1<<FS_BITS) /* number of levels for Fourier magnitudes */

/* compiler constants */

#define BWFACT 0.994f
#define PDECAY 0.95f
#define PEAK_THRESH 1.34f
#define PEAK_THR2 1.6f
#define SILENCE_DB 30.0f

#define FRAME_BEG (PITCHMAX -(FRAME/2))
#define FRAME_END (PITCHMAX + (FRAME/2))
#define PITCH_FR ((2*PITCHMAX)+1)
#define IN_BEG (PITCH_FR-(FRAME/2))

#define NUM_GOOD 3
#define NUM_PITCHES 2

#define NUM_MULT 8
#define SHORT_PITCH 30
#define MAXFRAC 2.0f
#define MINFRAC -1.0f

/* External variables */
extern const float lpf_num[], lpf_den[];
extern const float bpf_num[], bpf_den[];
extern const float win_cof[];
extern float msvq_cb[];
extern float fsvq_cb[];
extern const float bp_cof[NUM_BANDS][MIX_ORD+1];
extern const float disp_cof[DISP_ORD+1];
extern int fsvq_weighted;

/* compiler constants */

#if (MIX_ORD > DISP_ORD)
#define BEGIN MIX_ORD
#else
#define BEGIN DISP_ORD
#endif

#define TILT_ORD 1
#define SYN_GAIN 1000.0f
#define	SCALEOVER	10
#define PDEL SCALEOVER

/* Compiler constants */
#define UVMAX 0.55f
#define PDOUBLE1 0.75f
#define PDOUBLE2 0.5f
#define PDOUBLE3 0.9f
#define PDOUBLE4 0.7f
#define LONG_PITCH 100.0f


/* Structure definition */
typedef struct msvq_param {         /* Multistage VQ parameters */
	int num_stages;
	int num_dimensions;
	int num_best;
	int bits[4];
	int levels[4];
	int indices[4];
	float *cb;
}msvq_param_t;

typedef struct melp_param {         /* MELP parameters */
    float pitch;
    float lsf[LPC_ORD+1];
    float gain[NUM_GAINFR];
    float jitter;
    float bpvc[NUM_BANDS];
    int pitch_index;
    int lsf_index[LPC_ORD];
    int jit_index;
    int bpvc_index;
    int gain_index[NUM_GAINFR];
    int uv_flag;
    float fs_mag[NUM_HARM];
	msvq_param_t msvq_par;		/* MS VQ parameters */
	msvq_param_t fsvq_par;		/* Fourier series VQ parameters */
}melp_param_t;

#ifdef _MSC_VER
#define CCMRAM
#define RODATA
#else
#define CCMRAM __attribute__((section (".ccmram")))
#define RODATA __attribute__((section (".rodata")))
#endif

/* External function definitions */
#ifdef _MSC_VER
__declspec(dllexport) void __cdecl melp_ana(float sp_in[],struct melp_param *par);
__declspec(dllexport) void __cdecl melp_syn(struct melp_param *par, float sp_out[]);
__declspec(dllexport) void __cdecl melp_ana_init(struct melp_param *par);
__declspec(dllexport) void __cdecl melp_syn_init(struct melp_param *par);
__declspec(dllexport) int  __cdecl melp_chn_read(struct melp_param *par);
__declspec(dllexport) void __cdecl melp_chn_write(struct melp_param *par);

__declspec(dllexport) void __cdecl fec_code(struct melp_param *par);
__declspec(dllexport) int  __cdecl fec_decode(struct melp_param *par, int erase);
#else
void  melp_ana(float sp_in[],struct melp_param *par);
void  melp_syn(struct melp_param *par, float sp_out[]);
void  melp_ana_init(struct melp_param *par);
void  melp_syn_init(struct melp_param *par);
int   melp_chn_read(struct melp_param *par);
void  melp_chn_write(struct melp_param *par);

void  fec_code(struct melp_param *par);
int   fec_decode(struct melp_param *par, int erase);
#endif
