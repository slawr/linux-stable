/*
 * include/video/rcarfb.h
 *
 * Copyright (C) 2011-2012 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#ifndef __ASM_RCARFB_H__
#define __ASM_RCARFB_H__

#include <linux/fb.h>

struct rcar_reso_info {
	int num_modes;
	const struct fb_videomode *modes;
};

#define DISP_PLANE1	0
#define DISP_PLANE2	1
#define DISP_PLANE3	2
#define DISP_PLANE4	3
#define DISP_PLANE5	4
#define DISP_PLANE6	5
#define DISP_PLANE7	6
#define DISP_PLANE8	7
#define DISP0_PLANE_NUM	DISP_PLANE1
#define DISP1_PLANE_NUM	DISP_PLANE2

#define RCARFB_SUPPORTBLANK	0

#define BITPARPIXEL_8	8
#define BITPARPIXEL_16	16
#define BITPARPIXEL_24	24
#define BITPARPIXEL_32	32
#define DEFAULT_BITPARPIXEL	BITPARPIXEL_16
#define BYTEPIXEL	8

#define	RCAR_PIXFMT_ARGB8888	0
#define	RCAR_PIXFMT_ARGB0565	1
#define	RCAR_PIXFMT_ARGB4444	2 /* Not Support */
#define	RCAR_PIXFMT_ARGB1555	3
#define	RCAR_PIXFMT_RGB888	4
#define	RCAR_PIXFMT_RGB666	5
#define	RCAR_PIXFMT_UYVY	6
#define	RCAR_PIXFMT_YUYV	7

#define DEFAULT_XRES	640
#define DEFAULT_YRES	480

#define MAX_XRES	1280
#define MAX_YRES	800

#define FB_BYTES	4

#define DOT_CLK_DCLKIN	0x00000000
#define DOT_CLK_CLKS	(1 << 20)
#define DOT_CLK_MASK	DOT_CLK_CLKS

#define DU_START	1
#define DU_STOP		0

#define MODE_CHG_ON	1
#define MODE_CHG_OFF	0

#define DISPLAY_AREA0	0
#define DISPLAY_AREA1	1

struct RCARFB_PIXFMT {
	unsigned long pixfmt;
};

#define FBIOCS_PIXFMT	_IOW(0xFA, 0x00, struct RCARFB_PIXFMT)
#define FBIOCG_PIXFMT	_IOR(0xFA, 0x01, struct RCARFB_PIXFMT)

/***** R-Car Display Unit Register Definitons *****/
/* Display control register offset definitions */
#define DSYSR		0x00000	/* display 1 */
#define ILTS_DSYSR	(1<<29)
#define DSEC_DSYSR	(1<<20)
#define IUPD_DSYSR	(1<<16)
#define DRES_DSYSR	(1<<9)
#define DEN_DSYSR	(1<<8)
#define MASTER_DSYSR	(0<<6)
#define MASTER_DSYSR_MASK	(3<<6)
#define SYNC_CHG_DSYSR	(1<<6)
#define TV_SYNC_DSYSR	(2<<6)
#define N_INTR_DSYSR	(0<<4)
#define INTR_SYNC_DSYSR	(2<<4)
#define INTR_VIDEO_DSYSR	(3<<4)

#define DSMR		0x00004

#define DSSR		0x00008
#define TVR_DSSR	(1<<15)
#define VC1FB_DSA0_DSSR	(0<<30)
#define VC1FB_DSA1_DSSR	(1<<30)
#define VC1FB_DSA2_DSSR	(2<<30)
#define VC1FB_INIT_DSSR	(3<<30)
#define VC0FB_DSA0_DSSR	(0<<28)
#define VC0FB_DSA1_DSSR	(1<<28)
#define VC0FB_DSA2_DSSR	(2<<28)
#define VC0FB_INIT_DSSR	(3<<28)
#define DFB10_DSSR	(1<<25)
#define DFB9_DSSR	(1<<24)
#define DFB8_DSSR	(1<<23)
#define DFB7_DSSR	(1<<22)
#define DFB6_DSSR	(1<<21)
#define DFB5_DSSR	(1<<20)
#define DFB4_DSSR	(1<<19)
#define DFB3_DSSR	(1<<18)
#define DFB2_DSSR	(1<<17)
#define DFB1_DSSR	(1<<16)
#define TVR_DSSR	(1<<15)
#define FRM_DSSR	(1<<14)
#define VBK_DSSR	(1<<11)
#define RINT_DSSR	(1<<9)
#define HBK_DSSR	(1<<8)
#define DU1_INT_ALL	(TVR_DSSR | FRM_DSSR | VBK_DSSR | RINT_DSSR | HBK_DSSR)

#define DSRCR		0x0000c
#define DIER		0x00010
#define CPCR		0x00014

#define DPPR		0x00018
#define DPE		(1<<3)	/* Display plane enable */
#define DPS1		(0)	/* Display plane select */
#define DPS2		(1)
#define DPS3		(2)
#define DPS4		(3)
#define DPS5		(4)
#define DPS6		(5)
#define DPS7		(6)
#define DPS8		(7)
#define SHIFT_PRI8	(28)
#define SHIFT_PRI7	(24)
#define SHIFT_PRI6	(20)
#define SHIFT_PRI5	(16)
#define SHIFT_PRI4	(12)
#define SHIFT_PRI3	(8)
#define SHIFT_PRI2	(4)
#define SHIFT_PRI1	(0)
#define ALL_DPE		(DPE<<28 | DPE<<24 | DPE<<20 | DPE<<16 | DPE<<12 | \
			DPE<<8 | DPE<<4 | DPE<<0)
#define BPP16_DPPR	((DPE | DPS1) << SHIFT_PRI8)		/* plane1 */
#define BPP32_P1	((DPE | DPS1) << SHIFT_PRI7)
#define BPP32_P2	((DPE | DPS2) << SHIFT_PRI8)
#define BPP32_DPPR	(BPP32_P1 | BPP32_P2)			/* plane1 & 2 */

#define DPS1_MASK	(0x7 << SHIFT_PRI1)
#define DPS2_MASK	(0x7 << SHIFT_PRI2)
#define DPS3_MASK	(0x7 << SHIFT_PRI3)
#define DPS4_MASK	(0x7 << SHIFT_PRI4)
#define DPS5_MASK	(0x7 << SHIFT_PRI5)
#define DPS6_MASK	(0x7 << SHIFT_PRI6)
#define DPS7_MASK	(0x7 << SHIFT_PRI7)
#define DPS8_MASK	(0x7 << SHIFT_PRI8)

#define DEFR		0x00020
#define CODE_DEFR	(0x7773<<16)
#define EXSL_DEFR	(1<<12)
#define EXVL_DEFR	(1<<11)
#define EXUP_DEFR	(1<<5)
#define VCUP_DEFR	(1<<4)
#define DEFE_DEFR	(1<<0)
#define DEFAULT_DEFR	(CODE_DEFR | DEFE_DEFR)

#define DAPCR		0x00024

#define DEFR2		0x00034
#define CODE_DEFR2	(0x7775<<16)
#define DEFE2_DEFR2	(1<<0)
#define DEFAULT_DEFR2	(CODE_DEFR2 | DEFE2_DEFR2)

#define DEFR3		0x00038
#define CODE_DEFR3	(0x7776<<16)
#define EVDA_DEFR3	(1<<14)
#define EVDM_1_DEFR3	(1<<12)
#define EVDM_2_DEFR3	(2<<12)
#define EVDM_3_DEFR3	(3<<12)
#define VMSM2_EMA_DEFR3	(1<<6)
#define VMSM1_ENA_DEFR3	(1<<4)
#define DEFE3_DEFR3	(1<<0)
#define DEFAULT_DEFR3	(CODE_DEFR3 | DEFE3_DEFR3)

#define DEFR4		0x0003c
#define CODE_DEFR4	(0x7777<<16)
#define LRUO_DEFR4	(0<<5)
#define PRI_DEFR4	(1<<5)
#define SPCD_DEFR4	(0<<4)
#define SPCE_DEFR4	(1<<4)
#define BPP16_DEFR4	(CODE_DEFR4 | LRUO_DEFR4 | SPCD_DEFR4)
#define BPP32_DEFR4	(CODE_DEFR4 | PRI_DEFR4 | SPCD_DEFR4)

#define DVCSR		0x000d0

#define DEFR5		0x000e0
#define DDLTR		0x000e4
#define DEFR6		0x000e8
#define CODE_DEFR6	(0x7778<<16)
#define TCNE2_DEFR6	(0x01 << 6)
#define DEFAULT_DEFR6	(CODE_DEFR6 | TCNE2_DEFR6)

#define D2SYSR		0x30000	/* display 2 */
#define ILTS_D2SYSR	(1<<29)
#define DSEC_D2SYSR	(1<<20)
#define IUPD_D2SYSR	(1<<16)
#define DRES_D2SYSR	(1<<9)
#define DEN_D2SYSR	(1<<8)
#define MASTER_D2SYSR	(0<<6)
#define MASTER_D2SYSR_MASK	(3<<6)
#define SYNC_ENABLE_D2SYSR	(0<<6)
#define SYNC_DISABLE_D2SYSR	(2<<6)
#define SYNC_CHG_D2SYSR	(1<<6)
#define TV_SYNC_D2SYSR	(2<<6)
#define N_INTR_D2SYSR	(0<<4)
#define INTR_SYNC_D2SYSR	(2<<4)
#define INTR_VIDEO_D2SYSR	(3<<4)

#define D2SMR		0x30004

#define D2SRCR		0x3000c

#define D2EFR		0x30020
#define CODE_D2EFR	(0x7773<<16)
#define EXSL_D2EFR	(1<<12)
#define EXVL_D2EFR	(1<<11)
#define EXUP_D2EFR	(1<<5)
#define VCUP_D2EFR	(1<<4)
#define DEFE_D2EFR	(1<<0)
#define DEFAULT_D2EFR	(CODE_D2EFR | DEFE_D2EFR)

#define D2EFR2		0x30034
#define CODE_D2EFR2	(0x7775<<16)
#define DEFE2_D2EFR2	(1<<0)
#define DEFAULT_D2EFR2	(CODE_D2EFR2 | DEFE2_D2EFR2)

#define D2EFR3		0x30038
#define CODE_D2EFR3	(0x7776<<16)
#define EVDA_D2EFR3	(1<<14)
#define EVDM_1_D2EFR3	(1<<12)
#define EVDM_2_D2EFR3	(2<<12)
#define EVDM_3_D2EFR3	(3<<12)
#define VMSM2_EMA_D2EFR3	(1<<6)
#define VMSM1_ENA_D2EFR3	(1<<4)
#define DEFE3_D2EFR3	(1<<0)
#define DEFAULT_D2EFR3	(CODE_D2EFR3 | DEFE3_D2EFR3)

#define D2EFR4		0x3003c
#define CODE_D2EFR4	(0x7777<<16)
#define LRUO_D2EFR4	(0<<5)
#define PRI_D2EFR4	(1<<5)
#define SPCD_D2EFR4	(0<<4)
#define SPCE_D2EFR4	(1<<4)
#define BPP16_D2EFR4	(CODE_D2EFR4 | LRUO_D2EFR4 | SPCD_D2EFR4)
#define BPP32_D2EFR4	(CODE_D2EFR4 | PRI_D2EFR4 | SPCD_D2EFR4)

/* Display timing generation register offset definitions */
#define HDSR		0x00040	/* display 1 */
#define HDER		0x00044
#define VDSR		0x00048
#define VDER		0x0004c
#define HCR		0x00050
#define HSWR		0x00054
#define VCR		0x00058
#define VSPR		0x0005c
#define EQWR		0x00060
#define SPWR		0x00064
#define CLAMPSR		0x00070
#define CLAMPWR		0x00074
#define DESR		0x00078
#define DEWR		0x0007c

#define DISP2_REG_OFFSET 0x30000

/* Display attribute register offset definitions */
#define CP1TR		0x00080	/* display 1 */
#define CP2TR		0x00084
#define CP3TR		0x00088
#define CP4TR		0x0008c

#define DOOR		0x00090
#define DEFAULT_DOOR	(0x0)	/* Output color is black */

#define CDER		0x00094

#define BPOR		0x00098
#define DEFAULT_BPOR	(0x0)	/* Background color is black */

#define RINTOFSR	0x0009c

#define DSHPR		0x000c8
#define CODE_DSHPR	(0x7776<<16)
#define PRIH_DSHPR	(0xa<<4)
#define BPP16_PRIL_DSHPR	(CODE_DSHPR | PRIH_DSHPR | 0x8)
#define BPP32_PRIL_DSHPR	(CODE_DSHPR | PRIH_DSHPR | 0x9)
#define MASK_PRIL	0xffffff00

/* Display plane register offset definitions */
#define P1MR		0x00100 /* plane 1 */
#define PnDDDF_8BPP_PnMR	(0<<0)	/* 8bit */
#define PnDDDF_16BPP_32BPP_PnMR	(1<<0)	/* 16bit or 32bit */
#define PnDDDF_ARGB_PnMR	(2<<0)	/* ARGB */
#define PnDDDF_YC_PnMR		(3<<0)	/* YC */
#define PnDDDF_MASK		(3<<0)	/* PnDDF_MASK */
#define PnBM_MD_PnMR		(0<<4)	/* Manual display change mode */
#define PnBM_AR_PnMR		(1<<4)	/* Auto rendering mode */
#define PnBM_AD_PnMR		(2<<4)	/* Auto display change mode */
#define PnBM_VC_PnMR		(3<<4)	/* Video Capture Mode */
#define PnDC_ON_PnMR		(1<<7)	/* Display area change */
#define PnCPSL_CP1_PnMR		(0<<8)	/* Color pallet selected 1 */
#define PnCPSL_CP2_PnMR		(1<<8)	/* Color pallet selected 2 */
#define PnCPSL_CP3_PnMR		(2<<8)	/* Color pallet selected 3 */
#define PnCPSL_CP4_PnMR		(3<<8)	/* Color pallet selected 4 */
#define PnSPIM_TPON_PnMR	(0<<12)	/* Transparent Color on */
#define PnSPIM_ALPON_PnMR	(1<<12)	/* Alphar blend on */
#define PnSPIM_EORON_PnMR	(2<<12)	/* EOR on */
#define PnSPIM_TPOFF_PnMR	(4<<12)	/* Transparent Color off */
#define PnSPIM_ALPOFF_PnMR	(5<<12)	/* Alphar blend off */
#define PnSPIM_EOROFF_PnMR	(6<<12)	/* EOR off */
#define PnWAE_ON_PnMR		(1<<16)	/* Wrap around Enable */
#define PnTC_R_PnMR			(0<<17)	/* Tranparent color is PnTC1R */
#define PnTC_CP_PnMR		(1<<17)	/* Tranparent color is color pallet */
#define PnYCDF_UYVY_PnMR	(0<<20)	/* UYVY format */
#define PnYCDF_YUYV_PnMR	(1<<20)	/* YUYV format */
#define PnVISL_VIN0_PnMR	(0<<26)	/* use Video Input 0 */
#define PnVISL_VIN1_PnMR	(1<<26)	/* use Video Input 1 */
#define MODE_16BPP_32BPP_PnMR	(PnDDDF_16BPP_32BPP_PnMR \
							| PnBM_MD_PnMR | \
							 PnSPIM_TPOFF_PnMR)
#define MODE_RGB_PnMR		(PnDDDF_ARGB_PnMR | PnBM_MD_PnMR | \
							 PnSPIM_TPOFF_PnMR)

#define P1MWR		0x00104

#define P1ALPHAR	0x00108
#define DEFAULT_PnALPHAR		(0x000000ff)

#define P1DSXR		0x00110
#define P1DSYR		0x00114

#define P1DPXR		0x00118
#define DEFAULT_PnDPXR	(0x0)

#define P1DPYR		0x0011c
#define DEFAULT_PnDPYR	(0x0)

#define P1DSA0R		0x00120
#define P1DSA1R		0x00124
#define P1DSA2R		0x00128
#define PnDSA_MASK	(0xfffffff0)

#define P1SPXR		0x00130
#define DEFAULT_PnSPXR	(0x0)

#define P1SPYR		0x00134
#define DEFAULT_PnSPYR	(0x0)

#define P1WASPR		0x00138
#define DEFAULT_PnWASPR	(0x0)

#define P1WAMWR		0x0013c
#define DEFAULT_PnWAMWR	(4095)

#define P1BTR		0x00140
#define DEFAULT_PnBTR	(0x0)

#define P1TC1R		0x00144
#define P1TC2R		0x00148
#define P1TC3R		0x0014c

#define P1MLR		0x00150
#define DEFAULT_PnMLR	(0x0)

#define P1SWAPR		0x00180

#define P1DDCR		0x00184
#define CODE_DDCR	(0x7775<<16)
#define PnLRGB1_DDCR	(1<<11)
#define PnLRGB0_DDCR	(1<<10)
#define BPP16_DDCR		(CODE_DDCR)
#define BPP32AR_DDCR	(CODE_DDCR | PnLRGB0_DDCR)
#define BPP32GB_DDCR	(CODE_DDCR | PnLRGB1_DDCR)

#define P1DDCR2		0x00188
#define CODE_DDCR2	(0x7775<<16)
#define PnNV21_DDCR2	(1<<5)
#define PnY420_DDCR2	(1<<4)
#define PnDIVU_DDCR2	(1<<1)
#define PnDIVY_DDCR2	(1<<0)

#define P1DDCR4		0x00190
#define CODE_DDCR4	(0x7766<<16)
#define P1EDF_MASK	0x07
#define P1DDCR4_ARGB8888	0x01
#define P1DDCR4_RGB888		0x02
#define P1DDCR4_RGB666		0x03

#define PLANE_OFF	0x00100
#define PLANE1		0x0
#define PLANE2		0x1
#define PLANE3		0x2
#define PLANE4		0x3
#define PLANE5		0x4
#define PLANE6		0x5
#define PLANE7		0x6
#define PLANE8		0x7

#define P2MR		0x00200 /* plane 2 */
#define P2MWR		0x00204
#define P2ALPHAR	0x00208
#define P2DSXR		0x00210
#define P2DSYR		0x00214
#define P2DPXR		0x00218
#define P2DPYR		0x0021c
#define P2DSA0R		0x00220
#define P2DSA1R		0x00224
#define P2DSA2R		0x00228
#define P2SPXR		0x00230
#define P2SPYR		0x00234
#define P2WASPR		0x00238
#define P2WAMWR		0x0023c
#define P2BTR		0x00240
#define P2TC1R		0x00244
#define P2TC2R		0x00248
#define P2TC3R		0x0024c
#define P2MLR		0x00250
#define P2SWAPR		0x00280
#define P2DDCR		0x00284
#define P2DDCR2		0x00288
#define P2DDCR4		0x00290

#define P3MR		0x00300 /* plane 3 */
#define P3MWR		0x00304
#define P3ALPHAR	0x00308
#define P3DSXR		0x00310
#define P3DSYR		0x00314
#define P3DPXR		0x00318
#define P3DPYR		0x0031c
#define P3DSA0R		0x00320
#define P3DSA1R		0x00324
#define P3DSA2R		0x00328
#define P3SPXR		0x00330
#define P3SPYR		0x00334
#define P3WASPR		0x00338
#define P3WAMWR		0x0033c
#define P3BTR		0x00340
#define P3TC1R		0x00344
#define P3TC2R		0x00348
#define P3TC3R		0x0034c
#define P3MLR		0x00350
#define P3SWAPR		0x00380
#define P3DDCR		0x00384
#define P3DDCR2		0x00388
#define P3DDCR4		0x00390

#define P4MR		0x00400 /* plane 4 */
#define P4MWR		0x00404
#define P4ALPHAR	0x00408
#define P4DSXR		0x00410
#define P4DSYR		0x00414
#define P4DPXR		0x00418
#define P4DPYR		0x0041c
#define P4DSA0R		0x00420
#define P4DSA1R		0x00424
#define P4DSA2R		0x00428
#define P4SPXR		0x00430
#define P4SPYR		0x00434
#define P4WASPR		0x00438
#define P4WAMWR		0x0043c
#define P4BTR		0x00440
#define P4TC1R		0x00444
#define P4TC2R		0x00448
#define P4TC3R		0x0044c
#define P4MLR		0x00450
#define P4SWAPR		0x00480
#define P4DDCR		0x00484
#define P4DDCR2		0x00488
#define P4DDCR4		0x00490

#define P5MR		0x00500 /* plane 5 */
#define P5MWR		0x00504
#define P5ALPHAR	0x00508
#define P5DSXR		0x00510
#define P5DSYR		0x00514
#define P5DPXR		0x00518
#define P5DPYR		0x0051c
#define P5DSA0R		0x00520
#define P5DSA1R		0x00524
#define P5DSA2R		0x00528
#define P5SPXR		0x00530
#define P5SPYR		0x00534
#define P5WASPR		0x00538
#define P5WAMWR		0x0053c
#define P5BTR		0x00540
#define P5TC1R		0x00544
#define P5TC2R		0x00548
#define P5TC3R		0x0054c
#define P5MLR		0x00550
#define P5SWAPR		0x00580
#define P5DDCR		0x00584
#define P5DDCR2		0x00588
#define P5DDCR4		0x00590

#define P6MR		0x00600 /* plane 6 */
#define P6MWR		0x00604
#define P6ALPHAR	0x00608
#define P6DSXR		0x00610
#define P6DSYR		0x00614
#define P6DPXR		0x00618
#define P6DPYR		0x0061c
#define P6DSA0R		0x00620
#define P6DSA1R		0x00624
#define P6DSA2R		0x00628
#define P6SPXR		0x00630
#define P6SPYR		0x00634
#define P6WASPR		0x00638
#define P6WAMWR		0x0063c
#define P6BTR		0x00640
#define P6TC1R		0x00644
#define P6TC2R		0x00648
#define P6TC3R		0x0064c
#define P6MLR		0x00650
#define P6SWAPR		0x00680
#define P6DDCR		0x00684
#define P6DDCR2		0x00688
#define P6DDCR4		0x00690

#define P7MR		0x00700 /* plane 7 */
#define P7MWR		0x00704
#define P7ALPHAR	0x00708
#define P7DSXR		0x00710
#define P7DSYR		0x00714
#define P7DPXR		0x00718
#define P7DPYR		0x0071c
#define P7DSA0R		0x00720
#define P7DSA1R		0x00724
#define P7DSA2R		0x00728
#define P7SPXR		0x00730
#define P7SPYR		0x00734
#define P7WASPR		0x00738
#define P7WAMWR		0x0073c
#define P7BTR		0x00740
#define P7TC1R		0x00744
#define P7TC2R		0x00748
#define P7TC3R		0x0074c
#define P7MLR		0x00750
#define P7SWAPR		0x00780
#define P7DDCR		0x00784
#define P7DDCR2		0x00788
#define P7DDCR4		0x00790

#define P8MR		0x00800 /* plane 8 */
#define P8MWR		0x00804
#define P8ALPHAR	0x00808
#define P8DSXR		0x00810
#define P8DSYR		0x00814
#define P8DPXR		0x00818
#define P8DPYR		0x0081c
#define P8DSA0R		0x00820
#define P8DSA1R		0x00824
#define P8DSA2R		0x00828
#define P8SPXR		0x00830
#define P8SPYR		0x00834
#define P8WASPR		0x00838
#define P8WAMWR		0x0083c
#define P8BTR		0x00840
#define P8TC1R		0x00844
#define P8TC2R		0x00848
#define P8TC3R		0x0084c
#define P8MLR		0x00850
#define P8SWAPR		0x00880
#define P8DDCR		0x00884
#define P8DDCR2		0x00888
#define P8DDCR4		0x00890

#define AP1MR		0x0a100 /* alpha plane 1 */
#define AP1MWR		0x0a104
#define AP1DSXR		0x0a110
#define AP1DSYR		0x0a114
#define AP1DPXR		0x0a118
#define AP1DPYR		0x0a11c
#define AP1DSA0R	0x0a120
#define AP1DSA1R	0x0a124
#define AP1DSA2R	0x0a128
#define AP1SPXR		0x0a130
#define AP1SPYR		0x0a134
#define AP1WASPR	0x0a138
#define AP1WAMWR	0x0a13c
#define AP1BTR		0x0a140
#define AP1MLR		0x0a150
#define AP1SWAPR	0x0a180
#define AP1DDCR4	0x0a190

#define AP2MR		0x0a200 /* alpha plane 2 */
#define AP2MWR		0x0a204
#define AP2DSXR		0x0a210
#define AP2DSYR		0x0a214
#define AP2DPXR		0x0a218
#define AP2DPYR		0x0a21c
#define AP2DSA0R	0x0a220
#define AP2DSA1R	0x0a224
#define AP2DSA2R	0x0a228
#define AP2SPXR		0x0a230
#define AP2SPYR		0x0a234
#define AP2WASPR	0x0a238
#define AP2WAMWR	0x0a23c
#define AP2BTR		0x0a240
#define AP2MLR		0x0a250
#define AP2SWAPR	0x0a280
#define AP2DDCR4	0x0a290

/* Display capture register offset definitions */
#define DCMWR		0x0c104
#define DCSAR		0x0c120
#define DCMLR		0x0c150
#define DC2MWR		0x0c204
#define DC2SAR		0x0c220
#define DC2MLR		0x0c250

/* Color palette */
#define CP1_000R	0x01000
#define CP1_255R	0x013fc
#define CP2_000R	0x02000
#define CP2_255R	0x023fc
#define CP3_000R	0x03000
#define CP3_255R	0x033fc
#define CP4_000R	0x04000
#define CP4_255R	0x043fc

/* External synchronization control register offset definitions */
#define ESCR		0x10000
#define DCLKIN_ESCR	(0<<20)
#define CLKS_ESCR	(1<<20)
#define DCLKDIS_ESCR	(1<<16)
#define SYNCSEL_OFF_ESCR	(0<<8)
#define SYNCSEL_EXVSYNC_ESCR	(2<<8)
#define SYNCSEL_EXHSYNC_ESCR	(3<<8)

#define OTAR		0x10004
#define DEFAULT_OTAR	(0)

#define ESCR2		0x31000
#define DCLKIN_ESCR2	(0<<20)
#define CLKS_ESCR2	(1<<20)
#define DCLKDIS_ESCR2	(1<<16)
#define SYNCSEL_OFF_ESCR2	(0<<8)
#define SYNCSEL_EXVSYNC_ESCR2	(2<<8)
#define SYNCSEL_EXHSYNC_ESCR2	(3<<8)
#define OTAR2		0x31004
#define DEFAULT_OTAR2	(0)

/* Output control of two display systems register offset definitions */
#define DORCR		0x11000
#define PG2T_DORCR	(1<<30)
#define DK2S_DORCR	(1<<28)
#define PG2D_DS1_DORCR	(0<<24)
#define PG2D_DS2_DORCR	(1<<24)
#define PG2D_FIX0_DORCR	(2<<24)
#define PG2D_DOOR_DORCR	(3<<24)
#define DR1D_DORCR	(1<<21)
#define PG1D_DS1_DORCR	(0<<16)
#define PG1D_DS2_DORCR	(1<<16)
#define PG1D_FIX0_DORCR	(2<<16)
#define PG1D_DOOR_DORCR	(3<<16)
#define RGPV_DORCR	(1<<4)
#define DPRS_DORCR	(1<<0)
#define DU0_SINGLE_DORCR	(0)

#define DPTSR		0x11004
#define DPTSR_P2DK	(0x01 << 17)
#define DPTSR_P1DK	(0x01 << 16)
#define DPTSR_P2TS	(0x01 << 1)
#define DPTSR_P1TS	(0x01 << 0)
#define DPTSR_ALL2	0x00ff00ff

#define DAPTSR		0x11008
#define DS1PR		0x11020
#define DS2PR		0x11024
#define S2S2_2		(0x02 << 4)
#define S2S1_1		(0x01 << 0)
#define S2S8_1		(0x01 << 28)

/* YC-RGB conversion coefficient register offset definitions */
#define YNCR		0x11080
#define YNOR		0x11084
#define CRNOR		0x11088
#define CBNOR		0x1108c
#define RCRCR		0x11090
#define GCRCR		0x11094
#define GCBCR		0x11098
#define BCBCR		0x1109c

#endif /* __ASM_RCARFB_H__ */
