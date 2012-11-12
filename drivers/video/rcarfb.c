/*
 * Renesas Display Unit Framebuffer
 *
 * Copyright (C) 2012 Renesas Electronics Corporation
 *
 * This file is based on the sh_mobile_lcdcfb.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h> /* for copy_from_user() */
#include <video/rcarfb.h>

#include <mach/hardware.h>

#include <video/rcarfb.h>

#define PALETTE_NR	256
#define DEV_NAME	"rcarfb"
#define PLANE_NUM	2

static char __devinitdata *mode_option[2] = { "640x480-16", "640x480-16" };

struct rcar_pos_param {
	unsigned long	px;	/* position of x */
	unsigned long	py;	/* position of y */
};

struct rcar_size_param {
	unsigned long	sx;	/* size of x */
	unsigned long	sy;	/* size of y */
};

struct rcar_plane_param {
	unsigned long mwx;		/* MWR:plane memory width */
	unsigned long alpha;		/* ALPHA:blending rasio */
	struct rcar_size_param disp;	/* DSXR,DSYR:display size x/y */
	struct rcar_pos_param moni;	/* DPXR,DPYR:display position x/y */
	unsigned long disp_area0_base;	/* DSA0:display area start address0 */
	unsigned long disp_area1_base;	/* DSA1:display area start address1 */
	struct rcar_pos_param mem;	/* SPX,SPY:start pixel position x/y */
	unsigned long	waspr;		/* WASPR:wrap around start position */
	unsigned long	wamwr;		/* WAMWR:wrap around memory width */
	unsigned long	btr;		/* BTR:plane n blinking time */
	unsigned long	mlr;		/* MLR:plan n memory length */
	int plane_pixel;		/* plan n bit per pixel */
	int disp_area;		/* use plan number flag */
};

struct rcar_du_priv {
	struct device *dev;
	void __iomem *base;
	int		irq;
	int		du_channel;
	unsigned long	reg_offset;

	u32 pseudo_palette[PALETTE_NR];
	struct fb_info *info;
	dma_addr_t	dma_handle;

	int		bit_per_pixel;
	struct rcar_size_param pan_offset;
	int		blanked;
	unsigned long	plane_area;
	struct rcar_plane_param plane[8];
	struct clk *duclk;
	int		clock_rate;
	struct fb_videomode current_mode;
	struct rcar_reso_info *dispdev;
	wait_queue_head_t vsync_wait;
	int		vsync_flag;
};

static struct fb_fix_screeninfo rcar_du_fix  = {
	.id =		"R-Car DU",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.accel =	FB_ACCEL_NONE,
	.xpanstep =	0,
	.ypanstep =	0,
	.ywrapstep =	0,
	.smem_start =	0,
};

static void du_write(struct rcar_du_priv *priv,
		       unsigned long reg_offs, unsigned long data)
{
	iowrite32(data, priv->base + reg_offs);
}

static void du_write_ch(struct rcar_du_priv *priv,
		       unsigned long reg_offs, unsigned long data)
{
	iowrite32(data, priv->base + priv->reg_offset + reg_offs);
}

static unsigned long du_read(struct rcar_du_priv *priv,
			       unsigned long reg_offs)
{
	return ioread32(priv->base + reg_offs);
}

static unsigned long du_read_ch(struct rcar_du_priv *priv,
			       unsigned long reg_offs)
{
	return ioread32(priv->base + priv->reg_offset + reg_offs);
}

static int du_check_disp_mode(unsigned long xres, unsigned long yres)
{
	if ((xres > MAX_XRES) || (yres > MAX_YRES))
		return -1;

	return 0;
}

static void du_set_plane(struct rcar_du_priv *priv, unsigned long plane_no)
{
	struct rcar_plane_param *pl;
	struct rcar_pos_param moni, mem;
	struct rcar_size_param disp;

	pl = &priv->plane[plane_no];
	moni = pl->moni;
	mem = pl->mem;
	disp = pl->disp;

	if (priv->plane[plane_no].plane_pixel == BITPARPIXEL_16) {
		/* 16bpp */
		du_write(priv, P1DDCR4 + (plane_no * PLANE_OFF),
		 CODE_DDCR4 | (du_read(priv, P1DDCR4 +
		  (plane_no * PLANE_OFF)) & ~P1EDF_MASK));
	} else {
		/* 32bpp */
		du_write(priv, P1DDCR4 + (plane_no * PLANE_OFF),
			 CODE_DDCR4 | ((du_read(priv, P1DDCR4 +
			 (plane_no * PLANE_OFF))) & ~P1EDF_MASK)
			  | P1DDCR4_ARGB8888);
	}

	/* for plane */
	du_write(priv, P1MR + (plane_no * PLANE_OFF), MODE_16BPP_32BPP_PnMR);
	du_write(priv, P1MWR + (plane_no * PLANE_OFF), disp.sx);
	du_write(priv, P1ALPHAR + (plane_no * PLANE_OFF), pl->alpha);
	du_write(priv, P1DSXR + (plane_no * PLANE_OFF), disp.sx);
	du_write(priv, P1DSYR + (plane_no * PLANE_OFF), disp.sy);
	du_write(priv, P1DPXR + (plane_no * PLANE_OFF), moni.px);
	du_write(priv, P1DPYR + (plane_no * PLANE_OFF), moni.py);
	du_write(priv, P1DSA0R + (plane_no * PLANE_OFF),
		 pl->disp_area0_base);
	du_write(priv, P1SPXR + (plane_no * PLANE_OFF), mem.px);
	du_write(priv, P1SPYR + (plane_no * PLANE_OFF), mem.py);
	du_write(priv, P1WASPR + (plane_no * PLANE_OFF), pl->waspr);
	du_write(priv, P1WAMWR + (plane_no * PLANE_OFF), pl->wamwr);
	du_write(priv, P1BTR + (plane_no * PLANE_OFF), pl->btr);
	du_write(priv, P1MLR + (plane_no * PLANE_OFF), pl->mlr);
}

static void du_set_plane_mode(struct rcar_du_priv *priv,
				struct fb_info *info, unsigned long plane_no)
{
	unsigned long offset;

	offset = info->var.xres * info->var.yres * info->var.bits_per_pixel / 8;

	du_write(priv, DEFR4, BPP16_DEFR4);

	if (priv->du_channel == 0) {
		du_write(priv, DS1PR,
			((plane_no + 1) << (28 - (4 * plane_no))));
	} else {
		du_write(priv, DS2PR,
			((plane_no + 1) << (28 - (4 * plane_no))));
	}

	/* set plane param */
	priv->plane[plane_no].plane_pixel = info->var.bits_per_pixel;
	priv->plane[plane_no].alpha = DEFAULT_PnALPHAR;
	priv->plane[plane_no].disp.sx = info->var.xres;
	priv->plane[plane_no].disp.sy = info->var.yres;
	priv->plane[plane_no].moni.px = DEFAULT_PnDPXR;
	priv->plane[plane_no].moni.py = DEFAULT_PnDPYR;

	priv->plane[plane_no].disp_area0_base =
		(info->fix.smem_start & PnDSA_MASK);
	priv->plane[plane_no].disp_area1_base =
		((info->fix.smem_start + offset) & PnDSA_MASK);

	priv->plane[plane_no].mem.px = DEFAULT_PnSPXR;
	priv->plane[plane_no].mem.py = DEFAULT_PnSPYR;
	priv->plane[plane_no].waspr = DEFAULT_PnWASPR;
	priv->plane[plane_no].wamwr = DEFAULT_PnWAMWR;
	priv->plane[plane_no].btr = DEFAULT_PnBTR;
	priv->plane[plane_no].mlr = DEFAULT_PnMLR;

	/* set register */
	du_set_plane(priv, plane_no);

	return;
}

static irqreturn_t rcar_du_irq(int irq, void *data)
{
	struct rcar_du_priv *priv = data;
	unsigned long intr = 0;

	/* acknowledge interrupt */
	intr = du_read_ch(priv, DSSR);
	/* clear interrupt */
	du_write_ch(priv, DSRCR, intr);

	if (intr & VBK_DSSR) {
		priv->vsync_flag = priv->du_channel;
		wake_up_interruptible(&priv->vsync_wait);
	}

	return IRQ_HANDLED;
}

static void rcar_du_start_stop(struct rcar_du_priv *priv, int start)
{
	unsigned long tmp = du_read(priv, DSYSR);

	tmp &= ~(DRES_DSYSR | DEN_DSYSR | MASTER_DSYSR_MASK);
	/* start or stop the DU */
	if (start == DU_START) {
		/* Request DU Clock Operation */
		clk_enable(priv->duclk);
		if (priv->du_channel != 0) {
			du_write(priv, D2SYSR, SYNC_ENABLE_D2SYSR);

			/* Disable DU */
			du_write(priv, DSYSR, tmp | DRES_DSYSR);
			/* Enable DU */
			du_write(priv, DSYSR, tmp | DEN_DSYSR);
		}
		du_write(priv, DSYSR, tmp | DEN_DSYSR);
	} else {
		/* Stanby DU Clock Operation */
		du_write(priv, DSYSR, tmp | DRES_DSYSR);
		clk_disable(priv->duclk);
	}
}

static void rcar_du_stop(struct rcar_du_priv *priv)
{
	/* stop the lcdc */
	rcar_du_start_stop(priv, DU_STOP);
}

static int rcar_du_setup_clocks(struct rcar_du_priv *priv,
				int clks, struct fb_videomode *mode)
{
	unsigned long hc;
	unsigned long vc;
	unsigned long pclk;
	unsigned long tmp;

	/* DU Functionality expansion */
	du_write(priv, DEFR, DEFAULT_DEFR);
	du_write(priv, DEFR2, DEFAULT_DEFR2);
	du_write(priv, DEFR3, DEFAULT_DEFR3);

	/* Operate T-CON for DU1 */
	du_write(priv, DEFR6, DEFAULT_DEFR6);
	/* Use DS1PR or DS2PR */
	du_write(priv, DORCR,
		 PG2T_DORCR | DK2S_DORCR | PG2D_DS2_DORCR | DPRS_DORCR);

	/* Calculate pixel clock */
	hc = mode->hsync_len + mode->left_margin + mode->xres
		+ mode->right_margin;
	vc = mode->upper_margin + mode->yres + mode->lower_margin
		+ mode->vsync_len;
	pclk = hc * vc * mode->refresh;
	tmp = (((clks / (pclk / 2)) + 1) / 2) - 1;

	if (priv->du_channel == 0) {
		tmp |= CLKS_ESCR;
		du_write(priv, ESCR, tmp);
		du_write(priv, OTAR, DEFAULT_OTAR);
	} else {
		/* Use Dot clock generation part 2 in DU1 */
		du_write(priv, DPTSR,
			 (DPTSR_P1DK | DPTSR_P1TS) << priv->plane_area);
		tmp |= CLKS_ESCR2;
		du_write(priv, ESCR2, tmp);
		du_write(priv, OTAR2, DEFAULT_OTAR2);
	}

	return 0;
}

static int du_set_disp_timing(struct rcar_du_priv *priv,
	struct fb_videomode *mode)
{
	int ret;

	/* set dotclock */
	ret = rcar_du_setup_clocks(priv, priv->clock_rate, mode);
	if (ret)
		return ret;

	du_write_ch(priv, HDSR, mode->hsync_len + mode->left_margin - 19);
	du_write_ch(priv, HDER, mode->hsync_len + mode->left_margin - 19
				+ mode->xres);
	du_write_ch(priv, HSWR, mode->hsync_len - 2);
	du_write_ch(priv, HCR,  mode->hsync_len + mode->left_margin
				+ mode->xres + mode->right_margin);

	du_write_ch(priv, VDSR, mode->upper_margin - 2);
	du_write_ch(priv, VDER, mode->upper_margin - 2 + mode->yres);
	du_write_ch(priv, VSPR, mode->upper_margin + mode->yres
				+ mode->lower_margin - 1);
	du_write_ch(priv, VCR,  mode->upper_margin + mode->yres
				+ mode->lower_margin + mode->vsync_len - 1);

	/* Display Off mode Output */
	du_write_ch(priv, BPOR, DEFAULT_BPOR);
	/* Background Plane Output */
	du_write_ch(priv, DOOR, DEFAULT_DOOR);

	return 0;
}

static int rcar_du_start(struct rcar_du_priv *priv)
{
	struct fb_info *info;

	/* Request DU Clock Operation */
	clk_enable(priv->duclk);

	info = priv->info;
	/* setting the display timing create register */
	du_set_disp_timing(priv, &priv->current_mode);

	/* set priv param */
	priv->pan_offset.sx = 0;
	priv->pan_offset.sy = 0;
	priv->bit_per_pixel = info->var.bits_per_pixel;

	/* plane setting */
	du_set_plane_mode(priv, info, priv->plane_area);

	/* Clear VBK interrupt */
	du_write_ch(priv, DSRCR, VBK_DSSR);
	/* Enable VBK interrupt */
	du_write_ch(priv, DIER, VBK_DSSR);

	/* start the DU */
	rcar_du_start_stop(priv, DU_START);

	return 0;
}

static int rcar_du_setcolreg(u_int regno,
			     u_int red, u_int green, u_int blue,
			     u_int transp, struct fb_info *info)
{
	u32 *palette = info->pseudo_palette;

	if (regno >= PALETTE_NR)
		return -EINVAL;

	switch (info->var.bits_per_pixel) {
	case BITPARPIXEL_16:
		palette = info->pseudo_palette;
		palette[regno] = (red & 0xf800) |
				((green & 0xfc00) >> 5) |
				((blue & 0xf800) >> 11);
		break;
	case BITPARPIXEL_24:
	case BITPARPIXEL_32:
		palette = info->pseudo_palette;
		palette[regno] = ((transp & 0xff00) << 16) |
				((red & 0xff00) << 8) |
				(green & 0xff00) |
				((blue & 0xff00) >> 8);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int rcar_du_blank(int blank_mode, struct fb_info *info)
{
#if RCARFB_SUPPORTBLANK
	struct rcar_du_priv *priv = info->par;
	unsigned long tmp = readl(IO_ADDRESS(RC_BASE_DISPLAY + DSYSR));
	unsigned long cpg_du_st, cpg_du_mask;

	tmp &= ~(DRES_DSYSR | DEN_DSYSR | MASTER_DSYSR_MASK);

	pr_info("%s: %s mode=%d\n", DEV_NAME, __func__, blank_mode);

	if ((blank_mode != FB_BLANK_UNBLANK) && (!priv->blanked)) {
		/* Display Reset */
		writel(tmp | DRES_DSYSR, IO_ADDRESS(RC_BASE_DISPLAY + DSYSR));
		if (priv->du_channel != 0) {
			writel(SYNC_DISABLE_D2SYSR,
				 IO_ADDRESS(RC_BASE_DISPLAY + D2SYSR));
		}
		/* Stanby DU Clock Operation */
		clk_disable(priv->duclk);
		priv->blanked = 1;
	} else if (priv->blanked) {
		/* Request DU Clock Operation */
		clk_enable(priv->duclk);
		/* Display start */
		writel(tmp | DEN_DSYSR, IO_ADDRESS(RC_BASE_DISPLAY + DSYSR));
		if (priv->du_channel != 0) {
			writel(SYNC_ENABLE_D2SYSR,
				 IO_ADDRESS(RC_BASE_DISPLAY + D2SYSR));
		}
		priv->blanked = 0;
	}
#endif
	return 0;

}

static int rcar_du_pan_display(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	struct rcar_du_priv *priv = info->par;

	unsigned long plane_no = priv->plane_area;

	if (var->xoffset != priv->pan_offset.sx ||
		 var->yoffset != priv->pan_offset.sy) {
		if ((var->xoffset == 0) && (var->yoffset == 0)) {
			du_write(priv, P1DSA0R + (plane_no * PLANE_OFF),
				 priv->plane[plane_no].disp_area0_base);
		} else if (var->yoffset == info->var.yres) {
			du_write(priv, P1DSA0R + (plane_no * PLANE_OFF),
				 priv->plane[plane_no].disp_area1_base);
		} else {
			return -EINVAL;
		}
		/* display the area0 */
		priv->pan_offset.sx = var->xoffset;
		priv->pan_offset.sy = var->yoffset;
	}

	return 0;
}

static int rcar_du_check_var(struct fb_var_screeninfo *var,
	 struct fb_info *info)
{
	switch (var->bits_per_pixel) {
	case BITPARPIXEL_16:	/* RGB 565 */
		var->red.offset    = 11;
		var->red.length    = 5;
		var->green.offset  = 5;
		var->green.length  = 6;
		var->blue.offset   = 0;
		var->blue.length   = 5;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case BITPARPIXEL_24:	/* RGB888 */
	case BITPARPIXEL_32:	/* ARGB 8888 */
		var->red.offset    = 16;
		var->red.length    = 8;
		var->green.offset  = 8;
		var->green.length  = 8;
		var->blue.offset   = 0;
		var->blue.length   = 8;
		var->transp.offset = 24;
		var->transp.length = 8;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int rcar_du_set_par(struct fb_info *info)
{
	struct rcar_du_priv *priv = info->par;
	struct fb_var_screeninfo *var;
	struct fb_videomode mode;
	char modestr[20];
	int ret = 0;

	/* check the resolution */
	if (du_check_disp_mode(info->var.xres, info->var.yres))
		return -EINVAL;

	/* Note: this doesn't change any of the the timing related fields.
	   We need to look up the mode using fb_find_mode */
	fb_var_to_videomode(&mode, &info->var);

	if (fb_mode_is_equal(&priv->current_mode, &mode))
		return ret;

	var = &info->var;

	/* Create string for resolution & depth */
	if (var->bits_per_pixel == BITPARPIXEL_16)
		snprintf(modestr, 20, "%dx%d-16", mode.xres, mode.yres);
	else if (var->bits_per_pixel == BITPARPIXEL_24 ||
		var->bits_per_pixel == BITPARPIXEL_32)
		snprintf(modestr, 20, "%dx%d-32", mode.xres, mode.yres);
	else
		return -EINVAL;

	/* Platform specific video modes */
	if (priv->dispdev->modes) {
		ret = fb_find_mode(&info->var, info, modestr,
			priv->dispdev->modes, priv->dispdev->num_modes,
			NULL, 16);
	}

	if (!ret) {
		/* Standard modes */
		ret = fb_find_mode(&info->var, info, modestr,
			NULL, 0, NULL, 16);
	}

	if (!ret)
		return -EINVAL;

	fb_var_to_videomode(&mode, &info->var);

	ret = du_set_disp_timing(priv, &mode);
	if (ret)
		goto err1;

	/* update param */
	info->fix.ypanstep = var->yres;

	/* check the BPP */
	ret = rcar_du_check_var(var, info);
	if (ret)
		goto err1;

	du_set_plane_mode(priv, info, priv->plane_area);

	/* update param */
	priv->bit_per_pixel = var->bits_per_pixel;
	info->fix.line_length = info->var.xres *
		(info->var.bits_per_pixel / 8);

	/* start the DU */
	rcar_du_start_stop(priv, DU_START);

	return ret;

err1:
	memset(info->screen_base, 0,
		MAX_XRES * MAX_YRES * FB_BYTES * PLANE_NUM);

	return ret;
}
static int rcar_du_set_pixfmt(struct fb_info *info, unsigned long pixfmt)
{
	int PixData;
	int ret = 0;
	struct rcar_du_priv *priv = info->par;

	if ((pixfmt < RCAR_PIXFMT_ARGB8888) || (pixfmt > RCAR_PIXFMT_YUYV)) {
		ret = -EINVAL;
		goto error_end;
	}

	if ((pixfmt == RCAR_PIXFMT_ARGB8888) ||
		 (pixfmt == RCAR_PIXFMT_ARGB0565)) {
		PixData = 0x01;
	} else if ((pixfmt == RCAR_PIXFMT_ARGB1555) ||
		 (pixfmt == RCAR_PIXFMT_RGB888)) {
		PixData = 0x02;
	} else if ((pixfmt == RCAR_PIXFMT_UYVY) ||
		 (pixfmt == RCAR_PIXFMT_YUYV) ||
		 (pixfmt == RCAR_PIXFMT_RGB666)) {
		PixData = 0x03;
	} else {
		ret = -EINVAL;
		goto error_end;
	}

	if ((pixfmt == RCAR_PIXFMT_ARGB1555) ||
		 (pixfmt == RCAR_PIXFMT_ARGB0565) ||
		 (pixfmt == RCAR_PIXFMT_UYVY) ||
		 (pixfmt == RCAR_PIXFMT_YUYV)) {
		if (pixfmt == RCAR_PIXFMT_YUYV) {
			/* YUYV */
			du_write(priv, (P1MR + (PLANE_OFF * priv->plane_area)),
				((du_read(priv, P1MR +
				(PLANE_OFF * priv->plane_area))
				 & (~PnYCDF_YUYV_PnMR)) | (PnYCDF_YUYV_PnMR)));
		} else {
			/* UYVY */
			du_write(priv, (P1MR + (PLANE_OFF * priv->plane_area)),
				 (du_read(priv, P1MR +
				  (PLANE_OFF * priv->plane_area))
				  & (~PnYCDF_YUYV_PnMR)));
		}
		du_write(priv, (P1DDCR4 + (PLANE_OFF * priv->plane_area)),
			 CODE_DDCR4 | (du_read(priv, P1DDCR4 +
			 (PLANE_OFF * priv->plane_area)) & ~P1EDF_MASK));
		du_write(priv, (P1MR + (PLANE_OFF * priv->plane_area)),
			 ((du_read(priv, P1MR + (PLANE_OFF * priv->plane_area))
			  & ~PnDDDF_MASK) | PixData));
	} else {
		du_write(priv, (P1DDCR4 + (PLANE_OFF * priv->plane_area)),
			 CODE_DDCR4 | ((du_read(priv, P1DDCR4 +
			 (PLANE_OFF * priv->plane_area)) & ~P1EDF_MASK)
			  | PixData));
		du_write(priv, (P1MR + (PLANE_OFF * priv->plane_area)),
			 ((du_read(priv, P1MR + (PLANE_OFF * priv->plane_area))
			  & ~PnDDDF_MASK) | PnDDDF_16BPP_32BPP_PnMR));
	}

error_end:
	return ret;
}

static int rcar_du_get_pixfmt(struct fb_info *info, unsigned long *pixfmt)
{
	int temp;
	int ret = 0;
	struct rcar_du_priv *priv = info->par;

	if (pixfmt == NULL) {
		pr_info("%s error.(pixfmt=NULL)\n", __func__);
		ret = -EINVAL;
		goto error_end;
	}

	temp = du_read(priv, P1DDCR4 + (PLANE_OFF * priv->plane_area))
		 & P1EDF_MASK;

	if (temp) {
		if (temp == P1DDCR4_ARGB8888) {
			*pixfmt = RCAR_PIXFMT_ARGB8888;
		} else if (temp == P1DDCR4_RGB888) {
			*pixfmt = RCAR_PIXFMT_RGB888;
		} else if (temp == P1DDCR4_RGB666) {
			*pixfmt = RCAR_PIXFMT_RGB666;
		} else {
			ret = -EINVAL;
			goto error_end;
		}
	} else {
		temp = du_read(priv, P1MR +
			 (PLANE_OFF * priv->plane_area)) & PnDDDF_MASK;
		if (temp == PnDDDF_16BPP_32BPP_PnMR) {
			*pixfmt = RCAR_PIXFMT_ARGB0565;
		} else if (temp == PnDDDF_ARGB_PnMR) {
			*pixfmt = RCAR_PIXFMT_ARGB1555;
		} else if (temp == PnDDDF_YC_PnMR) {
			if ((du_read(priv, P1MR +
				 (PLANE_OFF * priv->plane_area))
			 & PnYCDF_YUYV_PnMR) == PnYCDF_YUYV_PnMR)
				*pixfmt = RCAR_PIXFMT_YUYV;
			else
				*pixfmt = RCAR_PIXFMT_UYVY;
		} else {
			ret = -EINVAL;
			goto error_end;
		}
	}

error_end:
	return ret;
}

static int rcar_du_wait_for_vsync(struct fb_info *info)
{
	struct rcar_du_priv *priv = info->par;
	int ret;

	priv->vsync_flag = -1;
	ret = wait_event_interruptible_timeout(priv->vsync_wait,
			priv->vsync_flag == priv->du_channel,
			msecs_to_jiffies(100));

	if (!ret)
		return -ETIMEDOUT;

	return 0;
}

static int rcar_du_ioctl(struct fb_info *info, unsigned int cmd,
			 unsigned long arg)
{
	int retval;
	struct RCARFB_PIXFMT rcar_fmt;

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		retval = rcar_du_wait_for_vsync(info);
		break;

	case FBIOCS_PIXFMT:
		if (copy_from_user(&rcar_fmt, (void __user *)arg,
				sizeof(struct RCARFB_PIXFMT))) {
			pr_info("copy_from_user error. (FBIOCS_PIXFMT)\n");
			retval = -EFAULT;
			break;
		}
		retval = rcar_du_set_pixfmt(info, rcar_fmt.pixfmt);
		break;

	case FBIOCG_PIXFMT:
		retval = rcar_du_get_pixfmt(info, &rcar_fmt.pixfmt);
		if (copy_to_user((void __user *)arg, &rcar_fmt,
				sizeof(struct RCARFB_PIXFMT))) {
			pr_info("copy_to_user error. (FBIOCG_PIXFMT)\n");
			retval = -EFAULT;
		}
		break;

	default:
		retval = -ENOIOCTLCMD;
		break;
	}

	return retval;
}

static struct fb_ops rcar_du_ops = {
	.owner          = THIS_MODULE,
	.fb_check_var	= rcar_du_check_var,
	.fb_set_par	= rcar_du_set_par,
	.fb_setcolreg	= rcar_du_setcolreg,
	.fb_blank	= rcar_du_blank,
	.fb_read        = fb_sys_read,
	.fb_write       = fb_sys_write,
	.fb_fillrect	= sys_fillrect,
	.fb_copyarea	= sys_copyarea,
	.fb_imageblit	= sys_imageblit,
	.fb_pan_display = rcar_du_pan_display,
	.fb_ioctl	= rcar_du_ioctl,
};

static int rcar_du_remove(struct platform_device *pdev);

static int __devinit rcar_du_probe(struct platform_device *pdev)
{
	struct fb_info *info;
	struct rcar_du_priv *priv;
	struct resource *res;
	int error;
	void *buf;
	int i;
	int ret = 0;

	if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "no platform data defined\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i = platform_get_irq(pdev, 0);
	if (!res || i < 0) {
		dev_err(&pdev->dev, "cannot get platform resources\n");
		return -ENOENT;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "cannot allocate device data\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, priv);

	init_waitqueue_head(&priv->vsync_wait);

	error = request_irq(i, rcar_du_irq, IRQF_SHARED,
			    dev_name(&pdev->dev), priv);
	if (error) {
		dev_err(&pdev->dev, "unable to request irq\n");
		goto err1;
	}

	priv->irq = i;
	priv->base = ioremap_nocache(res->start, resource_size(res));
	if (!priv->base)
		goto err1;

	priv->dispdev = pdev->dev.platform_data;

	if (pdev->id == 0)
		priv->plane_area = DISP0_PLANE_NUM;
	else
		priv->plane_area = DISP1_PLANE_NUM;

	info = framebuffer_alloc(0, &pdev->dev);
	if (!info) {
		dev_err(&pdev->dev, "unable to allocate fb_info\n");
		error = -ENOMEM;
		goto err1;
	}

	priv->duclk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->duclk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		return -ENOENT;
		goto err1;
	}
	priv->clock_rate = clk_get_rate(priv->duclk);

	priv->info = info;
	info->fbops = &rcar_du_ops;

	/* setting resolution parameter  */
	info->var.xres = DEFAULT_XRES;
	info->var.yres = DEFAULT_YRES;

	/* setting pixel format */
	info->var.bits_per_pixel = priv->bit_per_pixel = DEFAULT_BITPARPIXEL;

	info->var.width = -1;
	info->var.height = -1;
	info->var.activate = FB_ACTIVATE_NOW;

	priv->du_channel = pdev->id;
	if (pdev->id == 0)
		priv->reg_offset = 0;
	else
		priv->reg_offset = DISP2_REG_OFFSET;

	if (mode_option[pdev->id]) {
		/* Platform specific video modes */
		if (priv->dispdev->modes) {
			ret = fb_find_mode(&info->var, info,
				mode_option[pdev->id], priv->dispdev->modes,
				priv->dispdev->num_modes, NULL, 16);
		}

		if (ret != 1) {
			/* Standard modes */
			ret = fb_find_mode(&info->var, info,
				mode_option[pdev->id], NULL, 0, NULL, 16);
		}
	}

	fb_var_to_videomode(&priv->current_mode, &info->var);

	info->var.xres_virtual = info->var.xres;
	info->var.yres_virtual = info->var.yres * 2; /* for double buffer */

	error = rcar_du_check_var(&info->var, info);
	if (error)
		goto err1;

	info->fix = rcar_du_fix;
	info->fix.line_length = info->var.xres *
			(info->var.bits_per_pixel / 8);
	info->fix.ypanstep = info->var.yres;
	info->fix.smem_len = MAX_XRES * MAX_YRES * FB_BYTES * PLANE_NUM;

	buf = dma_alloc_coherent(&pdev->dev, info->fix.smem_len,
				 &priv->dma_handle, GFP_KERNEL);
	if (!buf) {
		dev_err(&pdev->dev, "unable to allocate buffer\n");
		error = -ENOMEM;
		goto err1;
	}

	info->pseudo_palette = &priv->pseudo_palette;
	info->flags = FBINFO_FLAG_DEFAULT;

	error = fb_alloc_cmap(&info->cmap, PALETTE_NR, 0);
	if (error < 0) {
		dev_err(&pdev->dev, "unable to allocate cmap\n");
		dma_free_coherent(&pdev->dev, info->fix.smem_len,
				  buf, priv->dma_handle);
		goto err1;
	}

	memset(buf, 0, info->fix.smem_len);
	info->fix.smem_start = priv->dma_handle;
	info->screen_base = buf;
	info->device = &pdev->dev;
	info->par = priv;

	if (error)
		goto err1;

	error = rcar_du_start(priv);
	if (error) {
		dev_err(&pdev->dev, "unable to start hardware\n");
		goto err1;
	}

	error = register_framebuffer(info);
	if (error < 0)
		goto err1;

	dev_info(info->dev,
		 "registered %s/%s as %dx%d %dbpp.\n",
		 pdev->name,
		 "display unit",
		 (int) info->var.xres,
		 (int) info->var.yres,
		 info->var.bits_per_pixel);

	return 0;
err1:
	rcar_du_remove(pdev);

	return error;
}

static int rcar_du_remove(struct platform_device *pdev)
{
	struct rcar_du_priv *priv = platform_get_drvdata(pdev);
	struct fb_info *info = priv->info;

	unregister_framebuffer(info);

	/* Interrupt VBK Disable */
	du_write_ch(priv, DIER, 0);

	rcar_du_stop(priv);

	if (info->screen_base) {
		dma_free_coherent(&pdev->dev, info->fix.smem_len,
				  info->screen_base, priv->dma_handle);
	}
	fb_dealloc_cmap(&info->cmap);
	framebuffer_release(info);

	if (priv->base)
		iounmap(priv->base);

	if (priv->irq)
		free_irq(priv->irq, priv);
	kfree(priv);
	return 0;
}

static struct platform_driver rcar_du_driver = {
	.driver		= {
		.name		= DEV_NAME,
		.owner		= THIS_MODULE,
	},
	.probe		= rcar_du_probe,
	.remove		= rcar_du_remove,
};

module_platform_driver(rcar_du_driver);

MODULE_DESCRIPTION("R-Car Display Unit Framebuffer driver");
MODULE_LICENSE("GPL v2");
