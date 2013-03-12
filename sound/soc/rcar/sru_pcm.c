/*
 *  sound/soc/rcar/sru_pcm.c
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

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/export.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <sound/soc.h>
#include <sound/hwdep.h>
#include <sound/rcar.h>

#include "sru_pcm.h"

#undef DEBUG
#ifdef DEBUG
#define FNC_ENTRY	pr_info("entry:%s\n", __func__);
#define FNC_EXIT	pr_info("exit:%s:%d\n", __func__, __LINE__);
#define DBG_CHK_POINT()	pr_info("check:%s:%d\n", __func__, __LINE__);
#define DBG_CHK_MSG(args...)	pr_info(args)
#else  /* DEBUG */
#define FNC_ENTRY
#define FNC_EXIT
#define DBG_CHK_POINT()
#define DBG_CHK_MSG(args...)
#endif /* DEBUG */

static struct rcar_audio_info audioinfo;
static spinlock_t sru_lock;	/* for sru common register  */
static struct clk *ssi_clk[NR_SSI_CHANS];
static struct clk *sru_clk;

static void __iomem *adg_io;

static u64 dma_mask = DMA_BIT_MASK(32);

/************************************************************************

	basic function

************************************************************************/
static void sru_or_writel(u32 data, u32 *reg)
{
	u32 val;

	spin_lock(&sru_lock);
	val = readl(reg);
	writel((val | data), reg);
	spin_unlock(&sru_lock);
}

static void sru_writel(u32 data, u32 mask, u32 *reg)
{
	u32 val;

	spin_lock(&sru_lock);
	val = readl(reg);
	writel(((val & mask) | data), reg);
	spin_unlock(&sru_lock);
}

#define ADG_AUDIO_CLKA		0
#define ADG_AUDIO_CLKB		1
#define ADG_AUDIO_CLKS1		2
#define ADG_AUDIO_CLKC		4
#define ADG_AUDIO_FIXED		5
static void adg_setup_brgx(void __iomem *adg_io,
	int brga_clk_source, int brgb_clk_source, int use_bgrb)
{
	void __iomem *ssickr = adg_io + ADG_SSICKR;

	sru_writel((use_bgrb << 31) |
		(brga_clk_source << 20) |
		(brgb_clk_source << 16),
		~0,
		(u32 *)ssickr);
}

#define ADG_DIVCLK_SEL_CLK_FIXED	0
#define ADG_DIVCLK_SEL_ACLKA		1
#define ADG_DIVCLK_SEL_ACLKB		2
#define ADG_DIVCLK_SEL_ACLKC		3
#define ADG_DIVCLK_SEL_MLBCLK		4
#define ADG_ACLK_SEL_BRGA		(1 << 4)
#define ADG_ACLK_SEL_BRGB		(2 << 4)
static void adg_select_ssi_clk(void __iomem *adg_io, int ssi_chan,
	int clk_source, int denominator)
{
	void __iomem *audio_clk_sel = adg_io + ADG_AUDIO_CLK_SEL0;
	int shift = (ssi_chan & 3) << 3;
	int reg_offset = ssi_chan & ~3;
	int divsel = ilog2(denominator) << 6;

	sru_writel((divsel | clk_source) << shift,
		~(0xff << shift),
		(u32 *)(audio_clk_sel + reg_offset));
}

static int adg_init(void)
{
	FNC_ENTRY

	/* BRGA/BRGB clock select */
	/* NOTE: BRGA/BRGB are currently not used */
	adg_setup_brgx(adg_io, ADG_AUDIO_CLKA, ADG_AUDIO_CLKA, 0);

	/* SSI0/SSI1 clock setting */
	adg_select_ssi_clk(adg_io, 0, ADG_DIVCLK_SEL_ACLKA, 1);
	adg_select_ssi_clk(adg_io, 1, ADG_DIVCLK_SEL_ACLKA, 1);

	/* SSI7/SSI8 (shared) clock setting */
	adg_select_ssi_clk(adg_io, 7, ADG_DIVCLK_SEL_ACLKB, 1);

	FNC_EXIT
	return 0;
}

static int sru_ssi_init(void)
{
	FNC_ENTRY

	/* SSI0/SSI1 setting for slave */
	writel(SSICR_P4643_ST, &audioinfo.ssireg[PLAY]->cr);
	writel(SSIWS_ST, &audioinfo.ssireg[PLAY]->wsr);
	writel(SSICR_C4643_ST, &audioinfo.ssireg[CAPT]->cr);

	/* SSI7/SSI8 */
	writel(SSICR_P4643_ST, &audioinfo.ssireg[7]->cr);
	writel(SSIWS_ST, &audioinfo.ssireg[7]->wsr);
	writel(SSICR_C4643_ST, &audioinfo.ssireg[8]->cr);

	FNC_EXIT
	return 0;
}

static void sru_ssi_start_n(int ssi_chan)
{
	u32 val;

	val = readl(&audioinfo.ssireg[ssi_chan]->cr);
	val |= (SSICR_DMEN | SSICR_UIEN | SSICR_UIEN | SSICR_ENABLE);
	writel(val, &audioinfo.ssireg[ssi_chan]->cr);
}

static void sru_ssi_stop_n(int ssi_chan)
{
	u32 val;

	val = readl(&audioinfo.ssireg[ssi_chan]->cr);
	val &= ~(SSICR_DMEN | SSICR_UIEN | SSICR_UIEN | SSICR_ENABLE);
	writel(val, &audioinfo.ssireg[ssi_chan]->cr);
}

static void sru_ssi_start(struct snd_pcm_substream *substream)
{
	struct rcar_pcm_info *rcar_priv = substream->runtime->private_data;
	int dir = substream->stream == SNDRV_PCM_STREAM_CAPTURE;

	if (rcar_priv->id == 0)
		/* SSI0/1 */
		sru_ssi_start_n(dir);
	else
		/* SSI7/8 */
		sru_ssi_start_n(dir+7);
}

static void sru_ssi_stop(struct snd_pcm_substream *substream)
{
	struct rcar_pcm_info *rcar_priv = substream->runtime->private_data;
	int dir = substream->stream == SNDRV_PCM_STREAM_CAPTURE;

	if (rcar_priv->id == 0)
		/* SSI0/1 */
		sru_ssi_stop_n(dir);
	else
		/* SSI7/8 */
		sru_ssi_stop_n(dir+7);
}

static int sru_init(void)
{
	FNC_ENTRY

	/* ADG setting */
	adg_init();

	/* SSI setting */
	sru_ssi_init();

	/* SSI_MODE0 setting (SSI independant) */
	sru_or_writel((SSI_MODE0_IND(0) | SSI_MODE0_IND(1) |
			SSI_MODE0_IND(7) | SSI_MODE0_IND(8)),
			&audioinfo.srureg->ssi_mode0);

	/* SSI_MODE1 setting */
	sru_or_writel(SSI_MODE1_SSI1_MASTER,
			&audioinfo.srureg->ssi_mode1);

	FNC_EXIT
	return 0;
}

static bool sru_dmae_filter(struct dma_chan *chan, void *slave)
{
	struct hpb_dmae_slave *param = slave;

	pr_debug("%s: slave ID %d\n", __func__, param->slave_id);

	chan->private = param;

	return true;
}

/* get dma slave id */
static int sru_dmae_slave_id(struct snd_pcm_substream *substream)
{
	struct rcar_pcm_info *rcar_priv = substream->runtime->private_data;
	int dir = substream->stream == SNDRV_PCM_STREAM_CAPTURE;
	int sid;		/* dma slave id */

	if (rcar_priv->id == 0) {
		/* SSI0/1 */
		if (!dir) /* playback */
			sid = HPBDMA_SLAVE_SSI0_TX_ST;
		else /* capture */
			sid = HPBDMA_SLAVE_SSI1_RX_ST;
	} else {
		/* SSI7/8 */
		if (!dir) /* playback */
			sid = HPBDMA_SLAVE_SSI7_TX_ST;
		else /* capture */
			sid = HPBDMA_SLAVE_SSI8_RX_ST;
	}

	return sid;
}

static void sru_dma_callback(struct snd_pcm_substream *substream)
{
	struct rcar_pcm_info *pcminfo = substream->runtime->private_data;

	FNC_ENTRY

	pcminfo->tran_period++;

	/* Notify alsa: a period is done */
	snd_pcm_period_elapsed(substream);

	/* stop dma */
	if (pcminfo->de_start == 0)
		return;

	/* call sru_io_tasklet() with tasklet */
	tasklet_schedule(&pcminfo->de_tasklet);

	FNC_EXIT
}

static int sru_dmae_start(struct snd_pcm_substream *substream)
{
	int dir = substream->stream == SNDRV_PCM_STREAM_CAPTURE;
	int pingpong;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct rcar_pcm_info *pcminfo = runtime->private_data;
	struct device *dev = substream->pcm->card->dev;
	struct dma_async_tx_descriptor *desc;
	struct scatterlist sg;
	dma_cookie_t cookie;
	u32 dma_size;
	u32 dma_paddr;

	pr_debug("%s: done period #%d (%u/%u bytes), cookie %d\n",
		__func__, pcminfo->period,
		(u_int)(pcminfo->period * runtime->period_size * 4),
		(u_int)(runtime->buffer_size * 4), pcminfo->de_cookie);

	/* Ping-Pong control */
	pingpong = pcminfo->period & (PERIODS_MAX - 1);

	/* DMA size */
	dma_size = frames_to_bytes(runtime, runtime->period_size);

	/* DMA physical adddress */
	dma_paddr = runtime->dma_addr + (pingpong * dma_size);

	sg_init_table(&sg, 1);
	sg_set_page(&sg, pfn_to_page(PFN_DOWN(dma_paddr)),
		    dma_size, offset_in_page(dma_paddr));
	sg_dma_address(&sg) = dma_paddr;

	desc = pcminfo->de_chan->device->device_prep_slave_sg(
		pcminfo->de_chan,
		&sg, 1, DMA_DIR(dir), (DMA_PREP_INTERRUPT | DMA_CTRL_ACK),
		NULL);

	if (!desc) {
		dev_err(dev, "Failed to allocate a dma descriptor\n");
		return -ENOMEM;
	}

	desc->callback = (dma_async_tx_callback)sru_dma_callback;
	desc->callback_param = substream;
	cookie = desc->tx_submit(desc);

	if (cookie < 0) {
		dev_err(dev, "Failed to submit a dma transfer\n");
		return cookie;
	}

	pcminfo->de_desc = desc;
	pcminfo->de_cookie = cookie;
	dma_async_issue_pending(pcminfo->de_chan);

	/* Update period */
	pcminfo->period++;

	return 0;
}

static void sru_io_tasklet(unsigned long data)
{
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *)data;
	struct rcar_pcm_info *pcminfo = substream->runtime->private_data;

	FNC_ENTRY
	if (pcminfo->de_first == 1) {
		/* 1st dma setting */
		sru_dmae_start(substream);
		sru_ssi_start(substream);
		pcminfo->de_first = 0;
	}
	/* next dma setting */
	sru_dmae_start(substream);

	FNC_EXIT
}

static int sru_audio_start(struct snd_pcm_substream *substream)
{
	struct rcar_pcm_info *pcminfo = substream->runtime->private_data;
	int ret = 0;

	FNC_ENTRY

	/* start dma */
	pcminfo->de_start = 1;

	/* request dma first time */
	pcminfo->de_first = 1;
	tasklet_schedule(&pcminfo->de_tasklet);

	FNC_EXIT
	return ret;
}

static int sru_audio_stop(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct rcar_pcm_info *pcminfo = substream->runtime->private_data;

	FNC_ENTRY
	/* stop dma */
	pcminfo->de_start = 0;

	/* src, ssi disable */
	sru_ssi_stop(substream);

	FNC_EXIT
	return ret;
}

static struct rcar_pcm_info *sru_pcm_new_stream(void)
{
	struct rcar_pcm_info *pcminfo;

	FNC_ENTRY
	/* allocate rcar_pcm_info structure */
	pcminfo = kzalloc(sizeof(struct rcar_pcm_info), GFP_KERNEL);
	if (!pcminfo)
		return pcminfo;

	/* initialize rcar_pcm_info structure */
	pcminfo->period      = 0;
	pcminfo->tran_period = 0;
	pcminfo->ainfo       = &audioinfo;
	pcminfo->de_chan     = NULL;
	spin_lock_init(&pcminfo->pcm_lock);

	FNC_EXIT
	return pcminfo;
}

static void sru_pcm_free_stream(struct snd_pcm_runtime *runtime)
{
	struct rcar_pcm_info *pcminfo = runtime->private_data;

	FNC_ENTRY

	/* post process */
	tasklet_kill(&pcminfo->de_tasklet);
	kfree(runtime->private_data);	/* free pcminfo structure */

	FNC_EXIT
	return;
}


/************************************************************************

	dai ops

************************************************************************/
static int sru_dai_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct rcar_pcm_info *pcminfo;
	int ret = 0;

	FNC_ENTRY
	pcminfo = sru_pcm_new_stream();
	if (pcminfo == NULL)
		return -ENOMEM;

	pcminfo->id = dai->id;
	runtime->private_data = pcminfo;
	runtime->private_free = sru_pcm_free_stream;

	tasklet_init(&pcminfo->de_tasklet, sru_io_tasklet,
			(unsigned long)substream);

	FNC_EXIT
	return ret;
}

static void sru_dai_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	FNC_ENTRY
	FNC_EXIT
	return;
}

static int sru_dai_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct rcar_pcm_info *pcminfo = runtime->private_data;
	int ret = 0;

	spin_lock(&pcminfo->pcm_lock);

	FNC_ENTRY
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		ret = sru_audio_start(substream);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		ret = sru_audio_stop(substream);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	spin_unlock(&pcminfo->pcm_lock);

	FNC_EXIT
	return ret;
}

static struct snd_soc_dai_ops sru_dai_ops = {
	.startup	= sru_dai_startup,
	.shutdown	= sru_dai_shutdown,
	.trigger	= sru_dai_trigger,
};

/************************************************************************

	pcm ops

************************************************************************/
static struct snd_pcm_hardware sru_pcm_hardware = {
	.info			= (SNDRV_PCM_INFO_INTERLEAVED	|
				   SNDRV_PCM_INFO_MMAP		|
				   SNDRV_PCM_INFO_MMAP_VALID	|
				   SNDRV_PCM_INFO_PAUSE),
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.rates			= SNDRV_PCM_RATE_8000_48000,
	.rate_min		= 8000,
	.rate_max		= 48000,
	.channels_min		= 2,
	.channels_max		= 9,
	.buffer_bytes_max	= BUFFER_BYTES_MAX,
	.period_bytes_min	= PERIOD_BYTES_MIN,
	.period_bytes_max	= PERIOD_BYTES_MAX,
	.periods_min		= PERIODS_MIN,
	.periods_max		= PERIODS_MAX,
};

static int sru_pcm_open(struct snd_pcm_substream *substream)
{
	struct rcar_pcm_info *pcminfo = substream->runtime->private_data;
	struct hpb_dmae_slave *param = &pcminfo->de_param;
	dma_cap_mask_t mask;
	int ret = 0;

	FNC_ENTRY
	ret = snd_soc_set_runtime_hwparams(substream, &sru_pcm_hardware);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	/* get dma slave id */
	param->slave_id = sru_dmae_slave_id(substream);

	/* request dma channel */
	if (pcminfo->de_chan == NULL) {
		pcminfo->de_chan =
		    dma_request_channel(mask, sru_dmae_filter, param);
		if (!pcminfo->de_chan) {
			printk(KERN_ERR "DMA channel request error\n");
			ret = -EBUSY;
		}
	}
	FNC_EXIT
	return ret;
}

static int sru_pcm_close(struct snd_pcm_substream *substream)
{
	struct rcar_pcm_info *pcminfo = substream->runtime->private_data;

	FNC_ENTRY
	if (pcminfo->de_chan) {
		dma_release_channel(pcminfo->de_chan);
		pcminfo->de_chan = NULL;
	}
	FNC_EXIT
	return 0;
}

static int sru_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *hw_params)
{
	FNC_ENTRY
	FNC_EXIT
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

static int sru_hw_free(struct snd_pcm_substream *substream)
{
	FNC_ENTRY
	FNC_EXIT
	return snd_pcm_lib_free_pages(substream);
}

static snd_pcm_uframes_t sru_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct rcar_pcm_info *pcminfo = runtime->private_data;
	snd_pcm_uframes_t position = 0;

	position = runtime->period_size *
			(pcminfo->tran_period & (PERIODS_MAX - 1));

	DBG_CHK_MSG("\tposition = %d\n", (u32)position);

	return position;
}

static struct snd_pcm_ops sru_pcm_ops = {
	.open		= sru_pcm_open,
	.close		= sru_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= sru_hw_params,
	.hw_free	= sru_hw_free,
	.pointer	= sru_pointer,
};

/************************************************************************

	snd_soc_platform

************************************************************************/
static int sru_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;

	struct snd_card *card = rtd->card->snd_card;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &dma_mask;

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	FNC_ENTRY
	ret = snd_pcm_lib_preallocate_pages_for_all(
		rtd->pcm,
		SNDRV_DMA_TYPE_DEV,
		rtd->card->snd_card->dev,
		BUFFER_BYTES_MAX, BUFFER_BYTES_MAX);

	FNC_EXIT
	return ret;
}

static void sru_pcm_free(struct snd_pcm *pcm)
{
	FNC_ENTRY

	/* free dma buffer */
	snd_pcm_lib_preallocate_free_for_all(pcm);

	FNC_EXIT
}

/************************************************************************


		alsa struct


************************************************************************/
struct snd_soc_dai_driver sru_soc_dai[] = {
	{
		.name			= "rcar_sru_codec1",
		.id			= 0,
		.playback = {
			.rates		= SNDRV_PCM_RATE_8000_48000,
			.formats	= SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min	= 2,
			.channels_max	= 2,
		},
		.capture = {
			.rates		= SNDRV_PCM_RATE_8000_48000,
			.formats	= SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min	= 2,
			.channels_max	= 2,
		},
		.ops = &sru_dai_ops,
	},
	{
		.name			= "rcar_sru_codec2",
		.id			= 1,
		.playback = {
			.rates		= SNDRV_PCM_RATE_8000_48000,
			.formats	= SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min	= 2,
			.channels_max	= 2,
		},
		.capture = {
			.rates		= SNDRV_PCM_RATE_8000_48000,
			.formats	= SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min	= 2,
			.channels_max	= 2,
		},
		.ops = &sru_dai_ops,
	},
};

static struct snd_soc_platform_driver sru_soc_platform = {
	.pcm_new	= sru_pcm_new,
	.pcm_free	= sru_pcm_free,
	.ops	= &sru_pcm_ops,
};
EXPORT_SYMBOL_GPL(sru_soc_platform);

/************************************************************************


		platform function


************************************************************************/
static int sru_probe(struct platform_device *pdev)
{
	int ret = 0;
	int ssi_chan;
	struct resource *sru_res;
	struct resource *sru_region;
	struct resource *adg_res;
	struct resource *adg_region = NULL;
	void __iomem *mem;

	FNC_ENTRY
	if (pdev->id != 0) {
		dev_err(&pdev->dev, "current sru support id 0 only now\n");
		return -ENODEV;
	}

	/* request clocks */
	for (ssi_chan = 0; ssi_chan < NR_SSI_CHANS; ssi_chan++) {
		char clk_name[] = "ssi?";
		clk_name[3] = '0' + ssi_chan;
		ssi_clk[ssi_chan] = clk_get(&pdev->dev, clk_name);
		if (IS_ERR(ssi_clk[ssi_chan])) {
			dev_err(&pdev->dev, "Unable to get ssi%d clock\n", ssi_chan);
			ret = PTR_ERR(ssi_clk[ssi_chan]);
			goto error_clk_put;
		}
	}

	sru_clk = clk_get(&pdev->dev, "sru");
	if (IS_ERR(sru_clk)) {
		dev_err(&pdev->dev, "Unable to get sru clock\n");
		ret = PTR_ERR(sru_clk);
		goto error_clk_put;
	}

	sru_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!sru_res) {
		dev_err(&pdev->dev, "No memory (0) resource\n");
		ret = -ENODEV;
		goto error_clk_put;
	}

	adg_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!sru_res) {
		dev_err(&pdev->dev, "No memory (1) resource\n");
		ret = -ENODEV;
		goto error_clk_put;
	}

	sru_region = request_mem_region(sru_res->start, resource_size(sru_res),
			pdev->name);
	if (!sru_region) {
		dev_err(&pdev->dev, "SRU region already claimed\n");
		ret = -EBUSY;
		goto error_release;
	}

	adg_region = request_mem_region(adg_res->start, resource_size(adg_res),
			pdev->name);
	if (!adg_region) {
		dev_err(&pdev->dev, "ADG region already claimed\n");
		ret = -EBUSY;
		goto error_release;
	}

	mem = ioremap_nocache(sru_res->start, resource_size(sru_res));
	if (!mem) {
		dev_err(&pdev->dev, "ioremap failed for sru\n");
		ret = -ENOMEM;
		goto error_unmap;
	}

	adg_io = ioremap_nocache(adg_res->start, resource_size(adg_res));
	if (!adg_io) {
		dev_err(&pdev->dev, "ioremap failed for adg\n");
		ret = -ENOMEM;
		goto error_unmap;
	}

	for (ssi_chan = 0; ssi_chan < NR_SSI_CHANS; ssi_chan++) {
		clk_enable(ssi_clk[ssi_chan]);
	}
	clk_enable(sru_clk);

	audioinfo.srureg = (struct sru_regs *)mem;

	/* Map SSI registers */
	for (ssi_chan = 0; ssi_chan < NR_SSI_CHANS; ssi_chan++) {
		audioinfo.ssireg[ssi_chan] = (struct ssi_regs *)(mem + SSI_BASE(ssi_chan));
	}

	ret = snd_soc_register_platform(&pdev->dev, &sru_soc_platform);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot snd soc register\n");
		goto error_unmap;
	}

	ret = snd_soc_register_dais(&pdev->dev, sru_soc_dai,
		ARRAY_SIZE(sru_soc_dai));
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot snd soc dais register\n");
		goto error_unregister;
	}

	sru_init();

	FNC_EXIT
	return ret;

error_unregister:
	snd_soc_unregister_platform(&pdev->dev);

error_unmap:
	if (audioinfo.srureg)
		iounmap(audioinfo.srureg);
	if (adg_io)
		iounmap(adg_io);


error_release:
	if (sru_region)
		release_mem_region(sru_res->start, resource_size(sru_res));
	if (adg_region)
		release_mem_region(adg_res->start, resource_size(adg_res));

error_clk_put:
	if (!IS_ERR(sru_clk))
		clk_put(sru_clk);
	for (ssi_chan = 0; ssi_chan < NR_SSI_CHANS; ssi_chan++) {
		if (!IS_ERR(ssi_clk[ssi_chan]))
			clk_put(ssi_clk[ssi_chan]);
	}

	return ret;
}

static int sru_remove(struct platform_device *pdev)
{
	int ssi_chan;

	FNC_ENTRY

	snd_soc_unregister_dais(&pdev->dev, ARRAY_SIZE(sru_soc_dai));
	snd_soc_unregister_platform(&pdev->dev);

	for (ssi_chan = 0; ssi_chan < NR_SSI_CHANS; ssi_chan++) {
		clk_disable(ssi_clk[ssi_chan]);
	}
	clk_disable(sru_clk);

	FNC_EXIT
	return 0;
}

static struct platform_driver sru_driver = {
	.driver		= {
		.name	= "rcar_sru",
	},
	.probe		= sru_probe,
	.remove		= sru_remove,
};

static int __init sru_modinit(void)
{
	FNC_ENTRY

	spin_lock_init(&sru_lock);

	FNC_EXIT
	return platform_driver_register(&sru_driver);
}
module_init(sru_modinit);

static void __exit sru_modexit(void)
{
	FNC_ENTRY

	/* un-regist audio driver */
	platform_driver_unregister(&sru_driver);

	FNC_EXIT
}
module_exit(sru_modexit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("R-Car SRU driver");
MODULE_AUTHOR("Renesas Electronics");
