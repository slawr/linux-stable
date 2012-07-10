/*
 * sound/soc/rcar/rcar_ak4643.c
 *
 * Copyright (C) 2011-2012 Renesas Electronics Corporation
 *
 * This file is based on the sound/soc/sh/fsi-ak4642.c
 *
 * FSI-AK464x sound support for ms7724se
 *
 * Copyright (C) 2009 Renesas Solutions Corp.
 * Kuninori Morimoto <morimoto.kuninori@renesas.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/wait.h>
#include <linux/module.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

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

/************************************************************************

	CODEC#1 machine dependent function

************************************************************************/
static struct snd_soc_card rcar_codec1_soc_card;
int sru_pcm_hwdep_new(struct snd_card *card, char *id);

static int rcar_codec1_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;
	struct snd_soc_dai *codec = rtd->codec_dai;

	FNC_ENTRY

	/* set PLL clock */
	ret = snd_soc_dai_set_sysclk(codec, 0, 11289600, 0);
	if (ret) {
		pr_err("snd_soc_dai_set_sysclk err=%d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(codec, SND_SOC_DAIFMT_CBS_CFS |
					 SND_SOC_DAIFMT_I2S);
	if (ret) {
		pr_err("snd_soc_dai_set_fmt err=%d\n", ret);
		return ret;
	}

	ret = sru_pcm_hwdep_new(rtd->card->snd_card, "sru_pcm0");
	if (ret)
		pr_err("sru_pcm_hwdep_new err=%d\n", ret);

	FNC_EXIT
	return ret;
}

/************************************************************************

	CODEC#1 machine driver ALSA SoC structure

************************************************************************/
static struct snd_soc_dai_link rcar_codec1_dai_link = {
	.name		= "CODEC#1(AK4643)",
	.stream_name	= "CODEC#1(AK4643)",
	.cpu_dai_name	= "rcar_sru_codec1",
	.codec_dai_name	= "ak4642-hifi",
	.platform_name	= "rcar_sru.0",
	.codec_name	= "ak4642-codec.0-0012",
	.init		= rcar_codec1_dai_init,
	.ops		= NULL,
};

static struct snd_soc_card rcar_codec1_soc_card  = {
	.owner		= THIS_MODULE,
	.name		= "RCAR_AK4643(CODEC#1)",
	.dai_link	= &rcar_codec1_dai_link,
	.num_links	= 1,
};

/************************************************************************

	AK4643(CODEC#1) machine driver function

************************************************************************/
static int __devinit rcar_probe(struct platform_device *pdev)
{
	int ret = -ENOMEM;
	FNC_ENTRY

	rcar_codec1_soc_card.dev = &pdev->dev;
	ret = snd_soc_register_card(&rcar_codec1_soc_card);
	if (ret)
		pr_err("Unable to register sourd card\n");

	return ret;
	FNC_EXIT
}

static struct platform_driver rcar_driver = {
	.driver = {
		.name = "rcar_alsa_soc_platform",
		.owner = THIS_MODULE,
	},
	.probe = rcar_probe,
};

module_platform_driver(rcar_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("R-Car SRU-AK4643 sound card");
MODULE_AUTHOR("Renesas Electronics");
