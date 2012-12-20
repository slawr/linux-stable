/*
 * sound/soc/rcar/rcar_ak4648.c
 *
 * Copyright (C) 2013 Renesas Electronics Europe
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

static int rcar_hurricane_pcm_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret = 0;

	/* set PLL clock */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 11289600, 0);
	if (ret) {
		pr_err("snd_soc_dai_set_sysclk err=%d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_CBS_CFS |
					 SND_SOC_DAIFMT_I2S);
	if (ret)
		pr_err("snd_soc_dai_set_fmt err=%d\n", ret);

	return ret;
}

static struct snd_soc_ops rcar_hurricane_pcm_ops = {
	.hw_params = rcar_hurricane_pcm_hw_params,
};

static struct snd_soc_dai_link rcar_codec_dai1[] = {
	{
		.name		= "AUDIO1",
		.stream_name	= "AUDIO1",
		.cpu_dai_name	= "rcar_sru_codec1",
		.codec_dai_name	= "ak4642-hifi",
		.platform_name	= "rcar_sru.0",
		.codec_name	= "ak4642-codec.0-0012",
		.ops = &rcar_hurricane_pcm_ops,
	},
};

static const struct snd_soc_dapm_widget rcar_ak4648_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("MicIn", NULL),
	SND_SOC_DAPM_LINE("LineIn", NULL),
};

static const struct snd_soc_dapm_route rcar_ak4648_audio_map[] = {
	/* MicIn feeds LIN1/RIN1 */
	{"LIN1", NULL, "MicIn"},
	{"RIN1", NULL, "MicIn"},

	/* Use mic gain & bias */
	{"LIN1", "Mic Gain", "Mic Bias"},
	{"RIN1", "Mic Gain", "Mic Bias"},

	/* LineIn feeds LIN2/RIN2 */
	{"LIN2", NULL, "LineIn"},
	{"RIN2", NULL, "LineIn"},
};

static struct snd_soc_card rcar_codec_soc_card1 = {
	.owner		= THIS_MODULE,
	.name		= "rcar-ak4648",
	.dai_link	= rcar_codec_dai1,
	.num_links	= 1,
	.dapm_widgets	= rcar_ak4648_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rcar_ak4648_dapm_widgets),
	.dapm_routes	= rcar_ak4648_audio_map,
	.num_dapm_routes = ARRAY_SIZE(rcar_ak4648_audio_map),
};

static struct snd_soc_dai_link rcar_codec_dai2[] = {
	{
		.name		= "AUDIO2",
		.stream_name	= "AUDIO2",
		.cpu_dai_name	= "rcar_sru_codec2",
		.codec_dai_name	= "ak4642-hifi",
		.platform_name	= "rcar_sru.0",
		.codec_name	= "ak4642-codec.0-0013",
		.ops = &rcar_hurricane_pcm_ops,
	},
};

static struct snd_soc_card rcar_codec_soc_card2 = {
	.owner		= THIS_MODULE,
	.name		= "rcar-ak4648",
	.dai_link	= rcar_codec_dai2,
	.num_links	= 1,
	.dapm_widgets	= rcar_ak4648_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rcar_ak4648_dapm_widgets),
	.dapm_routes	= rcar_ak4648_audio_map,
	.num_dapm_routes = ARRAY_SIZE(rcar_ak4648_audio_map),
};

/************************************************************************

	AK4648(CODEC#1) machine driver function

************************************************************************/
static int __devinit rcar_probe(struct platform_device *pdev)
{
	int ret;

	rcar_codec_soc_card1.dev = &pdev->dev;
	ret = snd_soc_register_card(&rcar_codec_soc_card1);
	if (ret) {
		pr_err("Unable to register sound card 1\n");
		return ret;
	}

	rcar_codec_soc_card2.dev = &pdev->dev;
	ret = snd_soc_register_card(&rcar_codec_soc_card2);
	if (ret)
		pr_err("Unable to register sound card 2\n");

	return ret;
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
MODULE_DESCRIPTION("R-Car SRU-AK4648 sound card");
MODULE_AUTHOR("Renesas Electronics");
