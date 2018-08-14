/*
 * tegra_t186ref_adau1x61_alt.c - Tegra T186 Machine driver for ADAU1x61
 *
 * Copyright (c) 2018 Verizon Communications, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/platform_data/tegra_asoc_pdata.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "../codecs/adau17x1.h"

#include "tegra_asoc_utils_alt.h"
#include "tegra_asoc_machine_alt.h"
#include "tegra_asoc_machine_alt_t18x.h"

#define DRV_NAME "t186ref-alt-adau1x61"
#define CODEC_NAME "adau-hifi"

struct tegra_t186ref {
	struct tegra_asoc_platform_data *pdata;
	struct tegra_asoc_audio_clock_info audio_clock;
	unsigned int num_codec_links;
	int gpio_requested;
	struct regulator *digital_reg;
	struct regulator *spk_reg;
	struct regulator *dmic_reg;
	struct snd_soc_card *pcard;
	int rate_via_kcontrol;
	int fmt_via_kcontrol;
};

static const int t186ref_adau1x61_srate_values[] = {
	0,
	7350,
	8000,
	11025,
	12000,
	14700,
	16000,
	22050,
	24000,
	29400,
	32000,
	44100,
	48000,
	88200,
	96000,
};

#define PARAMS(sformat, channels)			\
	{						\
		.formats = sformat,			\
		.rate_min = 48000,			\
		.rate_max = 48000,			\
		.channels_min = channels,		\
		.channels_max = channels,		\
	}
static struct snd_soc_pcm_stream tegra_t186ref_asrc_link_params[] = {
	PARAMS(SNDRV_PCM_FMTBIT_S32_LE, 8),
	PARAMS(SNDRV_PCM_FMTBIT_S16_LE, 2),
	PARAMS(SNDRV_PCM_FMTBIT_S16_LE, 2),
	PARAMS(SNDRV_PCM_FMTBIT_S16_LE, 2),
	PARAMS(SNDRV_PCM_FMTBIT_S16_LE, 2),
	PARAMS(SNDRV_PCM_FMTBIT_S16_LE, 2),
};
#undef PARAMS

static int set_cpu_dai_tdm_slot(struct snd_soc_pcm_runtime *rtd)
{
	unsigned int fmt, mask;
	int err = 0;

	fmt = rtd->dai_link->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK;
	mask = (1 << rtd->dai_link->params->channels_min) - 1;

	if ((fmt == SND_SOC_DAIFMT_DSP_A) || (fmt == SND_SOC_DAIFMT_DSP_B))
		err = snd_soc_dai_set_tdm_slot(rtd->cpu_dai, mask, mask, 0, 0);

	return err;
}

static int t186ref_adau1x61_dai_init(struct snd_soc_pcm_runtime *rtd,
					int rate,
					int channels,
					u64 formats)
{
	struct snd_soc_card *card = rtd->card;
	struct tegra_t186ref *machine = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_stream *dai_params;
	u64 format_k;
	unsigned int idx, mclk, clk_out_rate;
	unsigned int bclk_ratio;
	int err, codec_rate, clk_rate;
	int num_of_dai_links = TEGRA186_XBAR_DAI_LINKS + machine->num_codec_links;

	format_k = ((machine->fmt_via_kcontrol == 2) ?
		    SNDRV_PCM_FMTBIT_S32_LE : SNDRV_PCM_FMTBIT_S16_LE);
	dev_info(card->dev,
		 "%s: formats=0x%llx, format_k=0x%llx\n", __func__, formats, format_k);

	codec_rate = t186ref_adau1x61_srate_values[machine->rate_via_kcontrol];
	clk_rate = (machine->rate_via_kcontrol) ? codec_rate : rate;

	/*
	 * The codec driver really wants to use the on-board PLL,
	 * so we set the output frequency to 1024 * f-sub-s.
	 * The PLL's input frequency must be between 8MHz and 27MHz.
	 */
	switch (clk_rate) {
	case 7350:
	case 11025:
	case 14700:
	case 22050:
	case 29400:
	case 44100:
	case 88200:
		clk_out_rate = 44100 * 1024;
		mclk = 44100 * 256;
		break;
	case 8000:
	case 12000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 96000:
	default:
		clk_out_rate = 48000 * 1024;
		mclk = 48000 * 256;
		break;
	}

	err = tegra_alt_asoc_utils_set_rate(&machine->audio_clock,
					    clk_rate, mclk, clk_out_rate);
	if (err < 0) {
		dev_err(card->dev,
			"Can't configure clocks clk_rate %dHz pll_a_out0 %dHz clk_out %dHz\n",
			clk_rate, mclk, clk_out_rate);
		return err;
	}

	/* update dai link hw_params  */
	for (idx = 0; idx < num_of_dai_links; idx++) {
		struct snd_soc_pcm_runtime *rtd = &card->rtd[idx];
		if (rtd->dai_link->params) {
			dai_params =
			  (struct snd_soc_pcm_stream *)
			  card->rtd[idx].dai_link->params;
			dai_params->rate_min = rate;
			dai_params->channels_min = channels;
			dai_params->formats = format_k;

			if (idx >= TEGRA186_XBAR_DAI_LINKS) {
				bclk_ratio = tegra_machine_get_bclk_ratio_t18x(rtd);
				dai_params->formats = formats;
				if (bclk_ratio >= 0) {
					err = snd_soc_dai_set_bclk_ratio(rtd->cpu_dai,
									 bclk_ratio);
					if (err < 0)
						dev_err(card->dev,
							"Failed to set CPU DAI bclk ratio for %s\n",
							rtd->dai_link->name);
				}
				err = set_cpu_dai_tdm_slot(rtd);
				if (err < 0)
					dev_err(card->dev,
						"Failed to set CPU DAI slot mask for %s\n",
						rtd->cpu_dai->name);
			}
		}
	}

	idx = tegra_machine_get_codec_dai_link_idx_t18x(CODEC_NAME);
	if (idx < 0) {
		dev_err(card->dev, "could not get DAI link for " CODEC_NAME " (%d)\n", idx);
		return idx;
	}
	dai_params = (struct snd_soc_pcm_stream *)card->rtd[idx].dai_link->params;
	dai_params->rate_min = clk_rate;
	dai_params->formats = format_k;

	err = snd_soc_dai_set_pll(card->rtd[idx].codec_dai,
				  ADAU17X1_PLL, ADAU17X1_PLL_SRC_MCLK,
				  mclk, clk_out_rate);
	if (err < 0) {
		dev_err(card->dev, "could not set PLL: %d\n", err);
		return err;
	}
	err = snd_soc_dai_set_sysclk(card->rtd[idx].codec_dai,
				     ADAU17X1_CLK_SRC_PLL,
				     clk_out_rate, SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "could not set sysclk: %d\n", err);
		return err;
	}

	err = snd_soc_dai_set_tdm_slot(card->rtd[idx].codec_dai,
				       (1 << channels) - 1, (1 << channels) - 1, 0, 0);
	if (err < 0) {
		dev_err(card->dev, "Can't set codec dai slot ctrl: %d\n", err);
		return err;
	}

	return 0;
}

static int t186ref_adau1x61_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	int err;

	dev_info(card->dev, "setting up %s params\n",
		 (substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
		  "playback" : "capture"));
	err = t186ref_adau1x61_dai_init(rtd, params_rate(params),
			params_channels(params),
			(1ULL << (params_format(params))));
	if (err < 0)
		dev_err(card->dev, "Failed dai init\n");

	return err;
}

#ifdef CONFIG_SND_SOC_TEGRA210_ADSP_ALT
static int t186ref_adau1x61_compr_set_params(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_platform *platform = rtd->platform;
	struct snd_codec codec_params;
	int err;

	if (platform->driver->compr_ops &&
		platform->driver->compr_ops->get_params) {
		err = platform->driver->compr_ops->get_params(cstream,
			&codec_params);
		if (err < 0) {
			dev_err(card->dev, "Failed to get compr params\n");
			return err;
		}
	} else {
		dev_err(card->dev, "compr ops not set\n");
		return -EINVAL;
	}

	err = t186ref_adau1x61_dai_init(rtd, codec_params.sample_rate,
			codec_params.ch_out, SNDRV_PCM_FMTBIT_S16_LE);
	if (err < 0) {
		dev_err(card->dev, "Failed dai init\n");
		return err;
	}

	return 0;
}
static int t186ref_adau1x61_compr_startup(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_t186ref *machine = snd_soc_card_get_drvdata(card);

	tegra_alt_asoc_utils_clk_enable(&machine->audio_clock);

	return 0;
}

static void t186ref_adau1x61_compr_shutdown(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_t186ref *machine = snd_soc_card_get_drvdata(card);

	tegra_alt_asoc_utils_clk_disable(&machine->audio_clock);

	return;
}
#endif /* CONFIG_SND_SOC_TEGRA210_ADSP_ALT */

static int t186ref_adau1x61_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct tegra_t186ref *machine = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_stream *dai_params =
		(struct snd_soc_pcm_stream *)rtd->dai_link->params;
	unsigned int srate;
	int err;

	srate = dai_params->rate_min;

	err = tegra_alt_asoc_utils_set_extern_parent(&machine->audio_clock,
							"pll_a_out0");
	if (err < 0) {
		dev_err(card->dev, "Failed to set extern clk parent\n");
		return err;
	}

	snd_soc_dapm_force_enable_pin(dapm, "x Microphone");

	snd_soc_dapm_sync(&card->dapm);

	return 0;
}

static int t186ref_adau1x61_sfc_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int in_srate, out_srate;
	int err;

	in_srate = 48000;
	out_srate = 8000;

	err = snd_soc_dai_set_sysclk(codec_dai, 0, out_srate,
					SND_SOC_CLOCK_OUT);
	err = snd_soc_dai_set_sysclk(codec_dai, 0, in_srate,
					SND_SOC_CLOCK_IN);

	return err;
}

#ifdef CONFIG_SND_SOC_TEGRA210_ADSP_ALT
static struct snd_soc_compr_ops t186ref_adau1x61_compr_ops = {
	.set_params = t186ref_adau1x61_compr_set_params,
	.startup = t186ref_adau1x61_compr_startup,
	.shutdown = t186ref_adau1x61_compr_shutdown,
};
#endif

static const struct snd_soc_dapm_widget t186ref_adau1x61_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("x Microphone", NULL),
};

static int t186ref_adau1x61_suspend_pre(struct snd_soc_card *card)
{
	unsigned int idx;

	/* DAPM dai link stream work for non pcm links */
	for (idx = 0; idx < card->num_rtd; idx++) {
		if (card->rtd[idx].dai_link->params)
			INIT_DELAYED_WORK(&card->rtd[idx].delayed_work, NULL);
	}

	return 0;
}

static int t186ref_adau1x61_suspend_post(struct snd_soc_card *card)
{
	struct tegra_t186ref *machine = snd_soc_card_get_drvdata(card);

	if (machine->digital_reg)
		regulator_disable(machine->digital_reg);

	return 0;
}

static int t186ref_adau1x61_resume_pre(struct snd_soc_card *card)
{
	struct tegra_t186ref *machine = snd_soc_card_get_drvdata(card);
	int ret;

	if (machine->digital_reg) {
		ret = regulator_enable(machine->digital_reg);
		if (ret < 0)
			dev_err(card->dev, "could not enable regulator: %d\n", ret);
	}

	return 0;
}

static int t186ref_adau1x61_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_t186ref *machine = snd_soc_card_get_drvdata(rtd->card);

	tegra_alt_asoc_utils_clk_enable(&machine->audio_clock);

	return 0;
}

static void t186ref_adau1x61_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_t186ref *machine = snd_soc_card_get_drvdata(rtd->card);

	tegra_alt_asoc_utils_clk_disable(&machine->audio_clock);

	return;
}

static struct snd_soc_ops t186ref_adau1x61_ops = {
	.hw_params = t186ref_adau1x61_hw_params,
	.startup = t186ref_adau1x61_startup,
	.shutdown = t186ref_adau1x61_shutdown,
};

static const char * const t186ref_adau1x61_srate_text[] = {
	"None",
	"7kHz",
	"8kHz",
	"11kHz",
	"12kHz",
	"14kHz",
	"16kHz",
	"22kHz",
	"24kHz",
	"29kHz",
	"32kHz",
	"44kHz",
	"48kHz",
	"88kHz",
	"96kHz",
};

static int t186ref_adau1x61_codec_get_rate(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct tegra_t186ref *machine = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] = machine->rate_via_kcontrol;

	return 0;
}

static int t186ref_adau1x61_codec_put_rate(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct tegra_t186ref *machine = snd_soc_card_get_drvdata(card);

	/* set the rate control flag */
	machine->rate_via_kcontrol = ucontrol->value.integer.value[0];

	return 0;
}

static const char * const t186ref_adau1x61_format_text[] = {
	"None",
	"16",
	"32",
};

static int t186ref_adau1x61_codec_get_format(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct tegra_t186ref *machine = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] = machine->fmt_via_kcontrol;

	return 0;
}

static int t186ref_adau1x61_codec_put_format(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct tegra_t186ref *machine = snd_soc_card_get_drvdata(card);

	/* set the format control flag */
	machine->fmt_via_kcontrol = ucontrol->value.integer.value[0];

	return 0;
}

static const struct soc_enum t186ref_adau1x61_codec_rate =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(t186ref_adau1x61_srate_text),
		t186ref_adau1x61_srate_text);

static const struct soc_enum t186ref_adau1x61_codec_format =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(t186ref_adau1x61_format_text),
		t186ref_adau1x61_format_text);

static const struct snd_kcontrol_new t186ref_adau1x61_controls[] = {
	SOC_ENUM_EXT("codec rate", t186ref_adau1x61_codec_rate,
		t186ref_adau1x61_codec_get_rate, t186ref_adau1x61_codec_put_rate),
	SOC_ENUM_EXT("codec format", t186ref_adau1x61_codec_format,
		t186ref_adau1x61_codec_get_format, t186ref_adau1x61_codec_put_format),
};

static int t186ref_adau1x61_remove(struct snd_soc_card *card)
{
	return 0;
}

static struct snd_soc_card snd_soc_tegra_t186ref = {
	.name = "tegra-t186ref-adau1x61",
	.owner = THIS_MODULE,
	.remove = t186ref_adau1x61_remove,
	.suspend_post = t186ref_adau1x61_suspend_post,
	.suspend_pre = t186ref_adau1x61_suspend_pre,
	.resume_pre = t186ref_adau1x61_resume_pre,
	.controls = t186ref_adau1x61_controls,
	.num_controls = ARRAY_SIZE(t186ref_adau1x61_controls),
	.dapm_widgets = t186ref_adau1x61_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(t186ref_adau1x61_dapm_widgets),
	.fully_routed = true,
};

static void dai_link_setup(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_t186ref *machine = snd_soc_card_get_drvdata(card);
	struct snd_soc_codec_conf *tegra_machine_codec_conf = NULL;
	struct snd_soc_codec_conf *t186ref_adau1x61_codec_conf = NULL;
	struct snd_soc_dai_link *tegra_machine_dai_links = NULL;
	struct snd_soc_dai_link *t186ref_adau1x61_codec_links = NULL;
	int i;

	/* set new codec links and conf */
	t186ref_adau1x61_codec_links = tegra_machine_new_codec_links(pdev,
		t186ref_adau1x61_codec_links,
		&machine->num_codec_links);
	if (!t186ref_adau1x61_codec_links) {
		dev_err(&pdev->dev, "%s: could not set machine codec links\n", __func__);
		goto err_alloc_dai_link;
	}

	/* set codec init */
	for (i = 0; i < machine->num_codec_links; i++) {
		if (t186ref_adau1x61_codec_links[i].name) {
			if (strstr(t186ref_adau1x61_codec_links[i].name,
				   CODEC_NAME)) {
				t186ref_adau1x61_codec_links[i].init =
					t186ref_adau1x61_init;
			}
		}
	}

	t186ref_adau1x61_codec_conf = tegra_machine_new_codec_conf(pdev,
		t186ref_adau1x61_codec_conf,
		&machine->num_codec_links);
	if (!t186ref_adau1x61_codec_conf) {
		dev_err(&pdev->dev, "%s: could not set new codec configuration\n", __func__);
		goto err_alloc_dai_link;
	}

	/* get the xbar dai link/codec conf structure */
	tegra_machine_dai_links = tegra_machine_get_dai_link_t18x();
	if (!tegra_machine_dai_links) {
		dev_err(&pdev->dev, "%s: could not get machine links for xbar setup\n", __func__);
		goto err_alloc_dai_link;
	}
	tegra_machine_codec_conf = tegra_machine_get_codec_conf_t18x();
	if (!tegra_machine_codec_conf) {
		dev_err(&pdev->dev, "%s: could not get codec config for xbar setup\n", __func__);
		goto err_alloc_dai_link;
	}

	/* set ADMAIF dai_ops */
	for (i = TEGRA186_DAI_LINK_ADMAIF1;
		i <= TEGRA186_DAI_LINK_ADMAIF20; i++)
		tegra_machine_set_dai_ops(i, &t186ref_adau1x61_ops);

	/* set sfc dai_init */
	tegra_machine_set_dai_init(TEGRA186_DAI_LINK_SFC1_RX,
		&t186ref_adau1x61_sfc_init);

#ifdef CONFIG_SND_SOC_TEGRA210_ADSP_ALT
	/* set ADSP PCM */
	for (i = TEGRA186_DAI_LINK_ADSP_PCM1;
		i <= TEGRA186_DAI_LINK_ADSP_PCM2; i++) {
		tegra_machine_set_dai_ops(i,
			&t186ref_adau1x61_ops);
	}

	/* set ADSP COMPR */
	for (i = TEGRA186_DAI_LINK_ADSP_COMPR1;
		i <= TEGRA186_DAI_LINK_ADSP_COMPR2; i++) {
		tegra_machine_set_dai_compr_ops(i,
			&t186ref_adau1x61_compr_ops);
	}
#endif /* CONFIG_SND_SOC_TEGRA210_ADSP_ALT */

	/* set ASRC params. The default is 2 channels */
	for (i = 0; i < ARRAY_SIZE(tegra_t186ref_asrc_link_params); i++) {
		tegra_machine_set_dai_params(TEGRA186_DAI_LINK_ASRC1_TX1 + i,
			(struct snd_soc_pcm_stream *)
				&tegra_t186ref_asrc_link_params[i]);
		tegra_machine_set_dai_params(TEGRA186_DAI_LINK_ASRC1_RX1 + i,
			(struct snd_soc_pcm_stream *)
				&tegra_t186ref_asrc_link_params[i]);
	}

	/* append t186ref specific dai_links */
	card->num_links =
		tegra_machine_append_dai_link_t18x(t186ref_adau1x61_codec_links,
			2 * machine->num_codec_links);
	tegra_machine_dai_links = tegra_machine_get_dai_link_t18x();
	card->dai_link = tegra_machine_dai_links;

	/* append t186ref specific codec_conf */
	card->num_configs =
		tegra_machine_append_codec_conf_t18x(t186ref_adau1x61_codec_conf,
			machine->num_codec_links);
	tegra_machine_codec_conf = tegra_machine_get_codec_conf_t18x();
	card->codec_conf = tegra_machine_codec_conf;

	return;

err_alloc_dai_link:
	tegra_machine_remove_dai_link();
	tegra_machine_remove_codec_conf();
}

static int t186ref_adau1x61_driver_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &snd_soc_tegra_t186ref;
	struct tegra_t186ref *machine;
	struct tegra_asoc_platform_data *pdata = NULL;
	struct snd_soc_codec *codec = NULL;
	int ret = 0, idx;
	const char *codec_dai_name;

	if (!np) {
		dev_err(&pdev->dev, "No device tree node for t186ref adau1x61 driver");
		return -ENODEV;
	}

	machine = devm_kzalloc(&pdev->dev, sizeof(struct tegra_t186ref),
			       GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_t186ref struct\n");
		ret = -ENOMEM;
		goto err;
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

	ret = snd_soc_of_parse_card_name(card, "nvidia,model");
	if (ret)
		goto err;

	ret = snd_soc_of_parse_audio_routing(card,
				"nvidia,audio-routing");
	if (ret)
		goto err;


	if (of_property_read_u32(np, "nvidia,num-clk",
			       &machine->audio_clock.num_clk) < 0) {
		dev_err(&pdev->dev,
			"Missing property nvidia,num-clk\n");
		ret = -ENODEV;
		goto err;
	}

	if (of_property_read_u32_array(np, "nvidia,clk-rates",
				(u32 *)&machine->audio_clock.clk_rates,
				machine->audio_clock.num_clk) < 0) {
		dev_err(&pdev->dev,
			"Missing property nvidia,clk-rates\n");
		ret = -ENODEV;
		goto err;
	}

	dai_link_setup(pdev);

	pdata = devm_kzalloc(&pdev->dev,
				sizeof(struct tegra_asoc_platform_data),
				GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev,
			"Can't allocate tegra_asoc_platform_data struct\n");
		return -ENOMEM;
	}

	pdata->gpio_codec1 = pdata->gpio_codec2 = pdata->gpio_codec3 =
	pdata->gpio_spkr_en = pdata->gpio_hp_mute =
	pdata->gpio_hp_det = pdata->gpio_hp_det_active_high =
	pdata->gpio_int_mic_en = pdata->gpio_ext_mic_en = -1;

	machine->pdata = pdata;
	machine->pcard = card;

	ret = tegra_alt_asoc_utils_init(&machine->audio_clock,
					&pdev->dev,
					card);
	if (ret)
		goto err_alloc_dai_link;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_alloc_dai_link;
	}

	idx = tegra_machine_get_codec_dai_link_idx(CODEC_NAME);
	/* check if idx has valid number */
	if (idx == -EINVAL)
		dev_warn(&pdev->dev, "codec link not defined - codec not part of sound card");
	else {
		codec = card->rtd[idx].codec;
		codec_dai_name = card->rtd[idx].dai_link->codec_dai_name;
	}

	return 0;

err_alloc_dai_link:
	tegra_machine_remove_dai_link();
	tegra_machine_remove_codec_conf();
err:
	return ret;
}

static int t186ref_adau1x61_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_t186ref *machine = snd_soc_card_get_drvdata(card);

	snd_soc_unregister_card(card);

	tegra_machine_remove_dai_link();
	tegra_machine_remove_codec_conf();
	tegra_alt_asoc_utils_fini(&machine->audio_clock);

	return 0;
}

static const struct of_device_id t186ref_adau1x61_of_match[] = {
	{ .compatible = "nvidia,tegra-audio-t186ref-adau1x61", },
	{},
};

static struct platform_driver t186ref_adau1x61_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = t186ref_adau1x61_of_match,
	},
	.probe = t186ref_adau1x61_driver_probe,
	.remove = t186ref_adau1x61_driver_remove,
};
module_platform_driver(t186ref_adau1x61_driver);

MODULE_AUTHOR("Matt Madison <matthew.madison@verizon.com>");
MODULE_DESCRIPTION("Tegra t186ref machine ASoC driver for ADAU1x61");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, t186ref_adau1x61_of_match);
