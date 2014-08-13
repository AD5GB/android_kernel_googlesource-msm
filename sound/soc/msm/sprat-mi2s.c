/*
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2013 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/q6afe-v2.h>
#include "qdsp6v2/msm-pcm-routing-v2.h"

#define DRV_NAME "sprat-mi2s"

struct regulator *mic_supply;
static atomic_t mi2s_rsc_ref;

static struct gpio mi2s_gpio[] = {
	{
		.flags = 0,
		.label = "sprat,mic-sck-gpio",
	},
	{
		.flags = 0,
		.label = "sprat,mic-ws-gpio",
	},
	{
		.flags = 0,
		.label = "sprat,mic-din-gpio",
	},
};

static struct gpio mic_en_gpio = {
	.flags = GPIOF_OUT_INIT_LOW,
	.label = "sprat,mic-en-gpio",
};

static struct afe_clk_cfg lpass_mi2s = {
	AFE_API_VERSION_I2S_CONFIG,
	Q6AFE_LPASS_IBIT_CLK_3_P072_MHZ,
	Q6AFE_LPASS_OSR_CLK_12_P288_MHZ,
	Q6AFE_LPASS_CLK_SRC_INTERNAL,
	Q6AFE_LPASS_CLK_ROOT_DEFAULT,
	Q6AFE_LPASS_MODE_BOTH_VALID,
	0,
};

static struct afe_clk_cfg lpass_mi2s_disable = {
	AFE_API_VERSION_I2S_CONFIG,
	Q6AFE_LPASS_IBIT_CLK_DISABLE,
	Q6AFE_LPASS_OSR_CLK_DISABLE,
	Q6AFE_LPASS_CLK_SRC_INTERNAL,
	Q6AFE_LPASS_CLK_ROOT_DEFAULT,
	Q6AFE_LPASS_MODE_BOTH_VALID,
	0,
};

static int sprat_request_gpios(struct platform_device *pdev)
{
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(mi2s_gpio); i++)
		mi2s_gpio[i].gpio = of_get_named_gpio(pdev->dev.of_node,
						mi2s_gpio[i].label, 0);

	ret = gpio_request_array(mi2s_gpio, ARRAY_SIZE(mi2s_gpio));
	if (ret) {
		pr_err("%s: Unable to request gpios(%d)\n", __func__, ret);
		return ret;
	}

	/* Request optional mic enable GPIO */
	mic_en_gpio.gpio = of_get_named_gpio(pdev->dev.of_node,
							mic_en_gpio.label, 0);
	if (gpio_is_valid(mic_en_gpio.gpio)) {
		ret = gpio_request_one(mic_en_gpio.gpio, mic_en_gpio.flags,
							mic_en_gpio.label);
		if (ret) {
			pr_err("%s: Unable to request mic_en_gpio(%d)\n", __func__, ret);
			gpio_free_array(mi2s_gpio, ARRAY_SIZE(mi2s_gpio));

			return ret;
		}
	}

	return 0;
}

static void sprat_regulator_enable(bool enable)
{
	if (enable) {
		if (regulator_enable(mic_supply))
			pr_err("%s: enable mic-supply failed\n", __func__);
	} else {
		regulator_disable(mic_supply);
	}
}

static int sprat_mi2s_startup(struct snd_pcm_substream *substream)
{
	int ret;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	if (atomic_inc_return(&mi2s_rsc_ref) == 1) {
		if(gpio_is_valid(mic_en_gpio.gpio))	{
			gpio_set_value(mic_en_gpio.gpio, 1);
		}
		else	{
			sprat_regulator_enable(true);
		}

		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_CBS_CFS);
		if (ret < 0)
			dev_err(cpu_dai->dev, "set format for CPU dai failed\n");
	}

	return 0;
}

static int sprat_mi2s_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);

	/*
	 * This causes the mic to always run at 48KHz which results
	 * in a smaller warm-up delay. Remove this function to allow
	 * 8, 16 and 48KHz.
	 */
	rate->min = 48000;
	rate->max = 48000;

	return 0;
}

static int sprat_mi2s_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	int ret;

	/*
	 * This I2S mic only supports 24 bit output, which is why the
	 * clocks are never modified for 16 bit use.
	 */
	switch (params_rate(params)) {
	case 8000:
		lpass_mi2s.clk_val1 = Q6AFE_LPASS_IBIT_CLK_512_KHZ;
		lpass_mi2s.clk_val2 = Q6AFE_LPASS_OSR_CLK_2_P048_MHZ;
		break;
	case 16000:
		lpass_mi2s.clk_val1 = Q6AFE_LPASS_IBIT_CLK_1_P024_MHZ;
		lpass_mi2s.clk_val2 = Q6AFE_LPASS_OSR_CLK_4_P096_MHZ;
		break;
	default:
		lpass_mi2s.clk_val1 = Q6AFE_LPASS_IBIT_CLK_3_P072_MHZ;
		lpass_mi2s.clk_val2 = Q6AFE_LPASS_OSR_CLK_12_P288_MHZ;
	}

	ret = afe_set_lpass_clock(AFE_PORT_ID_TERTIARY_MI2S_TX,
							&lpass_mi2s);
	if (ret < 0) {
		pr_err("%s: Unable to enable LPASS clock\n", __func__);
		return ret;
	}

	return 0;
}

static void sprat_mi2s_shutdown(struct snd_pcm_substream *substream)
{
	if (atomic_dec_return(&mi2s_rsc_ref) == 0) {
		if (afe_set_lpass_clock(AFE_PORT_ID_TERTIARY_MI2S_TX, &lpass_mi2s_disable) < 0)
			pr_err("%s: Unable to disable LPASS clock\n", __func__);

		if (gpio_is_valid(mic_en_gpio.gpio))	{
			gpio_set_value(mic_en_gpio.gpio, 0);
	}
		else	{
			sprat_regulator_enable(false);
		}
	}
}

static struct snd_soc_ops sprat_mi2s_be_ops = {
	.startup = sprat_mi2s_startup,
	.hw_params = sprat_mi2s_hw_params,
	.shutdown = sprat_mi2s_shutdown
};

/* Digital audio interface glue - connects codec <---> CPU */
static struct snd_soc_dai_link sprat_dai[] = {
	/* FrontEnd DAI Links */
	{
		.name = "Sprat Media1",
		.stream_name = "MultiMedia1",
		.cpu_dai_name	= "MultiMedia1",
		.platform_name  = "msm-pcm-dsp.0",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		/* This dainlink has playback support */
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA1
	},
	/* Backend DAI Links */
	{
		.name = LPASS_BE_TERT_MI2S_TX,
		.stream_name = "Tertiary MI2S Capture",
		.cpu_dai_name = "msm-dai-q6-mi2s.2",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_TERTIARY_MI2S_TX,
		.be_hw_params_fixup = sprat_mi2s_hw_params_fixup,
		.ops = &sprat_mi2s_be_ops,
		.ignore_suspend = 1,
	},
};

struct snd_soc_card snd_soc_card_sprat = {
	.name		= "sprat-mi2s-snd-card",
	.dai_link	= sprat_dai,
	.num_links	= ARRAY_SIZE(sprat_dai),
};

static int sprat_asoc_machine_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_card_sprat;
	int ret;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No platform supplied from device tree\n");
		return -EINVAL;
	}

	mic_supply = regulator_get(&pdev->dev, "sprat,mic");

	if (IS_ERR(mic_supply)) {
		dev_err(&pdev->dev, "unable to get mic-supply regulator\n");
	}

	ret = sprat_request_gpios(pdev);
	if (ret)
		goto err;

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	atomic_set(&mi2s_rsc_ref, 0);

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err;
	}

	return 0;

err:
	gpio_free_array(mi2s_gpio, ARRAY_SIZE(mi2s_gpio));
	if (gpio_is_valid(mic_en_gpio.gpio))
		gpio_free(mic_en_gpio.gpio);

	return ret;
}

static int sprat_asoc_machine_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	gpio_free_array(mi2s_gpio, ARRAY_SIZE(mi2s_gpio));
	if (gpio_is_valid(mic_en_gpio.gpio))
		gpio_free(mic_en_gpio.gpio);
	else
		regulator_put(mic_supply);
	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id sprat_of_match[]  = {
	{ .compatible = "sprat,sprat-mi2s-audio", },
	{},
};

static struct platform_driver sprat_asoc_machine_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = sprat_of_match,
	},
	.probe = sprat_asoc_machine_probe,
	.remove = sprat_asoc_machine_remove,
};
module_platform_driver(sprat_asoc_machine_driver);

MODULE_DESCRIPTION("ALSA SoC sprat-mi2s");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, sprat_of_match);
