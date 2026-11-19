// SPDX-License-Identifier: GPL-2.0-only
//
// aw88461.c  --  AW88461 (PID 0x2308) ALSA SoC smart PA driver
//
// The I2S/TDM format, slot assignment and boost tuning all come from the
// register profile in the ACF firmware, so the DAI ops only gate power.
//

#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include "aw88395/aw88395_data_type.h"
#include "aw88395/aw88395_device.h"

#define AW88461_CHIP_ID			0x2308

#define AW88461_ID_REG			0x00
#define AW88461_SYSST_REG		0x01
#define AW88461_SYSINT_REG		0x02
#define AW88461_SYSINTM_REG		0x03
#define AW88461_SYSCTRL_REG		0x04
#define AW88461_SYSCTRL2_REG		0x05
#define AW88461_NGCTRL3_REG		0x5e
#define AW88461_REG_MAX			0x7f

#define AW88461_SOFT_RESET_VALUE	0x55aa
#define AW88461_SYSINTM_DEFAULT		0xffff

/* SYSST */
#define AW88461_BSTS			BIT(9)
#define AW88461_SWS			BIT(8)
#define AW88461_CLKS			BIT(4)
#define AW88461_PLLS			BIT(0)
#define AW88461_IIS_CHECK		(AW88461_CLKS | AW88461_PLLS)
#define AW88461_NO_SWS_CHECK		(AW88461_BSTS | AW88461_IIS_CHECK)
#define AW88461_SWS_CHECK		(AW88461_NO_SWS_CHECK | AW88461_SWS)

/* SYSCTRL */
#define AW88461_ULS_HMUTE		BIT(15)
#define AW88461_I2STXEN			BIT(13)
#define AW88461_HMUTE			BIT(8)
#define AW88461_AMPPD			BIT(1)
#define AW88461_PWDN			BIT(0)

/* SYSCTRL2: attenuation in steps of about 0.094 dB */
#define AW88461_VOL_MASK		GENMASK(9, 0)
#define AW88461_MUTE_VOL		1023

#define AW88461_NOISE_GATE_EN		BIT(0)

#define AW88461_CHECK_RETRIES		10
#define AW88461_START_RETRIES		3
#define AW88461_INIT_PROFILE		0
#define AW88461_ACF_FILE		"aw88461_acf.bin"

#define AW88461_RATES			(SNDRV_PCM_RATE_8000_48000 | \
					 SNDRV_PCM_RATE_96000)
#define AW88461_FORMATS			(SNDRV_PCM_FMTBIT_S16_LE | \
					 SNDRV_PCM_FMTBIT_S24_LE | \
					 SNDRV_PCM_FMTBIT_S32_LE)

enum {
	AW88461_PW_OFF = 0,
	AW88461_PW_ON,
};

enum {
	AW88461_FW_FAILED = 0,
	AW88461_FW_OK,
};

struct aw88461 {
	struct aw_device *aw_pa;
	struct regmap *regmap;
	/* Serializes profile loads and power sequencing. */
	struct mutex lock;
	struct aw_container *aw_cfg;

	unsigned int amppd_st;
	unsigned int mute_st;
	unsigned int txen_st;
	bool phase_sync;
};

static const struct regmap_config aw88461_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = AW88461_REG_MAX,
	.reg_format_endian = REGMAP_ENDIAN_LITTLE,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};

static void aw88461_update(struct aw88461 *aw, unsigned int mask, bool set)
{
	regmap_update_bits(aw->regmap, AW88461_SYSCTRL_REG, mask, set ? mask : 0);
}

static void aw88461_set_volume(struct aw88461 *aw, unsigned int atten)
{
	struct aw_volume_desc *vol = &aw->aw_pa->volume_desc;

	atten = min(atten + vol->init_volume, (unsigned int)AW88461_MUTE_VOL);
	regmap_update_bits(aw->regmap, AW88461_SYSCTRL2_REG, AW88461_VOL_MASK, atten);
}

static void aw88461_mute(struct aw88461 *aw, bool mute)
{
	if (mute) {
		aw88461_set_volume(aw, AW88461_MUTE_VOL);
		aw88461_update(aw, AW88461_HMUTE, true);
	} else {
		aw88461_update(aw, AW88461_HMUTE, false);
		aw88461_set_volume(aw, aw->aw_pa->volume_desc.ctl_volume);
	}
}

static void aw88461_clear_int_status(struct aw88461 *aw)
{
	unsigned int val;

	/* The status register is read-to-clear and may latch a second event. */
	regmap_read(aw->regmap, AW88461_SYSINT_REG, &val);
	regmap_read(aw->regmap, AW88461_SYSINT_REG, &val);
}

static int aw88461_wait_sysst(struct aw88461 *aw, unsigned int check)
{
	unsigned int val = 0;
	int i, ret;

	for (i = 0; i < AW88461_CHECK_RETRIES; i++) {
		ret = regmap_read(aw->regmap, AW88461_SYSST_REG, &val);
		if (ret)
			return ret;
		if ((val & check) == check)
			return 0;
		usleep_range(2000, 2010);
	}

	dev_dbg(aw->aw_pa->dev, "SYSST 0x%04x missing 0x%04x\n", val, check & ~val);
	return -EPERM;
}

static int aw88461_prof_reg_update(struct aw88461 *aw)
{
	struct aw_device *aw_dev = aw->aw_pa;
	struct aw_prof_desc *desc;
	struct aw_sec_data_desc *sec;
	unsigned int cur;
	const u16 *data;
	int i, len, ret;

	if (aw_dev->prof_index >= aw_dev->prof_info.count)
		return -EINVAL;

	desc = &aw_dev->prof_info.prof_desc[aw_dev->prof_index];
	sec = &desc->sec_desc[AW88395_DATA_TYPE_REG];
	if (!sec->data || !sec->len || sec->len % 4)
		return -EINVAL;

	data = (const u16 *)sec->data;
	len = sec->len / 2;

	for (i = 0; i < len; i += 2) {
		u8 reg = data[i];
		u16 val = data[i + 1];

		/* Profiles end with a 0xff padding entry. */
		if (data[i] > AW88461_REG_MAX)
			continue;

		switch (reg) {
		case AW88461_SYSINTM_REG:
			val = AW88461_SYSINTM_DEFAULT;
			break;
		case AW88461_SYSCTRL_REG:
			aw->amppd_st = val & AW88461_AMPPD;
			aw->mute_st = val & AW88461_HMUTE;
			aw->txen_st = val & AW88461_I2STXEN;

			ret = regmap_read(aw->regmap, reg, &cur);
			if (ret)
				return ret;

			/* Keep the live power state, hold the output muted, TX off. */
			val &= ~(AW88461_PWDN | AW88461_AMPPD | AW88461_HMUTE |
				 AW88461_I2STXEN);
			val |= cur & (AW88461_PWDN | AW88461_AMPPD | AW88461_HMUTE);
			val |= AW88461_ULS_HMUTE;
			break;
		case AW88461_SYSCTRL2_REG:
			aw_dev->volume_desc.init_volume = val & AW88461_VOL_MASK;
			break;
		}

		ret = regmap_write(aw->regmap, reg, val);
		if (ret)
			return ret;
	}

	if (aw_dev->prof_cur != aw_dev->prof_index)
		aw_dev->volume_desc.ctl_volume = 0;
	aw_dev->prof_cur = aw_dev->prof_index;

	aw88461_set_volume(aw, AW88461_MUTE_VOL);

	return 0;
}

static int aw88461_reg_update(struct aw88461 *aw)
{
	struct aw_device *aw_dev = aw->aw_pa;
	int ret;

	if (aw->phase_sync) {
		ret = regmap_write(aw->regmap, AW88461_ID_REG, AW88461_SOFT_RESET_VALUE);
		if (ret)
			return ret;
		return aw88461_prof_reg_update(aw);
	}

	if (aw_dev->prof_cur != aw_dev->prof_index)
		return aw88461_prof_reg_update(aw);

	return 0;
}

static void aw88461_dev_stop(struct aw88461 *aw)
{
	struct aw_device *aw_dev = aw->aw_pa;

	if (aw_dev->status == AW88461_PW_OFF)
		return;

	aw_dev->status = AW88461_PW_OFF;

	aw88461_clear_int_status(aw);
	aw88461_update(aw, AW88461_ULS_HMUTE, true);
	aw88461_mute(aw, true);
	usleep_range(5000, 5010);
	aw88461_update(aw, AW88461_I2STXEN, false);
	usleep_range(1000, 1100);
	aw88461_update(aw, AW88461_AMPPD, true);
	aw88461_update(aw, AW88461_PWDN, true);
}

static int aw88461_dev_start(struct aw88461 *aw)
{
	struct aw_device *aw_dev = aw->aw_pa;
	unsigned int ng, check;
	int ret;

	if (aw_dev->status == AW88461_PW_ON)
		return 0;

	aw88461_update(aw, AW88461_PWDN, false);
	usleep_range(2000, 2010);

	ret = aw88461_wait_sysst(aw, AW88461_IIS_CHECK);
	if (ret) {
		dev_dbg(aw_dev->dev, "PLL did not lock on the TDM clock\n");
		goto pwd;
	}

	aw88461_update(aw, AW88461_AMPPD, false);
	usleep_range(1000, 1050);

	/* With the noise gate on, the class-D stage may legitimately idle. */
	ret = regmap_read(aw->regmap, AW88461_NGCTRL3_REG, &ng);
	if (ret)
		goto amppd;
	check = (ng & AW88461_NOISE_GATE_EN) ? AW88461_NO_SWS_CHECK : AW88461_SWS_CHECK;

	ret = aw88461_wait_sysst(aw, check);
	if (ret) {
		dev_dbg(aw_dev->dev, "amplifier did not start\n");
		goto amppd;
	}

	if (aw->txen_st)
		aw88461_update(aw, AW88461_I2STXEN, true);
	if (aw->amppd_st)
		aw88461_update(aw, AW88461_AMPPD, true);

	aw88461_update(aw, AW88461_ULS_HMUTE, false);
	if (!aw->mute_st)
		aw88461_mute(aw, false);

	aw88461_clear_int_status(aw);
	aw_dev->status = AW88461_PW_ON;

	return 0;

amppd:
	aw88461_update(aw, AW88461_I2STXEN, false);
	aw88461_clear_int_status(aw);
	aw88461_update(aw, AW88461_AMPPD, true);
pwd:
	aw88461_update(aw, AW88461_PWDN, true);
	aw_dev->status = AW88461_PW_OFF;

	return ret;
}

static void aw88461_start(struct aw88461 *aw)
{
	int i, ret = 0;

	if (aw->aw_pa->fw_status != AW88461_FW_OK ||
	    aw->aw_pa->status == AW88461_PW_ON)
		return;

	for (i = 0; i < AW88461_START_RETRIES; i++) {
		ret = aw88461_reg_update(aw);
		if (ret)
			continue;
		ret = aw88461_dev_start(aw);
		if (!ret)
			return;
	}

	dev_err(aw->aw_pa->dev, "start failed: %d\n", ret);
}

static int aw88461_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	return 0;
}

static int aw88461_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
				unsigned int rx_mask, int slots, int slot_width)
{
	return 0;
}

static int aw88461_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *dai)
{
	struct aw88461 *aw = snd_soc_component_get_drvdata(dai->component);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		return 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		mutex_lock(&aw->lock);
		aw88461_start(aw);
		mutex_unlock(&aw->lock);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		mutex_lock(&aw->lock);
		aw88461_dev_stop(aw);
		mutex_unlock(&aw->lock);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops aw88461_dai_ops = {
	.set_fmt = aw88461_set_fmt,
	.set_tdm_slot = aw88461_set_tdm_slot,
	.trigger = aw88461_trigger,
};

static struct snd_soc_dai_driver aw88461_dai[] = {
	{
		.name = "aw88461-aif",
		.id = 1,
		.playback = {
			.stream_name = "Speaker_Playback",
			.channels_min = 1,
			.channels_max = 8,
			.rates = AW88461_RATES,
			.formats = AW88461_FORMATS,
		},
		.capture = {
			.stream_name = "Speaker_Capture",
			.channels_min = 1,
			.channels_max = 8,
			.rates = AW88461_RATES,
			.formats = AW88461_FORMATS,
		},
		.ops = &aw88461_dai_ops,
	},
};

static int aw88461_profile_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	struct snd_soc_component *codec = snd_kcontrol_chip(kcontrol);
	struct aw88461 *aw = snd_soc_component_get_drvdata(codec);
	struct aw_prof_info *info = &aw->aw_pa->prof_info;
	int count = info->count;
	unsigned int item;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = max(count, 0);
	if (count <= 0)
		return 0;

	item = min_t(unsigned int, uinfo->value.enumerated.item, count - 1);
	uinfo->value.enumerated.item = item;
	strscpy(uinfo->value.enumerated.name,
		info->prof_name_list[info->prof_desc[item].id]);

	return 0;
}

static int aw88461_profile_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_kcontrol_chip(kcontrol);
	struct aw88461 *aw = snd_soc_component_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = aw->aw_pa->prof_index;

	return 0;
}

static int aw88461_profile_set(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_kcontrol_chip(kcontrol);
	struct aw88461 *aw = snd_soc_component_get_drvdata(codec);
	unsigned int index = ucontrol->value.enumerated.item[0];

	if (index >= aw->aw_pa->prof_info.count)
		return -EINVAL;

	guard(mutex)(&aw->lock);
	if (aw->aw_pa->prof_index == index)
		return 0;

	aw->aw_pa->prof_index = index;
	if (aw->aw_pa->status == AW88461_PW_ON) {
		aw88461_dev_stop(aw);
		aw88461_start(aw);
	}

	return 1;
}

static int aw88461_volume_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_kcontrol_chip(kcontrol);
	struct aw88461 *aw = snd_soc_component_get_drvdata(codec);

	ucontrol->value.integer.value[0] =
		AW88461_MUTE_VOL - aw->aw_pa->volume_desc.ctl_volume;

	return 0;
}

static int aw88461_volume_set(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_kcontrol_chip(kcontrol);
	struct aw88461 *aw = snd_soc_component_get_drvdata(codec);
	struct aw_volume_desc *vol = &aw->aw_pa->volume_desc;
	long value = ucontrol->value.integer.value[0];

	if (value < 0 || value > AW88461_MUTE_VOL)
		return -EINVAL;

	value = AW88461_MUTE_VOL - value;
	if (vol->ctl_volume == value)
		return 0;

	guard(mutex)(&aw->lock);
	vol->ctl_volume = value;
	if (aw->aw_pa->status == AW88461_PW_ON && !aw->mute_st)
		aw88461_set_volume(aw, value);

	return 1;
}

/* 1023 steps of 0.094 dB; the TLV step is rounded down to 0.09 dB. */
static const DECLARE_TLV_DB_SCALE(aw88461_volume_tlv, -9622, 9, 1);

static const struct snd_kcontrol_new aw88461_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "PCM Playback Volume",
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |
			  SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = snd_soc_info_volsw,
		.get = aw88461_volume_get,
		.put = aw88461_volume_set,
		.tlv.p = aw88461_volume_tlv,
		.private_value = SOC_SINGLE_VALUE(SND_SOC_NOPM, 0, 0,
						  AW88461_MUTE_VOL, 0, 0),
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Profile Set",
		.info = aw88461_profile_info,
		.get = aw88461_profile_get,
		.put = aw88461_profile_set,
	},
};

static int aw88461_playback_event(struct snd_soc_dapm_widget *w,
				  struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct aw88461 *aw = snd_soc_component_get_drvdata(component);

	guard(mutex)(&aw->lock);
	aw88461_dev_stop(aw);

	return 0;
}

static const struct snd_soc_dapm_widget aw88461_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN_E("AIF_RX", "Speaker_Playback", 0, SND_SOC_NOPM, 0, 0,
			      aw88461_playback_event, SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_OUTPUT("DAC Output"),
	SND_SOC_DAPM_AIF_OUT("AIF_TX", "Speaker_Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_INPUT("ADC Input"),
};

static const struct snd_soc_dapm_route aw88461_audio_map[] = {
	{ "DAC Output", NULL, "AIF_RX" },
	{ "AIF_TX", NULL, "ADC Input" },
};

static int aw88461_dev_init(struct aw88461 *aw)
{
	struct aw_device *aw_dev = aw->aw_pa;
	int ret;

	ret = aw88395_dev_cfg_load(aw_dev, aw->aw_cfg);
	if (ret) {
		dev_err(aw_dev->dev, "ACF parse failed: %d\n", ret);
		return ret;
	}

	ret = regmap_write(aw->regmap, AW88461_ID_REG, AW88461_SOFT_RESET_VALUE);
	if (ret)
		return ret;

	aw_dev->prof_cur = AW88461_INIT_PROFILE;
	aw_dev->prof_index = AW88461_INIT_PROFILE;
	ret = aw88461_prof_reg_update(aw);
	if (ret) {
		dev_err(aw_dev->dev, "profile load failed: %d\n", ret);
		return ret;
	}

	aw_dev->status = AW88461_PW_ON;
	aw88461_dev_stop(aw);

	return 0;
}

static int aw88461_request_firmware(struct aw88461 *aw)
{
	const struct firmware *cont __free(firmware) = NULL;
	struct device *dev = aw->aw_pa->dev;
	struct aw_container *cfg;
	const char *fw_name;
	int ret;

	aw->aw_pa->fw_status = AW88461_FW_FAILED;

	if (device_property_read_string(dev, "firmware-name", &fw_name))
		fw_name = AW88461_ACF_FILE;

	ret = request_firmware(&cont, fw_name, dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to load %s\n", fw_name);

	cfg = devm_kzalloc(dev, struct_size(cfg, data, cont->size), GFP_KERNEL);
	if (!cfg)
		return -ENOMEM;

	cfg->len = cont->size;
	memcpy(cfg->data, cont->data, cont->size);
	aw->aw_cfg = cfg;

	ret = aw88395_dev_load_acf_check(aw->aw_pa, cfg);
	if (ret) {
		dev_err(dev, "invalid ACF %s\n", fw_name);
		return ret;
	}

	guard(mutex)(&aw->lock);
	ret = aw88461_dev_init(aw);
	if (ret)
		return ret;

	aw->aw_pa->fw_status = AW88461_FW_OK;
	dev_info(dev, "channel %u: %d profiles from %s\n", aw->aw_pa->channel,
		 aw->aw_pa->prof_info.count, fw_name);

	return 0;
}

static int aw88461_codec_probe(struct snd_soc_component *component)
{
	struct snd_soc_dapm_context *dapm = snd_soc_component_to_dapm(component);
	struct aw88461 *aw = snd_soc_component_get_drvdata(component);
	int ret;

	ret = aw88461_request_firmware(aw);
	if (ret)
		return ret;

	ret = snd_soc_dapm_new_controls(dapm, aw88461_dapm_widgets,
					ARRAY_SIZE(aw88461_dapm_widgets));
	if (ret)
		return ret;

	ret = snd_soc_dapm_add_routes(dapm, aw88461_audio_map,
				      ARRAY_SIZE(aw88461_audio_map));
	if (ret)
		return ret;

	return snd_soc_add_component_controls(component, aw88461_controls,
					      ARRAY_SIZE(aw88461_controls));
}

static const struct snd_soc_component_driver aw88461_component = {
	.probe = aw88461_codec_probe,
};

static int aw88461_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct aw_device *aw_dev;
	struct aw88461 *aw;
	unsigned int chip_id;
	u32 channel = 0;
	int ret;

	aw = devm_kzalloc(dev, sizeof(*aw), GFP_KERNEL);
	if (!aw)
		return -ENOMEM;

	aw_dev = devm_kzalloc(dev, sizeof(*aw_dev), GFP_KERNEL);
	if (!aw_dev)
		return -ENOMEM;

	mutex_init(&aw->lock);
	i2c_set_clientdata(i2c, aw);

	aw->regmap = devm_regmap_init_i2c(i2c, &aw88461_regmap_config);
	if (IS_ERR(aw->regmap))
		return dev_err_probe(dev, PTR_ERR(aw->regmap), "regmap init failed\n");

	ret = devm_regulator_get_enable(dev, "dvdd");
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable dvdd\n");

	/*
	 * The reset line may have been released only just now (by the default
	 * pinctrl state applied right before probe, or together with the
	 * supply). The chip does not acknowledge its I2C address until it has
	 * left reset, so give it time before the first access.
	 */
	usleep_range(5000, 6000);

	ret = regmap_read(aw->regmap, AW88461_ID_REG, &chip_id);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read chip id\n");
	if (chip_id != AW88461_CHIP_ID)
		return dev_err_probe(dev, -ENODEV, "unsupported chip id 0x%04x\n", chip_id);

	of_property_read_u32(dev->of_node, "awinic,audio-channel", &channel);
	aw->phase_sync = of_property_read_bool(dev->of_node, "awinic,sync-flag");

	aw_dev->i2c = i2c;
	aw_dev->dev = dev;
	aw_dev->regmap = aw->regmap;
	aw_dev->chip_id = AW88461_CHIP_ID;
	aw_dev->channel = channel;
	aw_dev->prof_info.prof_type = AW88395_DEV_NONE_TYPE_ID;
	aw_dev->fw_status = AW88461_FW_FAILED;
	aw_dev->status = AW88461_PW_OFF;
	aw_dev->volume_desc.mute_volume = AW88461_MUTE_VOL;
	aw->aw_pa = aw_dev;

	ret = devm_snd_soc_register_component(dev, &aw88461_component,
					      aw88461_dai, ARRAY_SIZE(aw88461_dai));
	return ret;
}

static const struct i2c_device_id aw88461_i2c_id[] = {
	{ "aw88461" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw88461_i2c_id);

static const struct of_device_id aw88461_of_match[] = {
	{ .compatible = "awinic,aw88461" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw88461_of_match);

static struct i2c_driver aw88461_i2c_driver = {
	.driver = {
		.name = "aw88461",
		.of_match_table = aw88461_of_match,
	},
	.probe = aw88461_i2c_probe,
	.id_table = aw88461_i2c_id,
};
module_i2c_driver(aw88461_i2c_driver);

MODULE_DESCRIPTION("ASoC AW88461 smart PA driver");
MODULE_LICENSE("GPL");
