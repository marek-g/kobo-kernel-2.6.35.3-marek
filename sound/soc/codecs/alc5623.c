/*
 * alc5623.c  --  ALC5623 ALSA SoC Audio driver
 *
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <mach/hardware.h>

#include <linux/gallen_dbg.h>

#include "alc5623.h"

struct alc5623_priv {
	int sysclk;
	int master;
	int fmt;
	int rev;
	int lrclk;
	int capture_channels;
	int playback_active;
	int capture_active;
	int clock_on;		/* clock enable status */
	int need_clk_for_access; /* need clock on because doing access */
	int need_clk_for_bias;   /* need clock on due to bias level */
	int (*clock_enable) (int enable);
	struct regulator *reg_vddio;
	struct regulator *reg_vdda;
	struct regulator *reg_vddd;
	int vddio;		/* voltage of VDDIO (mv) */
	int vdda;		/* voltage of vdda (mv) */
	int vddd;		/* voltage of vddd (mv), 0 if not connected */
	struct snd_pcm_substream *master_substream;
	struct snd_pcm_substream *slave_substream;
};

//Gallen 20100715 :  follow ALC5623 dac volume 5 bits table .
static const unsigned char gbRegVolTableA[32] = {
	0x1f,0x1e,0x1d,0x1c,0x1b,0x1a,0x19,0x18,0x17,0x16,
	0x15,0x14,0x13,0x12,0x11,0x10,0x0f,0x0e,0x0d,0x0c,
	0x0b,0x0a,0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x02,
	0x01,0x00	 
};

static volatile unsigned char gbVolR=0xB8,gbVolL=0xB8; // default is 0 db .

static int alc5623_set_bias_level(struct snd_soc_codec *codec,
				   enum snd_soc_bias_level level);
static int alc5623_mute (struct snd_soc_codec *codec , int mute);

#define ALC5623_MAX_CACHED_REG ALC5623_CHIP_SHORT_CTRL
static u16 alc5623_regs[(ALC5623_MAX_CACHED_REG >> 1) + 1];

static struct snd_soc_codec *alc5623_codec;

/*
 * Schedule clock to be turned off or turn clock on.
 */
static void alc5623_clock_gating(struct snd_soc_codec *codec, int enable)
{
	struct alc5623_priv *alc5623 = snd_soc_codec_get_drvdata(codec);

	if (alc5623->clock_enable == NULL)
		return;

	if (enable == 0) {
		if (!alc5623->need_clk_for_access &&
		    !alc5623->need_clk_for_bias)
			schedule_delayed_work(&codec->delayed_work,
					      msecs_to_jiffies(300));
	} else {
		if (!alc5623->clock_on) {
			alc5623->clock_enable(1);
			alc5623->clock_on = 1;
		}
	}
}

static unsigned int alc5623_read(struct snd_soc_codec *codec,
				     unsigned int reg)
{
	struct alc5623_priv *alc5623 = snd_soc_codec_get_drvdata(codec);
	struct i2c_client *client = codec->control_data;
	int i2c_ret;
	u16 value;
	u8 buf0[2], buf1[2];
	u16 addr = client->addr;
	u16 flags = client->flags;
	struct i2c_msg msg[2] = {
		{addr, flags, 1, buf0},
		{addr, flags | I2C_M_RD, 2, buf1},
	};

	alc5623->need_clk_for_access = 1;
	alc5623_clock_gating(codec, 1);
	buf0[0] = reg & 0xff;
	i2c_ret = i2c_transfer(client->adapter, msg, 2);
	alc5623->need_clk_for_access = 0;
	alc5623_clock_gating(codec, 0);
	if (i2c_ret < 0) {
		pr_err("%s: read reg error : Reg 0x%02x\n", __func__, reg);
		return 0;
	}

	value = buf1[0] << 8 | buf1[1];

	pr_debug("r r:%02x,v:%04x\n", reg, value);
	return value;
}

static int alc5623_write(struct snd_soc_codec *codec, unsigned int reg,
			  unsigned int value)
{
	struct alc5623_priv *alc5623 = snd_soc_codec_get_drvdata(codec);
	struct i2c_client *client = codec->control_data;
	u16 addr = client->addr;
	u16 flags = client->flags;
	u8 buf[4];
	int i2c_ret;
	struct i2c_msg msg = { addr, flags, 3, buf };
	static int entryCnt;

	if (3 < entryCnt++) {
		entryCnt = 0;
		return -EIO;
	}
	alc5623->need_clk_for_access = 1;
	alc5623_clock_gating(codec, 1);
	pr_debug("w r:%02x,v:%04x\n", reg, value);
//printk ("[%s-%d] REG0x%02X 0x%04X\n",__FUNCTION__,__LINE__,reg,value);
	buf[0] = reg & 0xff;
	buf[1] = (value & 0xff00) >> 8;
	buf[2] = value & 0xff;

	i2c_ret = i2c_transfer(client->adapter, &msg, 1);
	alc5623->need_clk_for_access = 0;
	alc5623_clock_gating(codec, 0);
	if (i2c_ret < 0) {
		msleep (20);
		pr_err("%s: write reg error : Reg 0x%02x = 0x%04x\n",
		       __func__, reg, value);
		return alc5623_write (codec, reg, value);
	}

	msleep (20);
	if (reg && value != alc5623_read (codec, reg))	{// Joseph 20100728
		if(printk_ratelimit()) {
			printk("alc5623 write not match !! reg %d value %X\n",reg,value);
		}
		alc5623_write (codec, reg, value);
	}
	entryCnt = 0;
	return i2c_ret;


}

int alc5623_get_volume(void)
{
	int iCurVolume=(int)(gbVolL);
	DBG_MSG("%s:iCurVolume=%d\n",__FUNCTION__,iCurVolume);
	return iCurVolume;
}

int alc5623_set_volume(int iSetVol)
{
	int iOldVolume=(int)(gbVolL);

	struct snd_soc_codec *codec ;
	unsigned short wTemp;
	unsigned char bTemp = (unsigned char)iSetVol;
	int iChk;
	
	if(alc5623_codec) {
		//unsigned char bTempL = (gbVolL>>3)&0x1f,bTempR = (gbVolR>>3)&0x1f;
		
		codec = alc5623_codec ;
	
		//wTemp=(unsigned short)((gbRegVolTableA[bTempL]<<8&0xff00)|gbRegVolTableA[bTempR])&0x1f1f;
		//iChk = alc5623_write(codec, (unsigned int)0x0C,(unsigned int)(0xe000|wTemp));// set DAC vol mute
		if (bTemp && (0 == gbVolL))
			alc5623_mute (codec, 0);
		
		gbVolL = gbVolR = bTemp;
		if (0 == bTemp) {
			alc5623_mute (codec, 1);
		}
		else {
			bTemp = (bTemp>>3)&0x1f;
			wTemp=(unsigned short)((gbRegVolTableA[bTemp]<<8&0xff00)|gbRegVolTableA[bTemp]);
			DBG_MSG("%s: write alc5623 (0x%x,0x%x),0x%x,0x%x\n",__FUNCTION__,gbVolL,gbVolR,bTemp,wTemp);
			//iChk = alc5623_write(codec, (unsigned int)0x0C,(unsigned int)(0xe000|wTemp));// set DAC vol 
			iChk = alc5623_write(codec, (unsigned int)0x0C,(unsigned int)(wTemp));// set DAC vol unmute .
		}
	}
	else {
		gbVolL = gbVolR = bTemp;
		WARNING_MSG("sgtl5000 : %s fail -> codec not probe ready !\n ",__FUNCTION__);
	}
	return iOldVolume;
	
}

static int all_reg[] = {
	ALC5623_CHIP_ID,
	ALC5623_CHIP_DIG_POWER,
	ALC5623_CHIP_CLK_CTRL,
	ALC5623_CHIP_I2S_CTRL,
	ALC5623_CHIP_SSS_CTRL,
	ALC5623_CHIP_ADCDAC_CTRL,
	ALC5623_CHIP_DAC_VOL,
	ALC5623_CHIP_PAD_STRENGTH,
	ALC5623_CHIP_ANA_ADC_CTRL,
	ALC5623_CHIP_ANA_HP_CTRL,
	ALC5623_CHIP_ANA_CTRL,
	ALC5623_CHIP_LINREG_CTRL,
	ALC5623_CHIP_REF_CTRL,
	ALC5623_CHIP_MIC_CTRL,
	ALC5623_CHIP_LINE_OUT_CTRL,
	ALC5623_CHIP_LINE_OUT_VOL,
	ALC5623_CHIP_ANA_POWER,
	ALC5623_CHIP_PLL_CTRL,
	ALC5623_CHIP_CLK_TOP_CTRL,
	ALC5623_CHIP_ANA_STATUS,
	ALC5623_CHIP_SHORT_CTRL,
};

#ifdef DEBUG
static void dump_reg(struct snd_soc_codec *codec)
{
	int i, reg;
	printk(KERN_DEBUG "dump begin\n");
	for (i = 0; i < 21; i++) {
		reg = alc5623_read(codec, all_reg[i]);
		printk(KERN_DEBUG "d r %04x, v %04x\n", all_reg[i], reg);
	}
	printk(KERN_DEBUG "dump end\n");
}
#else
static void dump_reg(struct snd_soc_codec *codec)
{
}
#endif

static const char *adc_mux_text[] = {
	"MIC_IN", "LINE_IN"
};

static const char *dac_mux_text[] = {
	"DAC", "LINE_IN"
};

static const struct soc_enum adc_enum =
SOC_ENUM_SINGLE(ALC5623_CHIP_ANA_CTRL, 2, 2, adc_mux_text);

static const struct soc_enum dac_enum =
SOC_ENUM_SINGLE(ALC5623_CHIP_ANA_CTRL, 6, 2, dac_mux_text);

static const struct snd_kcontrol_new adc_mux =
SOC_DAPM_ENUM("ADC Mux", adc_enum);

static const struct snd_kcontrol_new dac_mux =
SOC_DAPM_ENUM("DAC Mux", dac_enum);

static const struct snd_soc_dapm_widget alc5623_dapm_widgets[] = {
//	SND_SOC_DAPM_INPUT("LINE_IN"),
//	SND_SOC_DAPM_INPUT("MIC_IN"),

	SND_SOC_DAPM_OUTPUT("HP_OUT"),
	SND_SOC_DAPM_OUTPUT("LINE_OUT"),

//	SND_SOC_DAPM_PGA("HP", ALC5623_CHIP_ANA_CTRL, 4, 1, NULL, 0),
//	SND_SOC_DAPM_PGA("LO", ALC5623_CHIP_ANA_CTRL, 8, 1, NULL, 0),

//	SND_SOC_DAPM_MUX("ADC Mux", SND_SOC_NOPM, 0, 0, &adc_mux),
//	SND_SOC_DAPM_MUX("DAC Mux", SND_SOC_NOPM, 0, 0, &dac_mux),

//	SND_SOC_DAPM_ADC("ADC", "Capture", ALC5623_CHIP_DIG_POWER, 6, 0),
	SND_SOC_DAPM_DAC("DAC", "Playback", SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{"ADC Mux", "LINE_IN", "LINE_IN"},
	{"ADC Mux", "MIC_IN", "MIC_IN"},
	{"ADC", NULL, "ADC Mux"},
	{"DAC Mux", "DAC", "DAC"},
	{"DAC Mux", "LINE_IN", "LINE_IN"},
	{"LO", NULL, "DAC"},
	{"HP", NULL, "DAC Mux"},
	{"LINE_OUT", NULL, "LO"},
	{"HP_OUT", NULL, "HP"},
};

static int alc5623_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, alc5623_dapm_widgets,
				  ARRAY_SIZE(alc5623_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

static int dac_info_volsw(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0xfc - 0x3c;
	return 0;
}

static int dac_get_volsw(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
#if 0
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg, l, r;

	reg = alc5623_read(codec, ALC5623_CHIP_DAC_VOL);
	l = (reg & ALC5623_DAC_VOL_LEFT_MASK) >> ALC5623_DAC_VOL_LEFT_SHIFT;
	r = (reg & ALC5623_DAC_VOL_RIGHT_MASK) >> ALC5623_DAC_VOL_RIGHT_SHIFT;
	l = l < 0x3c ? 0x3c : l;
	l = l > 0xfc ? 0xfc : l;
	r = r < 0x3c ? 0x3c : r;
	r = r > 0xfc ? 0xfc : r;
	l = 0xfc - l;
	r = 0xfc - r;

	ucontrol->value.integer.value[0] = l;
	ucontrol->value.integer.value[1] = r;
#endif
	return 0;
}

static int dac_put_volsw(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg, l, r;

	l = ucontrol->value.integer.value[0];
	r = ucontrol->value.integer.value[1];

	l = l < 0 ? 0 : l;
	l = l > 0xfc - 0x3c ? 0xfc - 0x3c : l;
	r = r < 0 ? 0 : r;
	r = r > 0xfc - 0x3c ? 0xfc - 0x3c : r;
	l = 0xfc - l;
	r = 0xfc - r;

	reg = l << ALC5623_DAC_VOL_LEFT_SHIFT |
	    r << ALC5623_DAC_VOL_RIGHT_SHIFT;

	alc5623_write(codec, ALC5623_CHIP_DAC_VOL, reg);

	return 0;
}

static const char *mic_gain_text[] = {
	"0dB", "20dB", "30dB", "40dB"
};

static const char *adc_m6db_text[] = {
	"No Change", "Reduced by 6dB"
};

static const struct soc_enum mic_gain =
SOC_ENUM_SINGLE(ALC5623_CHIP_MIC_CTRL, 0, 4, mic_gain_text);

static const struct soc_enum adc_m6db =
SOC_ENUM_SINGLE(ALC5623_CHIP_ANA_ADC_CTRL, 8, 2, adc_m6db_text);

static const struct snd_kcontrol_new alc5623_snd_controls[] = {
	SOC_ENUM("MIC GAIN", mic_gain),
	SOC_DOUBLE("Capture Volume", ALC5623_CHIP_ANA_ADC_CTRL, 0, 4, 0xf, 0),
	SOC_ENUM("Capture Vol Reduction", adc_m6db),
	{.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .name = "Playback Volume",
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
	 SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = dac_info_volsw,
	 .get = dac_get_volsw,
	 .put = dac_put_volsw,
	 },
	SOC_DOUBLE("Headphone Volume", ALC5623_CHIP_ANA_HP_CTRL, 0, 8, 0x7f,
		   1),
};

static int alc5623_mute (struct snd_soc_codec *codec , int mute)
{
	if (mute) {
		alc5623_write(codec, 0x5E, 0x0200);// Enable HP depop2
		msleep (20);	// delay 1ms
		alc5623_write(codec, 0x3A, 0x8110);// Disable HP out
		alc5623_write(codec, 0x3A, 0x8100);// Disable HP Amp
		alc5623_write(codec, 0x04, 0x8080);// HP out mute
		alc5623_write(codec, 0x5E, 0x0000);// Disable HP depop2
	}
	else {
		alc5623_write(codec, 0x5E, 0x0200);// Enable HP depop2
		alc5623_write(codec, 0x3A, 0x8110);// Disable HP out
		alc5623_write(codec, 0x3A, 0x8100);// Disable HP Amp
		alc5623_write(codec, 0x04, 0x0000);// set volume
		msleep (20);	// delay 1ms
		alc5623_write(codec, 0x3A, 0x8120);// Enable HP out
		alc5623_write(codec, 0x3A, 0x8130);// Enable HP Amp
		alc5623_write(codec, 0x5E, 0x0000);// Disable HP depop2
	}
}

static int alc5623_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
#if 1
	alc5623_mute (codec_dai->codec, mute);
#else
	struct snd_soc_codec *codec = codec_dai->codec;

	/* the digital mute covers the HiFi and Voice DAC's on the alc5623.
	 * make sure we check if they are not both active when we mute */
//printk ("[%s-%d] %d\n",__FUNCTION__,__LINE__, mute);
	if (mute) {
		alc5623_write(codec, 0x5E, 0x0200);// Enable HP depop2
		msleep (20);	// delay 1ms
		alc5623_write(codec, 0x3A, 0x8110);// Disable HP out
		alc5623_write(codec, 0x3A, 0x8100);// Disable HP Amp
		alc5623_write(codec, 0x04, 0x8080);// HP out mute
		alc5623_write(codec, 0x5E, 0x0000);// Disable HP depop2
	}
	else {
		alc5623_write(codec, 0x5E, 0x0200);// Enable HP depop2
		alc5623_write(codec, 0x3A, 0x8110);// Disable HP out
		alc5623_write(codec, 0x3A, 0x8100);// Disable HP Amp
		alc5623_write(codec, 0x04, 0x0000);// set volume
		msleep (20);	// delay 1ms
		alc5623_write(codec, 0x3A, 0x8120);// Enable HP out
		alc5623_write(codec, 0x3A, 0x8130);// Enable HP Amp
		alc5623_write(codec, 0x5E, 0x0000);// Disable HP depop2
	}
#endif
	return 0;
}

static u16 i2sctl = 0;
static int alc5623_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct alc5623_priv *alc5623 = snd_soc_codec_get_drvdata(codec);
	pr_debug("%s:fmt=%08x\n", __func__, fmt);
	
printk ("[%s-%d] fmt=%08x\n",__FUNCTION__,__LINE__,fmt);
	pr_debug("%s:fmt=%08x\n", __func__, fmt);

	alc5623_write(codec, 0x3E, 0x8000);// Power on main bias
	alc5623_write(codec, 0x04, 0x8080);// HP L/R mute
	alc5623_write(codec, 0x3A, 0x0100);// Power on softgen
	alc5623_write(codec, 0x3c, 0x3730);// Power on Vref
	alc5623_write(codec, 0x3E, 0x8600);// Power on HP L/R vol
	alc5623_write(codec, 0x5E, 0x0200);// Enable HP depop2
	// delay > 300 ms
	msleep (400);
//	alc5623_write(codec, 0x0C, 0x0808);// set DAC vol 0db
	alc5623_write(codec, 0x0C, (unsigned short)((gbRegVolTableA[(gbVolL>>3)]<<8&0xff00)|gbRegVolTableA[(gbVolR>>3)]));
	alc5623_write(codec, 0x1C, 0xD300);// set hp vol io select
	alc5623_write(codec, 0x34, 0x0000);// 
	alc5623_write(codec, 0x36, 0x1A69);// 
	
/*	
	alc5623_write(codec, 0x04, 0x0000);// set volume
	alc5623_write(codec, 0x3A, 0x8120);// Enable HP out
	alc5623_write(codec, 0x3A, 0x8130);// Enable HP Amp
*/	
	alc5623_write(codec, 0x5E, 0x0000);// Disable HP depop2
	
	alc5623->master = 0;
	i2sctl |= 0x8000;
	i2sctl |= 0x001C;
	
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
printk ("[%s-%d] SND_SOC_DAIFMT_CBS_CFS\n",__FUNCTION__,__LINE__);
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
printk ("[%s-%d] SND_SOC_DAIFMT_CBM_CFM\n",__FUNCTION__,__LINE__);
		i2sctl &= ~0x8000;
		alc5623->master = 1;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
printk ("[%s-%d] SND_SOC_DAIFMT_CBM_CFS\n",__FUNCTION__,__LINE__);
	case SND_SOC_DAIFMT_CBS_CFM:
printk ("[%s-%d] SND_SOC_DAIFMT_CBS_CFM\n",__FUNCTION__,__LINE__);
		return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
printk ("[%s-%d] SND_SOC_DAIFMT_FORMAT_MASK\n",__FUNCTION__,__LINE__);
		i2sctl |= 0x0003;
		break;
	case SND_SOC_DAIFMT_DSP_B:
printk ("[%s-%d] SND_SOC_DAIFMT_DSP_B\n",__FUNCTION__,__LINE__);
		i2sctl |= 0x4003;
		break;
	case SND_SOC_DAIFMT_I2S:
printk ("[%s-%d] SND_SOC_DAIFMT_I2S\n",__FUNCTION__,__LINE__);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
printk ("[%s-%d] SND_SOC_DAIFMT_RIGHT_J\n",__FUNCTION__,__LINE__);
		i2sctl |= 0x0001;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
printk ("[%s-%d] SND_SOC_DAIFMT_LEFT_J\n",__FUNCTION__,__LINE__);
		i2sctl |= 0x0002;
		break;
	default:
		return -EINVAL;
	}
	alc5623->fmt = fmt & SND_SOC_DAIFMT_FORMAT_MASK;

	/* Clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
printk ("[%s-%d] SND_SOC_DAIFMT_NB_NF\n",__FUNCTION__,__LINE__);
//		i2sctl |= 0x0010;	// DAC L/R swap.
	case SND_SOC_DAIFMT_NB_IF:
printk ("[%s-%d] SND_SOC_DAIFMT_NB_IF\n",__FUNCTION__,__LINE__);
		break;
	case SND_SOC_DAIFMT_IB_IF:
printk ("[%s-%d] SND_SOC_DAIFMT_IB_IF\n",__FUNCTION__,__LINE__);
	case SND_SOC_DAIFMT_IB_NF:
printk ("[%s-%d] SND_SOC_DAIFMT_IB_NF\n",__FUNCTION__,__LINE__);
		i2sctl |= 0x0080;
		break;
	default:
		return -EINVAL;
	}
	alc5623_write(codec, 0x34, i2sctl);

	return 0;
}

static int alc5623_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				   int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct alc5623_priv *alc5623 = snd_soc_codec_get_drvdata(codec);

	switch (clk_id) {
	case ALC5623_SYSCLK:
		alc5623->sysclk = freq;
		break;
	case ALC5623_LRCLK:
		alc5623->lrclk = freq;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int alc5623_pcm_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
#if 0
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct alc5623_priv *alc5623 = snd_soc_codec_get_drvdata(codec);
	int reg;

	reg = alc5623_read(codec, ALC5623_CHIP_DIG_POWER);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		reg |= ALC5623_I2S_IN_POWERUP;
	else
		reg |= ALC5623_I2S_OUT_POWERUP;
	alc5623_write(codec, ALC5623_CHIP_DIG_POWER, reg);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		reg = alc5623_read(codec, ALC5623_CHIP_ANA_POWER);
		reg |= ALC5623_ADC_POWERUP;
		if (alc5623->capture_channels == 1)
			reg &= ~ALC5623_ADC_STEREO;
		else
			reg |= ALC5623_ADC_STEREO;
		alc5623_write(codec, ALC5623_CHIP_ANA_POWER, reg);
	}
#endif
	return 0;
}

static int alc5623_pcm_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct alc5623_priv *alc5623 = snd_soc_codec_get_drvdata(codec);
	struct snd_pcm_runtime *master_runtime;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		alc5623->playback_active++;
	else
		alc5623->capture_active++;

	/* The DAI has shared clocks so if we already have a playback or
	 * capture going then constrain this substream to match it.
	 */
	if (alc5623->master_substream) {
		master_runtime = alc5623->master_substream->runtime;

		if (master_runtime->rate != 0) {
			pr_debug("Constraining to %dHz\n",
				 master_runtime->rate);
			snd_pcm_hw_constraint_minmax(substream->runtime,
						     SNDRV_PCM_HW_PARAM_RATE,
						     master_runtime->rate,
						     master_runtime->rate);
		}

		if (master_runtime->sample_bits != 0) {
			pr_debug("Constraining to %d bits\n",
				 master_runtime->sample_bits);
			snd_pcm_hw_constraint_minmax(substream->runtime,
						     SNDRV_PCM_HW_PARAM_SAMPLE_BITS,
						     master_runtime->sample_bits,
						     master_runtime->sample_bits);
		}

		alc5623->slave_substream = substream;
	} else
		alc5623->master_substream = substream;

	return 0;
}

static void alc5623_pcm_shutdown(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct alc5623_priv *alc5623 = snd_soc_codec_get_drvdata(codec);
	int reg, dig_pwr, ana_pwr;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		alc5623->playback_active--;
	else
		alc5623->capture_active--;

	if (alc5623->master_substream == substream)
		alc5623->master_substream = alc5623->slave_substream;

	alc5623->slave_substream = NULL;

#if 0
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		ana_pwr = alc5623_read(codec, ALC5623_CHIP_ANA_POWER);
		ana_pwr &= ~(ALC5623_ADC_POWERUP | ALC5623_ADC_STEREO);
		alc5623_write(codec, ALC5623_CHIP_ANA_POWER, ana_pwr);
	}

	dig_pwr = alc5623_read(codec, ALC5623_CHIP_DIG_POWER);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dig_pwr &= ~ALC5623_I2S_IN_POWERUP;
	else
		dig_pwr &= ~ALC5623_I2S_OUT_POWERUP;
	alc5623_write(codec, ALC5623_CHIP_DIG_POWER, dig_pwr);

	if (!alc5623->playback_active && !alc5623->capture_active) {
		reg = alc5623_read(codec, ALC5623_CHIP_I2S_CTRL);
		reg &= ~ALC5623_I2S_MASTER;
		alc5623_write(codec, ALC5623_CHIP_I2S_CTRL, reg);
	}
#endif
}

/*
 * Set PCM DAI bit size and sample rate.
 * input: params_rate, params_fmt
 */
static int alc5623_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct alc5623_priv *alc5623 = snd_soc_codec_get_drvdata(codec);
	int channels = params_channels(params);
	int clk_ctl = 0;
	int pll_ctl = 0;
	int i2s_ctl;
	int div2 = 0;
	int reg;
	int sys_fs;

	pr_debug("%s channels=%d\n", __func__, channels);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		alc5623->capture_channels = channels;

	i2sctl &= 0xFFF3;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
printk ("[%s-%d] SNDRV_PCM_FORMAT_S16_LE\n",__FUNCTION__,__LINE__);
		if (alc5623->fmt == SND_SOC_DAIFMT_RIGHT_J)
			return -EINVAL;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
printk ("[%s-%d] SNDRV_PCM_FORMAT_S20_3LE\n",__FUNCTION__,__LINE__);
		i2sctl |= 0x04;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
printk ("[%s-%d] SNDRV_PCM_FORMAT_S24_LE\n",__FUNCTION__,__LINE__);
		i2sctl |= 0x08;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
printk ("[%s-%d] SNDRV_PCM_FORMAT_S32_LE\n",__FUNCTION__,__LINE__);
		if (alc5623->fmt == SND_SOC_DAIFMT_RIGHT_J)
			return -EINVAL;
		i2sctl |= 0x0C;
		break;
	default:
		return -EINVAL;
	}
	alc5623_write(codec, 0x34, i2sctl);

	printk ("[%s-%d] set sampling rate %d\n",__FUNCTION__,__LINE__, params_rate(params));
#if 1
	// 26M MCLK
	if (48000 == params_rate(params)) {
		alc5623_write(codec, 0x44, 0x4227);// PLL control	Fout = 24.6M for 48K & 32K
		alc5623_write(codec, 0x36, 0x1A69);// 
	}
	else if (32000 == params_rate(params)) {
		alc5623_write(codec, 0x44, 0x5028);// PLL control	Fout = 22.667M for 44.1K
		alc5623_write(codec, 0x36, 0x1C6B);// 
	}
	else {
		alc5623_write(codec, 0x44, 0x4428);// MCLK = 13M PLL control	Fout = 22.667M for 44.1K
		alc5623_write(codec, 0x36, 0x1A69);// 
	}
#else
	// 4M MCLK
	if (48000 == params_rate(params)) {
		alc5623_write(codec, 0x44, 0x9221);// PLL control	Fout = 24.6M for 48K
		alc5623_write(codec, 0x36, 0x1A69);// 
	}
	else if (32000 == params_rate(params)) {
		alc5623_write(codec, 0x44, 0x9221);// PLL control	Fout = 24.6M for 32K
		alc5623_write(codec, 0x36, 0x1C6B);// 
	}
	else {
		alc5623_write(codec, 0x44, 0xB322);// PLL control	Fout = 22.667M for 44.1K
		alc5623_write(codec, 0x36, 0x1A69);// 
	}
#endif

	if (alc5623->master)
		alc5623_write(codec, 0x42, 0x8001);// clock source MCLK, PLL from MCLK
	else
		alc5623_write(codec, 0x42, 0xC000);// PLL output, PLL from BCLK

	return 0;
}

static void alc5623_mic_bias(struct snd_soc_codec *codec, int enable)
{
}

static int alc5623_set_bias_level(struct snd_soc_codec *codec,
				   enum snd_soc_bias_level level)
{
	codec->bias_level = level;
	return 0;
}

#define ALC5623_RATES (SNDRV_PCM_RATE_32000 |\
		      SNDRV_PCM_RATE_44100 |\
		      SNDRV_PCM_RATE_48000)

#define ALC5623_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE)

struct snd_soc_dai_ops alc5623_ops = {
	.prepare = alc5623_pcm_prepare,
	.startup = alc5623_pcm_startup,
	.shutdown = alc5623_pcm_shutdown,
	.hw_params = alc5623_pcm_hw_params,
	.digital_mute = alc5623_digital_mute,
	.set_fmt = alc5623_set_dai_fmt,
	.set_sysclk = alc5623_set_dai_sysclk
};

struct snd_soc_dai alc5623_dai = {
	.name = "ALC5623",
	.playback = {
		     .stream_name = "Playback",
		     .channels_min = 2,
		     .channels_max = 2,
		     .rates = ALC5623_RATES,
		     .formats = ALC5623_FORMATS,
		     },
	.capture = {
		    .stream_name = "Capture",
		    .channels_min = 2,
		    .channels_max = 2,
		    .rates = ALC5623_RATES,
		    .formats = ALC5623_FORMATS,
		    },
	.ops = &alc5623_ops,
};
EXPORT_SYMBOL_GPL(alc5623_dai);

/*
 * Delayed work that turns off the audio clock after a delay.
 */
static void alc5623_work(struct work_struct *work)
{
	struct snd_soc_codec *codec =
		container_of(work, struct snd_soc_codec, delayed_work.work);
	struct alc5623_priv *alc5623 = snd_soc_codec_get_drvdata(codec);

	if (!alc5623->need_clk_for_access &&
	    !alc5623->need_clk_for_bias &&
	    alc5623->clock_on) {
		alc5623->clock_enable(0);
		alc5623->clock_on = 0;
	}
}

static int alc5623_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int alc5623_resume(struct platform_device *pdev)
{
	return 0;
}


/*
 * initialise the ALC5623 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int alc5623_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = alc5623_codec;
	struct alc5623_priv *alc5623 = snd_soc_codec_get_drvdata(codec);
	struct alc5623_setup_data *setup = socdev->codec_data;
	u16 reg, ana_pwr, lreg_ctrl, ref_ctrl, lo_ctrl, short_ctrl, sss;
	int vag;
	int ret = 0;
	u32 val;

	socdev->card->codec = alc5623_codec;

	if ((setup != NULL) && (setup->clock_enable != NULL)) {
		alc5623->clock_enable = setup->clock_enable;
		alc5623->need_clk_for_bias = 1;
		INIT_DELAYED_WORK(&codec->delayed_work, alc5623_work);
	}

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "failed to create pcms\n");
		return ret;
	}

#if 0
	val = alc5623_read(codec, 0x7c);	// Get vendor ID
	printk ("[%s-%d] ALC5623 ID %04X\n",__FUNCTION__,__LINE__,val);
	if (0x10EC != val) {
		pr_err("Device with ID register %x is not a ALC5623\n", val);
		ret = -ENODEV;
	}
	val = alc5623_read(codec, 0x7E);
	alc5623->rev = val & 0xFF;
	printk("ALC5623 revision %d\n", alc5623->rev);

	/* set the update bits */
	alc5623_write(codec, 0x3A, 0);	// power management addition 1
	alc5623_write(codec, 0x3C, 0);	// power management addition 2
	alc5623_write(codec, 0x3E, 0);	// power management addition 3
#endif

	snd_soc_add_controls(codec, alc5623_snd_controls,
			     ARRAY_SIZE(alc5623_snd_controls));
	alc5623_add_widgets(codec);

	alc5623_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

/*
 * This function forces any delayed work to be queued and run.
 */
static int run_delayed_work(struct delayed_work *dwork)
{
	int ret;

	/* cancel any work waiting to be queued. */
	ret = cancel_delayed_work(dwork);

	/* if there was any work waiting then we run it now and
	 * wait for it's completion */
	if (ret) {
		schedule_delayed_work(dwork, 0);
		flush_scheduled_work();
	}
	return ret;
}

/* power down chip */
static int alc5623_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	if (codec->control_data)
		alc5623_set_bias_level(codec, SND_SOC_BIAS_OFF);
	run_delayed_work(&codec->delayed_work);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_alc5623 = {
	.probe = alc5623_probe,
	.remove = alc5623_remove,
	.suspend = alc5623_suspend,
	.resume = alc5623_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_alc5623);

static __devinit int alc5623_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct alc5623_priv *alc5623;
	struct snd_soc_codec *codec;
	struct regulator *reg;
	int ret = 0;
	u32 val;

printk ("[%s-%d] ...\n",__func__,__LINE__);
	if (alc5623_codec) {
		dev_err(&client->dev,
			"Multiple ALC5623 devices not supported\n");
		return -ENOMEM;
	}

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	alc5623 = kzalloc(sizeof(struct alc5623_priv), GFP_KERNEL);
	if (alc5623 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	alc5623->clock_enable = (client->dev).platform_data;
		alc5623->need_clk_for_bias = 1;
	snd_soc_codec_set_drvdata(codec, alc5623);
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	i2c_set_clientdata(client, codec);
	codec->control_data = client;

	alc5623->vddd = 0; /* use internal regulator */

#if 0
	msleep(1);

	val = alc5623_read(codec, 0x7c);	// Get vendor ID
	printk ("[%s-%d] ALC5623 ID %04X\n",__FUNCTION__,__LINE__,val);
	if (0x10EC != val) {
		pr_err("Device with ID register %x is not a ALC5623\n", val);
		ret = -ENODEV;
		goto err_codec_reg;
	}
	val = alc5623_read(codec, 0x7E);
	alc5623->rev = val & 0xFF;
	dev_info(&client->dev, "ALC5623 revision %d\n", alc5623->rev);
#endif

	codec->dev = &client->dev;
	codec->name = "ALC5623";
	codec->owner = THIS_MODULE;
	codec->read = alc5623_read;
	codec->write = alc5623_write;
	codec->bias_level = SND_SOC_BIAS_OFF;
	codec->set_bias_level = alc5623_set_bias_level;
	codec->dai = &alc5623_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = sizeof(alc5623_regs);
	codec->reg_cache_step = 2;
	codec->reg_cache = (void *)&alc5623_regs;

	alc5623_codec = codec;
	alc5623_dai.dev = &client->dev;

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		goto err_codec_reg;
	}

	ret = snd_soc_register_dai(&alc5623_dai);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAIs: %d\n", ret);
		goto err_codec_reg;
	}

	return 0;

err_codec_reg:
	if (alc5623->reg_vddd)
		regulator_disable(alc5623->reg_vddd);
	if (alc5623->reg_vdda)
		regulator_disable(alc5623->reg_vdda);
	if (alc5623->reg_vddio)
		regulator_disable(alc5623->reg_vddio);
	if (alc5623->reg_vddd)
		regulator_put(alc5623->reg_vddd);
	if (alc5623->reg_vdda)
		regulator_put(alc5623->reg_vdda);
	if (alc5623->reg_vddio)
		regulator_put(alc5623->reg_vddio);
	kfree(alc5623);
	kfree(codec);
	return ret;
}

static __devexit int alc5623_i2c_remove(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	struct alc5623_priv *alc5623 = snd_soc_codec_get_drvdata(codec);

	if (client->dev.platform_data)
		clk_disable((struct clk *)client->dev.platform_data);

	snd_soc_unregister_dai(&alc5623_dai);
	snd_soc_unregister_codec(codec);

	if (alc5623->reg_vddio) {
		regulator_disable(alc5623->reg_vddio);
		regulator_put(alc5623->reg_vddio);
	}
	if (alc5623->reg_vddd) {
		regulator_disable(alc5623->reg_vddd);
		regulator_put(alc5623->reg_vddd);
	}
	if (alc5623->reg_vdda) {
		regulator_disable(alc5623->reg_vdda);
		regulator_put(alc5623->reg_vdda);
	}

	kfree(codec);
	kfree(alc5623);
	alc5623_codec = NULL;
	return 0;
}

static const struct i2c_device_id alc5623_id[] = {
	{"alc5623-i2c", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, alc5623_id);

static struct i2c_driver alc5623_i2c_driver = {
	.driver = {
		   .name = "alc5623-i2c",
		   .owner = THIS_MODULE,
		   },
	.probe = alc5623_i2c_probe,
	.remove = __devexit_p(alc5623_i2c_remove),
	.id_table = alc5623_id,
};

static int __init alc5623_modinit(void)
{
	return i2c_add_driver(&alc5623_i2c_driver);
}
module_init(alc5623_modinit);

static void __exit alc5623_exit(void)
{
	i2c_del_driver(&alc5623_i2c_driver);
}
module_exit(alc5623_exit);

MODULE_DESCRIPTION("ASoC ALC5623 driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
