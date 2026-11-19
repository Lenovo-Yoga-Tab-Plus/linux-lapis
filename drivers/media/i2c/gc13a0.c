// SPDX-License-Identifier: GPL-2.0
/*
 * GalaxyCore GC13A0 / GC13A2 sensor driver for Lenovo TB520FU (lapis).
 *
 * GC13A0 is the main rear camera (7-bit 0x11, id 0x13a0), GC13A2 the front
 * camera (7-bit 0x39, id 0x13a2; the 0x13ac/0x13ad in the stock tuning only
 * tell the two module vendors apart). Register tables, timing and control limits
 * are decoded from the stock CamX sensormodules (see gc13a0_mode_tables.h and
 * gc13a2_mode_tables.h): RAW10 over 4 lanes with a 19.2 MHz MCLK, as
 * 4208x3120 or half-size 2104x1560, both at 30 fps.
 *
 * The GC13A0 GRBG Bayer order and test pattern indices were verified on
 * hardware with the solid red/green/blue and colour bar patterns.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define GC13A0_REG_CHIP_ID		CCI_REG16(0x03f0)
#define GC13A0_CHIP_ID			0x13a0
#define GC13A2_CHIP_ID			0x13a2
#define GC13A0_STREAMING_REG		CCI_REG8(0x0100)
#define GC13A0_REG_GROUP_HOLD		CCI_REG8(0x0104)

#define GC13A0_REG_TEST_PATTERN_EN	CCI_REG8(0x008c)
#define GC13A0_REG_TEST_PATTERN_IDX	CCI_REG8(0x008d)

#define GC13A0_REG_EXPOSURE		CCI_REG16(0x0202)
#define GC13A0_EXPOSURE_MIN		4
#define GC13A0_EXPOSURE_MARGIN		16
#define GC13A0_EXPOSURE_DEFAULT		0x0b64

/*
 * GC13A0 gain, 1024 = 1x. CamX stops at 16x (its maxAnalogGain), but the
 * register keeps scaling the output linearly up to 0xffff with the black
 * level unchanged (measured: 2x and 4x the 16x signal at 0x8000 and 0xffff).
 * The software ISP this board uses has no digital gain of its own, so the
 * full range is exposed to let auto-exposure reach dim scenes at 30 fps.
 */
#define GC13A0_REG_ANALOG_GAIN		CCI_REG16(0x0204)
#define GC13A0_AGAIN_MIN		1024
#define GC13A0_AGAIN_MAX		0xffff

/*
 * GC13A2 gain, three bytes, 8192 = 1x. CamX stops at 8x; the output keeps
 * doubling up to 32x and collapses to the black level at 64x.
 */
#define GC13A2_REG_ANALOG_GAIN		CCI_REG24(0x0204)
#define GC13A2_AGAIN_MIN		8192
#define GC13A2_AGAIN_MAX		(32 * 8192)

#define GC13A0_REG_FRAME_LENGTH		CCI_REG16(0x0340)
#define GC13A0_FRAME_LENGTH_MAX		0xfffe

#define GC13A0_NATIVE_WIDTH		4208
#define GC13A0_NATIVE_HEIGHT		3120

#define GC13A0_DEFAULT_CLK_FREQ		(19200000)
#define GC13A0_MBUS_CODE		MEDIA_BUS_FMT_SGRBG10_1X10

/*
 * CamX timing, the same for both modes of a sensor.
 * GC13A0: 5850 pixel clocks per line and 3240 lines per frame at 30 fps, i.e.
 * a 568.62 MHz pixel array clock.
 * GC13A2: 4224 pixel clocks per line and 3248 lines per frame at 30.09 fps,
 * i.e. 412.8 MHz.
 */
#define GC13A0_HTS			5850
#define GC13A0_PIXEL_RATE		(GC13A0_HTS * 3240 * 30LL)
#define GC13A2_HTS			4224
#define GC13A2_PIXEL_RATE		412800000LL
#define GC13A0_SLEEP_US			(2 * USEC_PER_MSEC)

static const char *const gc13a0_supply_name[] = {
	"avdd",
	"dvdd",
	"dovdd",
};

static const s64 gc13a0_link_freq_menu[] = {
	590400000LL,
	302400000LL,
};

static const s64 gc13a2_link_freq_menu[] = {
	595200000LL,
	297600000LL,
};

static const char * const gc13a0_test_pattern_menu[] = {
	"Disabled", "Solid Black", "Colour Bars", "Solid White",
	"Solid Red", "Solid Green", "Solid Blue",
};

static const u8 gc13a0_test_pattern_idx[] = {
	0x00, 0x00, 0x10, 0x04, 0x05, 0x06, 0x07,
};

#include "gc13a0_mode_tables.h"
#include "gc13a2_mode_tables.h"

struct gc13a0_mode {
	u32 width;
	u32 height;
	u32 vts_def;
	u32 link_freq_index;
	const struct cci_reg_sequence *regs;
	u32 num_regs;
};

struct gc13a0_variant {
	const char *name;
	u16 chip_id;
	u32 hts;
	s64 pixel_rate;
	const s64 *link_freqs;
	u32 num_link_freqs;
	const struct gc13a0_mode *modes;
	u32 num_modes;
	const struct cci_reg_sequence *common_regs;
	u32 num_common_regs;
	/* Analogue gain register and its limits. */
	u32 gain_reg;
	u32 gain_min;
	u32 gain_max;
	u8 stream_off;
	bool has_test_pattern;
};

static const struct gc13a0_mode gc13a0_modes[] = {
	{
		.width = 4208,
		.height = 3120,
		.vts_def = 3240,
		.link_freq_index = 0,
		.regs = gc13a0_mode_4208x3120,
		.num_regs = ARRAY_SIZE(gc13a0_mode_4208x3120),
	}, {
		/* 2x2 binned, full field of view. */
		.width = 2104,
		.height = 1560,
		.vts_def = 3240,
		.link_freq_index = 1,
		.regs = gc13a0_mode_2104x1560,
		.num_regs = ARRAY_SIZE(gc13a0_mode_2104x1560),
	},
};

static const struct gc13a0_mode gc13a2_modes[] = {
	{
		.width = 4208,
		.height = 3120,
		.vts_def = 3248,
		.link_freq_index = 0,
		.regs = gc13a2_mode_4208x3120,
		.num_regs = ARRAY_SIZE(gc13a2_mode_4208x3120),
	}, {
		.width = 2104,
		.height = 1560,
		.vts_def = 3240,
		.link_freq_index = 1,
		.regs = gc13a2_mode_2104x1560,
		.num_regs = ARRAY_SIZE(gc13a2_mode_2104x1560),
	},
};

static const struct gc13a0_variant gc13a0_variant = {
	.name = "gc13a0",
	.chip_id = GC13A0_CHIP_ID,
	.hts = GC13A0_HTS,
	.pixel_rate = GC13A0_PIXEL_RATE,
	.link_freqs = gc13a0_link_freq_menu,
	.num_link_freqs = ARRAY_SIZE(gc13a0_link_freq_menu),
	.modes = gc13a0_modes,
	.num_modes = ARRAY_SIZE(gc13a0_modes),
	.common_regs = gc13a0_mode_common,
	.num_common_regs = ARRAY_SIZE(gc13a0_mode_common),
	.gain_reg = GC13A0_REG_ANALOG_GAIN,
	.gain_min = GC13A0_AGAIN_MIN,
	.gain_max = GC13A0_AGAIN_MAX,
	.stream_off = 0x00,
	.has_test_pattern = true,
};

static const struct gc13a0_variant gc13a2_variant = {
	.name = "gc13a2",
	.chip_id = GC13A2_CHIP_ID,
	.hts = GC13A2_HTS,
	.pixel_rate = GC13A2_PIXEL_RATE,
	.link_freqs = gc13a2_link_freq_menu,
	.num_link_freqs = ARRAY_SIZE(gc13a2_link_freq_menu),
	.modes = gc13a2_modes,
	.num_modes = ARRAY_SIZE(gc13a2_modes),
	.common_regs = gc13a2_mode_common,
	.num_common_regs = ARRAY_SIZE(gc13a2_mode_common),
	.gain_reg = GC13A2_REG_ANALOG_GAIN,
	.gain_min = GC13A2_AGAIN_MIN,
	.gain_max = GC13A2_AGAIN_MAX,
	.stream_off = 0x80,
};

struct gc13a0 {
	struct device *dev;
	const struct gc13a0_variant *variant;
	const struct gc13a0_mode *mode;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct clk *xclk;
	struct regulator_bulk_data supplies[ARRAY_SIZE(gc13a0_supply_name)];
	struct gpio_desc *reset_gpio;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *link_freq;
	struct regmap *regmap;
	bool streaming;
};

static inline struct gc13a0 *to_gc13a0(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gc13a0, sd);
}

static int gc13a0_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gc13a0 *gc13a0 = to_gc13a0(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(gc13a0_supply_name),
				    gc13a0->supplies);
	if (ret)
		return ret;

	ret = clk_prepare_enable(gc13a0->xclk);
	if (ret) {
		regulator_bulk_disable(ARRAY_SIZE(gc13a0_supply_name),
				       gc13a0->supplies);
		return ret;
	}

	fsleep(GC13A0_SLEEP_US);
	gpiod_set_value_cansleep(gc13a0->reset_gpio, 0);
	fsleep(GC13A0_SLEEP_US);

	return 0;
}

static int gc13a0_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gc13a0 *gc13a0 = to_gc13a0(sd);

	gpiod_set_value_cansleep(gc13a0->reset_gpio, 1);
	clk_disable_unprepare(gc13a0->xclk);
	regulator_bulk_disable(ARRAY_SIZE(gc13a0_supply_name),
			       gc13a0->supplies);

	return 0;
}

static int gc13a0_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = GC13A0_MBUS_CODE;
	return 0;
}

static int gc13a0_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct gc13a0 *gc13a0 = to_gc13a0(sd);

	if (fse->index >= gc13a0->variant->num_modes ||
	    fse->code != GC13A0_MBUS_CODE)
		return -EINVAL;

	fse->min_width = gc13a0->variant->modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = gc13a0->variant->modes[fse->index].height;
	fse->max_height = fse->min_height;
	return 0;
}

/* Called with the control handler lock held. */
static int gc13a0_update_mode_controls(struct gc13a0 *gc13a0,
				       const struct gc13a0_mode *mode)
{
	u32 hblank = gc13a0->variant->hts - mode->width;
	u32 vblank = mode->vts_def - mode->height;
	int ret;

	ret = __v4l2_ctrl_s_ctrl(gc13a0->link_freq, mode->link_freq_index);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(gc13a0->hblank, hblank, hblank, 1,
				       hblank);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(gc13a0->vblank, vblank,
				       GC13A0_FRAME_LENGTH_MAX - mode->height,
				       1, vblank);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl(gc13a0->vblank, vblank);
	if (ret)
		return ret;

	return __v4l2_ctrl_modify_range(gc13a0->exposure,
					gc13a0->exposure->minimum,
					mode->vts_def - GC13A0_EXPOSURE_MARGIN,
					1, GC13A0_EXPOSURE_DEFAULT);
}

static int gc13a0_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	struct gc13a0 *gc13a0 = to_gc13a0(sd);
	struct v4l2_mbus_framefmt *format;
	const struct gc13a0_mode *mode;
	int ret;

	mode = v4l2_find_nearest_size(gc13a0->variant->modes,
				      gc13a0->variant->num_modes, width, height,
				      fmt->format.width, fmt->format.height);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		if (gc13a0->streaming)
			return -EBUSY;

		if (mode != gc13a0->mode) {
			/* The control handlers look at the current mode. */
			gc13a0->mode = mode;
			ret = gc13a0_update_mode_controls(gc13a0, mode);
			if (ret)
				return ret;
		}
	}

	format = v4l2_subdev_state_get_format(state, 0);
	format->width = mode->width;
	format->height = mode->height;
	format->code = GC13A0_MBUS_CODE;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;
	fmt->format = *format;
	return 0;
}

static int gc13a0_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = GC13A0_NATIVE_WIDTH;
		sel->r.height = GC13A0_NATIVE_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int gc13a0_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct gc13a0 *gc13a0 = to_gc13a0(sd);
	struct v4l2_mbus_framefmt *format;

	/* Both the try and the active state start in the default mode. */
	format = v4l2_subdev_state_get_format(state, 0);
	format->width = gc13a0->variant->modes[0].width;
	format->height = gc13a0->variant->modes[0].height;
	format->code = GC13A0_MBUS_CODE;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;
	return 0;
}

static int gc13a0_start_streaming(struct gc13a0 *gc13a0)
{
	const struct gc13a0_variant *variant = gc13a0->variant;
	const struct gc13a0_mode *mode = gc13a0->mode;
	int ret;

	ret = pm_runtime_resume_and_get(gc13a0->dev);
	if (ret < 0)
		return ret;

	ret = cci_write(gc13a0->regmap, GC13A0_STREAMING_REG,
			variant->stream_off, NULL);
	if (ret) {
		dev_err(gc13a0->dev, "stream off before init failed: %d\n", ret);
		goto err_rpm;
	}

	ret = cci_multi_reg_write(gc13a0->regmap, variant->common_regs,
				  variant->num_common_regs, NULL);
	if (ret) {
		dev_err(gc13a0->dev, "common mode write failed: %d\n", ret);
		goto err_rpm;
	}

	ret = cci_multi_reg_write(gc13a0->regmap, mode->regs, mode->num_regs,
				  NULL);
	if (ret) {
		dev_err(gc13a0->dev, "%ux%u mode write failed: %d\n",
			mode->width, mode->height, ret);
		goto err_rpm;
	}

	ret = __v4l2_ctrl_handler_setup(&gc13a0->ctrls);
	if (ret) {
		dev_err(gc13a0->dev, "applying controls failed: %d\n", ret);
		goto err_rpm;
	}

	ret = cci_write(gc13a0->regmap, GC13A0_STREAMING_REG, 1, NULL);
	if (ret) {
		dev_err(gc13a0->dev, "stream on failed: %d\n", ret);
		goto err_rpm;
	}

	gc13a0->streaming = true;
	return 0;

err_rpm:
	pm_runtime_put(gc13a0->dev);
	return ret;
}

static int gc13a0_stop_streaming(struct gc13a0 *gc13a0)
{
	int ret;

	ret = cci_write(gc13a0->regmap, GC13A0_STREAMING_REG,
			gc13a0->variant->stream_off, NULL);
	if (ret)
		dev_err(gc13a0->dev, "stream off failed: %d\n", ret);

	gc13a0->streaming = false;
	pm_runtime_put(gc13a0->dev);
	return ret;
}

static int gc13a0_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct gc13a0 *gc13a0 = to_gc13a0(sd);
	struct v4l2_subdev_state *state;
	int ret = 0;

	/* The state lock is the control handler lock. */
	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (gc13a0->streaming == !!enable)
		goto unlock;

	if (enable)
		ret = gc13a0_start_streaming(gc13a0);
	else
		ret = gc13a0_stop_streaming(gc13a0);

unlock:
	v4l2_subdev_unlock_state(state);
	return ret;
}

static const struct v4l2_subdev_video_ops gc13a0_video_ops = {
	.s_stream = gc13a0_set_stream,
};

static const struct v4l2_subdev_pad_ops gc13a0_pad_ops = {
	.enum_mbus_code = gc13a0_enum_mbus_code,
	.enum_frame_size = gc13a0_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = gc13a0_set_format,
	.get_selection = gc13a0_get_selection,
};

static const struct v4l2_subdev_ops gc13a0_subdev_ops = {
	.video = &gc13a0_video_ops,
	.pad = &gc13a0_pad_ops,
};

static const struct v4l2_subdev_internal_ops gc13a0_internal_ops = {
	.init_state = gc13a0_init_state,
};

static int gc13a0_parse_fwnode(struct gc13a0 *gc13a0)
{
	struct v4l2_fwnode_endpoint vep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct fwnode_handle *endpoint;
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(gc13a0->dev), NULL);
	if (!endpoint)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &vep);
	fwnode_handle_put(endpoint);
	if (ret)
		return ret;

	if (vep.bus.mipi_csi2.num_data_lanes != 4)
		dev_warn(gc13a0->dev, "expected 4 data lanes, got %u\n",
			 vep.bus.mipi_csi2.num_data_lanes);

	v4l2_fwnode_endpoint_free(&vep);
	return 0;
}

static int gc13a0_identify(struct gc13a0 *gc13a0)
{
	u64 val;
	int ret;

	ret = cci_read(gc13a0->regmap, GC13A0_REG_CHIP_ID, &val, NULL);
	if (ret)
		return ret;

	if (val != gc13a0->variant->chip_id) {
		dev_err(gc13a0->dev, "chip id mismatch: 0x%x != 0x%llx\n",
			gc13a0->variant->chip_id, val);
		return -ENXIO;
	}

	dev_info(gc13a0->dev, "GalaxyCore sensor 0x%llx detected\n", val);
	return 0;
}

/*
 * Frame length, exposure and gain are programmed together under group hold
 * so that they take effect on the same frame.
 */
static int gc13a0_write_exposure(struct gc13a0 *gc13a0)
{
	int ret = 0;

	cci_write(gc13a0->regmap, GC13A0_REG_GROUP_HOLD, 1, &ret);
	cci_write(gc13a0->regmap, GC13A0_REG_FRAME_LENGTH,
		  gc13a0->mode->height + gc13a0->vblank->val, &ret);
	cci_write(gc13a0->regmap, GC13A0_REG_EXPOSURE, gc13a0->exposure->val,
		  &ret);
	cci_write(gc13a0->regmap, gc13a0->variant->gain_reg, gc13a0->gain->val,
		  &ret);
	/* Release the hold even if a write failed. */
	cci_write(gc13a0->regmap, GC13A0_REG_GROUP_HOLD, 0, NULL);

	return ret;
}

static int gc13a0_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc13a0 *gc13a0 = container_of(ctrl->handler, struct gc13a0, ctrls);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		/* The frame length bounds the exposure time. */
		s64 max = gc13a0->mode->height + ctrl->val -
			  GC13A0_EXPOSURE_MARGIN;

		ret = __v4l2_ctrl_modify_range(gc13a0->exposure,
					       gc13a0->exposure->minimum, max, 1,
					       min_t(s64, max, GC13A0_EXPOSURE_DEFAULT));
		if (ret)
			return ret;
	}

	/* Controls are applied from the cache when streaming starts. */
	if (!pm_runtime_get_if_active(gc13a0->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_ANALOGUE_GAIN:
	case V4L2_CID_VBLANK:
		ret = gc13a0_write_exposure(gc13a0);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(gc13a0->regmap, GC13A0_REG_TEST_PATTERN_IDX,
			  gc13a0_test_pattern_idx[ctrl->val], &ret);
		cci_write(gc13a0->regmap, GC13A0_REG_TEST_PATTERN_EN,
			  ctrl->val ? 1 : 0, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(gc13a0->dev);
	return ret;
}

static const struct v4l2_ctrl_ops gc13a0_ctrl_ops = {
	.s_ctrl = gc13a0_set_ctrl,
};

static int gc13a0_init_controls(struct gc13a0 *gc13a0)
{
	struct v4l2_ctrl_handler *hdl = &gc13a0->ctrls;
	struct v4l2_fwnode_device_properties props;
	const struct gc13a0_variant *variant = gc13a0->variant;
	const struct gc13a0_mode *mode = gc13a0->mode;
	const u32 vblank = mode->vts_def - mode->height;
	const u32 hblank = variant->hts - mode->width;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 9);
	if (ret)
		return ret;

	gc13a0->link_freq =
		v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ,
				       variant->num_link_freqs - 1,
				       mode->link_freq_index,
				       variant->link_freqs);
	if (gc13a0->link_freq)
		gc13a0->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, variant->pixel_rate,
			  variant->pixel_rate, 1, variant->pixel_rate);

	gc13a0->hblank = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK, hblank,
					   hblank, 1, hblank);
	if (gc13a0->hblank)
		gc13a0->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	gc13a0->vblank =
		v4l2_ctrl_new_std(hdl, &gc13a0_ctrl_ops, V4L2_CID_VBLANK, vblank,
				  GC13A0_FRAME_LENGTH_MAX - mode->height,
				  1, vblank);

	gc13a0->exposure =
		v4l2_ctrl_new_std(hdl, &gc13a0_ctrl_ops, V4L2_CID_EXPOSURE,
				  GC13A0_EXPOSURE_MIN,
				  mode->vts_def - GC13A0_EXPOSURE_MARGIN, 1,
				  GC13A0_EXPOSURE_DEFAULT);

	gc13a0->gain =
		v4l2_ctrl_new_std(hdl, &gc13a0_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
				  variant->gain_min, variant->gain_max, 1,
				  variant->gain_min);

	if (variant->has_test_pattern)
		v4l2_ctrl_new_std_menu_items(hdl, &gc13a0_ctrl_ops,
					     V4L2_CID_TEST_PATTERN,
					     ARRAY_SIZE(gc13a0_test_pattern_menu) - 1,
					     0, 0, gc13a0_test_pattern_menu);

	ret = v4l2_fwnode_device_parse(gc13a0->dev, &props);
	if (!ret)
		v4l2_ctrl_new_fwnode_properties(hdl, &gc13a0_ctrl_ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	gc13a0->sd.ctrl_handler = hdl;
	return 0;
}

static int gc13a0_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gc13a0 *gc13a0;
	unsigned int i;
	int ret;

	gc13a0 = devm_kzalloc(dev, sizeof(*gc13a0), GFP_KERNEL);
	if (!gc13a0)
		return -ENOMEM;

	gc13a0->dev = dev;
	gc13a0->variant = i2c_get_match_data(client);
	if (!gc13a0->variant)
		return -ENODEV;
	gc13a0->mode = &gc13a0->variant->modes[0];

	ret = gc13a0_parse_fwnode(gc13a0);
	if (ret)
		return ret;

	gc13a0->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(gc13a0->regmap))
		return PTR_ERR(gc13a0->regmap);

	gc13a0->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(gc13a0->xclk))
		return dev_err_probe(dev, PTR_ERR(gc13a0->xclk),
				     "failed to get xclk\n");

	if (clk_get_rate(gc13a0->xclk) != GC13A0_DEFAULT_CLK_FREQ)
		dev_warn(dev, "xclk %lu Hz (expected %u)\n",
			 clk_get_rate(gc13a0->xclk), GC13A0_DEFAULT_CLK_FREQ);

	for (i = 0; i < ARRAY_SIZE(gc13a0_supply_name); i++)
		gc13a0->supplies[i].supply = gc13a0_supply_name[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(gc13a0_supply_name),
				      gc13a0->supplies);
	if (ret)
		return ret;

	gc13a0->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(gc13a0->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(gc13a0->reset_gpio),
				     "failed to get reset gpio\n");

	v4l2_i2c_subdev_init(&gc13a0->sd, client, &gc13a0_subdev_ops);
	/* Userspace picks its sensor tuning by entity name. */
	v4l2_i2c_subdev_set_name(&gc13a0->sd, client, gc13a0->variant->name,
				 NULL);
	gc13a0->sd.internal_ops = &gc13a0_internal_ops;

	ret = gc13a0_power_on(dev);
	if (ret)
		return ret;

	ret = gc13a0_identify(gc13a0);
	if (ret)
		goto err_power_off;

	ret = gc13a0_init_controls(gc13a0);
	if (ret)
		goto err_power_off;

	gc13a0->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	gc13a0->pad.flags = MEDIA_PAD_FL_SOURCE;
	gc13a0->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&gc13a0->sd.entity, 1, &gc13a0->pad);
	if (ret)
		goto err_ctrls;

	gc13a0->sd.state_lock = gc13a0->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&gc13a0->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	ret = v4l2_async_register_subdev_sensor(&gc13a0->sd);
	if (ret)
		goto err_rpm;

	return 0;

err_rpm:
	pm_runtime_disable(dev);
	v4l2_subdev_cleanup(&gc13a0->sd);
err_entity:
	media_entity_cleanup(&gc13a0->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&gc13a0->ctrls);
err_power_off:
	gc13a0_power_off(dev);
	return ret;
}

static void gc13a0_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc13a0 *gc13a0 = to_gc13a0(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&gc13a0->ctrls);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		gc13a0_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id gc13a0_of_match[] = {
	{ .compatible = "galaxycore,gc13a0", .data = &gc13a0_variant },
	{ .compatible = "galaxycore,gc13a2", .data = &gc13a2_variant },
	{ }
};
MODULE_DEVICE_TABLE(of, gc13a0_of_match);

static DEFINE_RUNTIME_DEV_PM_OPS(gc13a0_pm_ops, gc13a0_power_off,
				 gc13a0_power_on, NULL);

static struct i2c_driver gc13a0_i2c_driver = {
	.driver = {
		.name = "gc13a0",
		.pm = pm_ptr(&gc13a0_pm_ops),
		.of_match_table = gc13a0_of_match,
	},
	.probe = gc13a0_probe,
	.remove = gc13a0_remove,
};
module_i2c_driver(gc13a0_i2c_driver);

MODULE_DESCRIPTION("GalaxyCore GC13A0/GC13A2 camera sensor");
MODULE_LICENSE("GPL");
