// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for the OV7695 camera sensor.
 *
 * Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015 By Tech Design S.L. All Rights Reserved.
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Based on:
 * - the OV7695 driver from QC msm-3.10 kernel on codeaurora.org:
 *   https://us.codeaurora.org/cgit/quic/la/kernel/msm-3.10/tree/drivers/
 *       media/platform/msm/camera_v2/sensor/ov7695.c?h=LA.BR.1.2.4_rb1.41
 * - the OV5640 driver posted on linux-media:
 *   https://www.mail-archive.com/linux-media%40vger.kernel.org/msg92671.html
 */

/*
 */
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define AD_DEV_DBG dev_dbg

#define OV7695_SENSOR_ID 0x7695

// Starts/stops streaming
#define OV7695_MODE_DELECT		0x0100
#define		OV7695_MODE_DELECT_START	0x01
#define		OV7695_MODE_DELECT_STOP		0x00

#define OV7695_CHIP_ID_HIGH		0x300a
#define		OV7695_CHIP_ID_HIGH_BYTE	0x76
#define OV7695_CHIP_ID_LOW		0x300b
#define		OV7695_CHIP_ID_LOW_BYTE		0x95

static __u32 s_v4l2_subdev_mbus_code = MEDIA_BUS_FMT_YUYV8_1X16;

enum ov7695_frame_rate {
	OV7695_30_FPS = 0,
	OV7695_60_FPS,
	OV7695_120_FPS,
	OV7695_NUM_FRAMERATES,
};

static const int ov7695_framerates[] = {
	[OV7695_30_FPS] = 30,
	[OV7695_60_FPS] = 60,
	[OV7695_120_FPS] = 120,
};

/* regulator supplies */
static const char * const ov7695_supply_name[] = {
	"vdddo", /* Digital I/O (1.8V) supply */
	"vdda",  /* Analog (2.8V) supply */
	"vddd",  /* Digital Core (1.5V) supply */
};

#define OV7695_NUM_SUPPLIES ARRAY_SIZE(ov7695_supply_name)

struct reg_value {
	u16 reg;
	u8 val;
};

struct ov7695_mode_info {
	u32 width;
	u32 height;
	const struct reg_value *data;
	u32 data_size;
	u32 pixel_clock;
	u32 link_freq;

	/* Used by s_frame_interval only. */
	u32 max_fps;
	u32 def_fps;
};

struct ov7695 {
	struct i2c_client *i2c_client;
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_rect crop;
	struct clk *xclk;

	struct regulator_bulk_data supplies[OV7695_NUM_SUPPLIES];

	const struct ov7695_mode_info *current_mode;
	enum ov7695_frame_rate current_fr;
	struct v4l2_fract frame_interval;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *pixel_clock;
	struct v4l2_ctrl *link_freq;

	/* Cached register values */
	u8 aec_pk_manual;
	u8 timing_tc_reg20;
	u8 timing_tc_reg21;

	struct mutex power_lock; /* lock to protect power state */
	int power_count;

	struct gpio_desc *enable_gpio;
	struct gpio_desc *rst_gpio;
	struct gpio_desc *mclk_enable_gpio;

	u8 external_power_supplies;

	bool pending_mode_change;
	bool streaming;
};

static inline struct ov7695 *to_ov7695(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov7695, sd);
}

#define OV7695_USE_GLOBAL_INIT_SETTINGS 0
#if OV7695_USE_GLOBAL_INIT_SETTINGS
static const struct reg_value ov7695_global_init_setting[] = {
};
#endif

static struct reg_value ov7695_setting_vga[] = {
	{0x0103, 0x01,},
	{0x3620, 0x2f,},
	{0x3623, 0x12,},
    {0x3718, 0x88,},
    {0x3703, 0x80,},
    {0x3712, 0x40,},
    {0x3706, 0x40,},
    {0x3631, 0x44,},
    {0x3632, 0x05,},
    {0x3013, 0xd0,},
    {0x3705, 0x1d,},
    {0x3713, 0x0e,},
    {0x3012, 0x0a,},
    {0x3717, 0x18,}, //0x19
    {0x3621, 0x47,}, //0x44
    {0x0309, 0x24,},
    {0x3820, 0x90,},
    {0x4803, 0x08,},
    {0x0101, 0x01,},
    {0x5100, 0x01,},
    {0x4500, 0x24,}, //0x25
    {0x5301, 0x05,},
    {0x5302, 0x0c,},
    {0x5303, 0x1c,},
    {0x5304, 0x2a,},
    {0x5305, 0x39,},
    {0x5306, 0x45,},
    {0x5307, 0x52,},
    {0x5308, 0x5d,},
    {0x5309, 0x68,},
    {0x530a, 0x7f,},
    {0x530b, 0x91,},
    {0x530c, 0xa5,},
    {0x530d, 0xc6,},
    {0x530e, 0xde,},
    {0x530f, 0xef,},
    {0x5310, 0x16,},
    {0x520a, 0xf4,},
    {0x520b, 0xf4,},
    {0x520c, 0xf4,},
    {0x5504, 0x08,},
    {0x5505, 0x48,},
    {0x5506, 0x07,},
    {0x5507, 0x0b,},
    {0x3a18, 0x01,},
    {0x3a19, 0x00,},
    {0x3503, 0x03,},
    {0x3500, 0x00,},
    {0x3501, 0x21,},
    {0x3502, 0x00,},
    {0x350a, 0x00,},
    {0x350b, 0x00,},
    {0x4008, 0x02,},
    {0x4009, 0x09,},
    {0x3002, 0x09,},
    {0x3024, 0x00,},
    {0x3503, 0x00,},
	{0x4300, 0x30,}, // Setting output format: YUYV : 0x3f(default) or 0x32; YVYU : 0x33; VYUY : 0x31; UYVY : 0x30;
};

static const s64 link_freq[] = {
	240000000
};

static const struct ov7695_mode_info ov7695_mode_info_data[] = {
	{
		.width = 640,
		.height = 480,
		.data = ov7695_setting_vga,
		.data_size = ARRAY_SIZE(ov7695_setting_vga),
		.pixel_clock = 48000000,
		.link_freq = 0, /* an index in link_freq[] */
		.max_fps	= OV7695_30_FPS,
		.def_fps	= OV7695_30_FPS
	},
};

static int ov7695_write_reg(struct ov7695 *ov7695, u16 reg, u8 val)
{
	u8 regbuf[3];
	int ret;

	regbuf[0] = reg >> 8;
	regbuf[1] = reg & 0xff;
	regbuf[2] = val;

	ret = i2c_master_send(ov7695->i2c_client, regbuf, 3);
	if (ret < 0) {
		dev_err(ov7695->dev, "%s: write reg error %d: reg=%x, val=%x\n",
			__func__, ret, reg, val);
		return ret;
	}

	return 0;
}

static int ov7695_read_reg(struct ov7695 *ov7695, u16 reg, u8 *val)
{
	u8 regbuf[2];
	int ret;

	regbuf[0] = reg >> 8;
	regbuf[1] = reg & 0xff;

	ret = i2c_master_send(ov7695->i2c_client, regbuf, 2);
	if (ret < 0) {
		dev_err(ov7695->dev, "%s: write reg error %d: reg=%x\n",
			__func__, ret, reg);
		return ret;
	}

	ret = i2c_master_recv(ov7695->i2c_client, val, 1);
	if (ret < 0) {
		dev_err(ov7695->dev, "%s: read reg error %d: reg=%x\n",
			__func__, ret, reg);
		return ret;
	}

	return 0;
}

static int ov7695_set_register_array(struct ov7695 *ov7695,
				     const struct reg_value *settings,
				     unsigned int num_settings)
{
	unsigned int i;
	int ret;

	for (i = 0; i < num_settings; ++i, ++settings) {
		ret = ov7695_write_reg(ov7695, settings->reg, settings->val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ov7695_set_power_on(struct ov7695 *ov7695)
{
	if (!ov7695->external_power_supplies) {
		int ret;
		ret = regulator_bulk_enable(OV7695_NUM_SUPPLIES, ov7695->supplies);
		if (ret < 0)
			return ret;
	}

	if (!IS_ERR_OR_NULL(ov7695->xclk)) {
		int ret;
		ret = clk_prepare_enable(ov7695->xclk);
		if (ret < 0) {
			dev_err(ov7695->dev, "clk prepare enable failed\n");
			if (!ov7695->external_power_supplies) {
				regulator_bulk_disable(OV7695_NUM_SUPPLIES, ov7695->supplies);
			}
			return ret;
		}
	}

	gpiod_set_value(ov7695->enable_gpio, 1);

	if (!IS_ERR_OR_NULL(ov7695->mclk_enable_gpio)) {
		gpiod_set_value(ov7695->mclk_enable_gpio, 1);
	}
	msleep(350);

	return 0;
}

static void ov7695_set_power_off(struct ov7695 *ov7695)
{
	usleep_range(25, 100);
	if (!IS_ERR_OR_NULL(ov7695->mclk_enable_gpio)) {
		gpiod_set_value(ov7695->mclk_enable_gpio, 0);
	}
	usleep_range(10, 100);

	gpiod_set_value(ov7695->enable_gpio, 0);


	if (!IS_ERR_OR_NULL(ov7695->xclk)) {
		clk_disable_unprepare(ov7695->xclk);
	}
	if (!ov7695->external_power_supplies) {
		regulator_bulk_disable(OV7695_NUM_SUPPLIES, ov7695->supplies);
	}
	msleep(10);
}

static int ov7695_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov7695 *ov7695 = to_ov7695(sd);
	int ret = 0;

	mutex_lock(&ov7695->power_lock);

	/* If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (ov7695->power_count == !on) {
		if (on) {
			ret = ov7695_set_power_on(ov7695);
			if (ret < 0)
				goto exit;
#if OV7695_USE_GLOBAL_INIT_SETTINGS
			ret = ov7695_set_register_array(ov7695,
					ov7695_global_init_setting,
					ARRAY_SIZE(ov7695_global_init_setting));
			if (ret < 0) {
				dev_err(ov7695->dev,
					"could not set init registers\n");
				ov7695_set_power_off(ov7695);
				goto exit;
			}

			usleep_range(500, 1000);
#endif
		} else {
			ov7695_set_power_off(ov7695);
		}
	}

	/* Update the power count. */
	ov7695->power_count += on ? 1 : -1;
	WARN_ON(ov7695->power_count < 0);

exit:
	mutex_unlock(&ov7695->power_lock);

	return ret;
}

static int ov7695_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;
}

static int ov7695_s_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;
}

static const struct v4l2_ctrl_ops ov7695_ctrl_ops = {
	.g_volatile_ctrl = ov7695_g_volatile_ctrl,
	.s_ctrl = ov7695_s_ctrl,
};

static int ov7695_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = s_v4l2_subdev_mbus_code;

	return 0;
}

static int ov7695_enum_frame_size(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct ov7695 *ov7695 = to_ov7695(subdev);

	AD_DEV_DBG(ov7695->dev, "%s: Enter.  fse->index: %d; fse->code: 0x%x; ARRAY_SIZE(ov7695_mode_info_data): %lu;\n", __func__, fse->index, fse->code, ARRAY_SIZE(ov7695_mode_info_data));
	if (fse->code != MEDIA_BUS_FMT_YUYV8_1X16) {
		AD_DEV_DBG(ov7695->dev, "%s: fse->index: %d; fse->code: 0x%x;   ---> fse->code != MEDIA_BUS_FMT_YUYV8_1X16\n", __func__, fse->index, fse->code);
		return -EINVAL;
	}

	if (fse->index >= ARRAY_SIZE(ov7695_mode_info_data))
		return -EINVAL;

	fse->min_width = ov7695_mode_info_data[fse->index].width;
	fse->max_width = ov7695_mode_info_data[fse->index].width;
	fse->min_height = ov7695_mode_info_data[fse->index].height;
	fse->max_height = ov7695_mode_info_data[fse->index].height;
	fse->code = s_v4l2_subdev_mbus_code;

	AD_DEV_DBG(ov7695->dev, "%s: fse->min_width: %d; fse->max_width: %d; fse->min_height: %d; fse->max_height: %d;\n", __func__, fse->min_width, fse->max_width, fse->min_height, fse->max_height);


	return 0;
}

static struct v4l2_mbus_framefmt *
__ov7695_get_pad_format(struct ov7695 *ov7695,
			struct v4l2_subdev_state *sd_state,
			unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&ov7695->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ov7695->fmt;
	default:
		return NULL;
	}
}

static int ov7695_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_format *format)
{
	struct ov7695 *ov7695 = to_ov7695(sd);

	AD_DEV_DBG(ov7695->dev, "%s: Enter\n", __func__);
	format->format = *__ov7695_get_pad_format(ov7695, sd_state,
						  format->pad,
						  format->which);
	return 0;
}

static struct v4l2_rect *
__ov7695_get_pad_crop(struct ov7695 *ov7695,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&ov7695->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ov7695->crop;
	default:
		return NULL;
	}
}

static int ov7695_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_format *format)
{
	struct ov7695 *ov7695 = to_ov7695(sd);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	const struct ov7695_mode_info *new_mode;
	int ret;

	__crop = __ov7695_get_pad_crop(ov7695, sd_state, format->pad,
				       format->which);

	new_mode = v4l2_find_nearest_size(ov7695_mode_info_data,
			       ARRAY_SIZE(ov7695_mode_info_data),
			       width, height,
			       format->format.width, format->format.height);

	__crop->width = new_mode->width;
	__crop->height = new_mode->height;

	AD_DEV_DBG(ov7695->dev, "%s: __crop->width: %d; __crop->height: %d\n", __func__, __crop->width,  __crop->height);

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		AD_DEV_DBG(ov7695->dev, "%s: format->which == V4L2_SUBDEV_FORMAT_ACTIVE - new_mode->pixel_clock: %d;\n", __func__, new_mode->pixel_clock);
		ret = v4l2_ctrl_s_ctrl_int64(ov7695->pixel_clock,
					     new_mode->pixel_clock);
		if (ret < 0) {
			return ret;
		}

		ret = v4l2_ctrl_s_ctrl(ov7695->link_freq,
				       new_mode->link_freq);
		if (ret < 0) {
			return ret;
		}

		ov7695->current_mode = new_mode;
	}

	__format = __ov7695_get_pad_format(ov7695, sd_state, format->pad,
					   format->which);
	__format->width = __crop->width;
	__format->height = __crop->height;
	__format->code = s_v4l2_subdev_mbus_code;
	__format->field = V4L2_FIELD_NONE;
	__format->colorspace = V4L2_COLORSPACE_SRGB;

	format->format = *__format;

	return 0;
}

static int ov7695_entity_init_cfg(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_state *sd_state)
{
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = sd_state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.format.width = 640;
	fmt.format.height = 480;

	ov7695_set_format(subdev, sd_state, &fmt);

	return 0;
}

static int ov7695_get_selection(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_selection *sel)
{
	struct ov7695 *ov7695 = to_ov7695(sd);

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	sel->r = *__ov7695_get_pad_crop(ov7695, sd_state, sel->pad,
					sel->which);
	return 0;
}

static int ov7695_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct ov7695 *ov7695 = to_ov7695(subdev);
	int ret;

	if (enable) {
		ret = ov7695_set_register_array(ov7695,
					ov7695->current_mode->data,
					ov7695->current_mode->data_size);
		if (ret < 0) {
			dev_err(ov7695->dev, "could not set mode %dx%d\n",
				ov7695->current_mode->width,
				ov7695->current_mode->height);
			return ret;
		}
		ret = v4l2_ctrl_handler_setup(&ov7695->ctrls);
		if (ret < 0) {
			dev_err(ov7695->dev, "could not sync v4l2 controls\n");
			return ret;
		}

		ret = ov7695_write_reg(ov7695, OV7695_MODE_DELECT, OV7695_MODE_DELECT_START);
		if (ret < 0) {
			dev_err(ov7695->dev, "%s: could not start streaming: %d\n", __func__, ret);
			return ret;
		}
	} else {
		ret = ov7695_write_reg(ov7695, OV7695_MODE_DELECT, OV7695_MODE_DELECT_STOP);
		if (ret < 0) {
			dev_err(ov7695->dev, "%s: could not stop streaming: %d\n", __func__, ret);
			return ret;
		}
	}

	return 0;
}


static const struct ov7695_mode_info *
ov7695_find_mode(struct ov7695 *sensor, int width, int height, bool nearest)
{
	const struct ov7695_mode_info *mode;

	mode = v4l2_find_nearest_size(ov7695_mode_info_data,
				      ARRAY_SIZE(ov7695_mode_info_data),
				      width, height, width, height);

	if (!mode ||
	    (!nearest &&
	     (mode->width != width || mode->height != height)))
		return NULL;

	return mode;
}

static int ov7695_try_frame_interval(struct ov7695 *sensor,
				     struct v4l2_fract *fi,
				     u32 width, u32 height)
{
	const struct ov7695_mode_info *mode;
	enum ov7695_frame_rate rate = OV7695_30_FPS;
	int minfps, maxfps, best_fps, fps;
	int i;

	mode = ov7695_find_mode(sensor, width, height, false);
	if (!mode)
		return -EINVAL;

	minfps = ov7695_framerates[OV7695_30_FPS];
	maxfps = ov7695_framerates[mode->max_fps];

	if (fi->numerator == 0) {
		fi->denominator = maxfps;
		fi->numerator = 1;
		rate = mode->max_fps;
		goto find_mode;
	}

	fps = clamp_val(DIV_ROUND_CLOSEST(fi->denominator, fi->numerator),
			minfps, maxfps);

	best_fps = minfps;
	for (i = 0; i < ARRAY_SIZE(ov7695_framerates); i++) {
		int curr_fps = ov7695_framerates[i];

		if (abs(curr_fps - fps) < abs(best_fps - fps)) {
			best_fps = curr_fps;
			rate = i;
		}
	}

	fi->numerator = 1;
	fi->denominator = best_fps;

find_mode:
	mode = ov7695_find_mode(sensor, width, height, false);
	return mode ? rate : -EINVAL;
}
#if 0
static int ov7695_update_pixel_rate(struct ov7695 *sensor)
{
	const struct ov7695_mode_info *mode = sensor->current_mode;
	enum ov5640_pixel_rate_id pixel_rate_id = mode->pixel_rate;
	struct v4l2_mbus_framefmt *fmt = &sensor->fmt;
	const struct ov5640_timings *timings = ov5640_timings(sensor, mode);
	s32 exposure_val, exposure_max;
	unsigned int hblank;
	unsigned int i = 0;
	u32 pixel_rate;
	s64 link_freq;
	u32 num_lanes;
	u32 vblank;
	u32 bpp;

	/*
	 * Update the pixel rate control value.
	 *
	 * For DVP mode, maintain the pixel rate calculation using fixed FPS.
	 */
	if (!ov5640_is_csi2(sensor)) {
		__v4l2_ctrl_s_ctrl_int64(sensor->ctrls.pixel_rate,
					 ov5640_calc_pixel_rate(sensor));

		__v4l2_ctrl_vblank_update(sensor, timings->vblank_def);

		return 0;
	}

	/*
	 * The MIPI CSI-2 link frequency should comply with the CSI-2
	 * specification and be lower than 1GHz.
	 *
	 * Start from the suggested pixel_rate for the current mode and
	 * progressively slow it down if it exceeds 1GHz.
	 */
	num_lanes = sensor->ep.bus.mipi_csi2.num_data_lanes;
	bpp = ov5640_code_to_bpp(sensor, fmt->code);
	do {
		pixel_rate = ov5640_pixel_rates[pixel_rate_id];
		link_freq = pixel_rate * bpp / (2 * num_lanes);
	} while (link_freq >= 1000000000U &&
		 ++pixel_rate_id < OV5640_NUM_PIXEL_RATES);

	sensor->current_link_freq = link_freq;

	/*
	 * Higher link rates require the clock tree to be programmed with
	 * 'mipi_div' = 1; this has the effect of halving the actual output
	 * pixel rate in the MIPI domain.
	 *
	 * Adjust the pixel rate and link frequency control value to report it
	 * correctly to userspace.
	 */
	if (link_freq > OV5640_LINK_RATE_MAX) {
		pixel_rate /= 2;
		link_freq /= 2;
	}

	for (i = 0; i < ARRAY_SIZE(ov5640_csi2_link_freqs); ++i) {
		if (ov5640_csi2_link_freqs[i] == link_freq)
			break;
	}
	WARN_ON(i == ARRAY_SIZE(ov5640_csi2_link_freqs));

	__v4l2_ctrl_s_ctrl_int64(sensor->ctrls.pixel_rate, pixel_rate);
	__v4l2_ctrl_s_ctrl(sensor->ctrls.link_freq, i);

	hblank = timings->htot - mode->width;
	__v4l2_ctrl_modify_range(sensor->ctrls.hblank,
				 hblank, hblank, 1, hblank);

	vblank = timings->vblank_def;

	if (sensor->current_fr != mode->def_fps) {
		/*
		 * Compute the vertical blanking according to the framerate
		 * configured with s_frame_interval.
		 */
		int fie_num = sensor->frame_interval.numerator;
		int fie_denom = sensor->frame_interval.denominator;

		vblank = ((fie_num * pixel_rate / fie_denom) / timings->htot) -
			mode->height;
	}

	__v4l2_ctrl_vblank_update(sensor, vblank);

	exposure_max = timings->crop.height + vblank - 4;
	exposure_val = clamp_t(s32, sensor->ctrls.exposure->val,
			       sensor->ctrls.exposure->minimum,
			       exposure_max);

	__v4l2_ctrl_modify_range(sensor->ctrls.exposure,
				 sensor->ctrls.exposure->minimum,
				 exposure_max, 1, exposure_val);

	return 0;
}

#endif
static int ov7695_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ov7695 *sensor = to_ov7695(sd);

	// -- mutex_lock(&sensor->lock);
	fi->interval = sensor->frame_interval;
	// -- mutex_unlock(&sensor->lock);
	return 0;
}

static int ov7695_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ov7695 *sensor = to_ov7695(sd);

	const struct ov7695_mode_info *mode;
	int frame_rate, ret = 0;

	if (fi->pad != 0)
		return -EINVAL;

	//mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	mode = sensor->current_mode;

	frame_rate = ov7695_try_frame_interval(sensor, &fi->interval,
					       mode->width,
					       mode->height);
	AD_DEV_DBG(sensor->dev, "%s: after ov7695_try_frame_interval. mode->width: %d; mode->height: %d; frame_rate: %d;\n", __func__, mode->width, mode->height, frame_rate);
	if (frame_rate < 0) {
		/* Always return a valid frame interval value */
		fi->interval = sensor->frame_interval;
		goto out;
	}

	mode = ov7695_find_mode(sensor, mode->width, mode->height, true);



	if (!mode) {
		ret = -EINVAL;
		goto out;
	}

	AD_DEV_DBG(sensor->dev, "%s: after ov7695_find_mode. mode->width: %d; mode->height: %d;\n", __func__, mode->width, mode->height);
	AD_DEV_DBG(sensor->dev, "%s: ov7695_framerates[frame_rate]: %d; ov7695_framerates[mode->max_fps] : %d;\n", __func__, ov7695_framerates[frame_rate], ov7695_framerates[mode->max_fps]);

	if (ov7695_framerates[frame_rate] > ov7695_framerates[mode->max_fps]) {
		ret = -EINVAL;
		goto out;
	}

	if (mode != sensor->current_mode ||
	    frame_rate != sensor->current_fr) {
		sensor->current_fr = frame_rate;
		sensor->frame_interval = fi->interval;
		sensor->current_mode = mode;
		sensor->pending_mode_change = true;

		//--------------ov7695_update_pixel_rate(sensor);
	}
out:
	//mutex_unlock(&sensor->lock);
	return ret;
}

static const struct v4l2_subdev_core_ops ov7695_core_ops = {
	.s_power = ov7695_s_power,
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops ov7695_video_ops = {
	.g_frame_interval = ov7695_g_frame_interval,
	.s_frame_interval = ov7695_s_frame_interval,
	.s_stream = ov7695_s_stream,
};

static const struct v4l2_subdev_pad_ops ov7695_subdev_pad_ops = {
	.init_cfg = ov7695_entity_init_cfg,
	.enum_mbus_code = ov7695_enum_mbus_code,
	.enum_frame_size = ov7695_enum_frame_size,
	.get_fmt = ov7695_get_format,
	.set_fmt = ov7695_set_format,
	.get_selection = ov7695_get_selection,
};

static const struct v4l2_subdev_ops ov7695_subdev_ops = {
	.core = &ov7695_core_ops,
	.video = &ov7695_video_ops,
	.pad = &ov7695_subdev_pad_ops,
};

static int ov7695_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations ov7695_sd_media_ops = {
	.link_setup = ov7695_link_setup,
};

static int ov7695_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *np = client->dev.of_node;
	struct device_node *endpoint;
	struct ov7695 *ov7695;
	u8 chip_id_high, chip_id_low;
	unsigned int i;
	int ret;

	ov7695 = devm_kzalloc(dev, sizeof(struct ov7695), GFP_KERNEL);
	if (!ov7695)
		return -ENOMEM;

	ov7695->i2c_client = client;
	ov7695->dev = dev;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint),
					 &ov7695->ep);

	of_node_put(endpoint);

	if (ret < 0) {
		dev_err(dev, "parsing endpoint node failed\n");
		return ret;
	}

	if (ov7695->ep.bus_type != V4L2_MBUS_CSI2_DPHY) {
		dev_err(dev, "invalid bus type, must be CSI2\n");
		return -EINVAL;
	}

	/* get system clock (xclk) */
	ov7695->xclk = devm_clk_get(dev, "xclk");
	if (!IS_ERR_OR_NULL(ov7695->xclk)) {
		u32 xclk_freq;
		ret = of_property_read_u32(dev->of_node, "clock-frequency", &xclk_freq);
		if (ret) {
			dev_err(dev, "could not get xclk frequency\n");
			return ret;
		}

		/* external clock must be 24MHz, allow 1% tolerance */
		if (xclk_freq < 23760000 || xclk_freq > 24240000) {
			dev_err(dev, "external clock frequency %u is not supported\n",
				xclk_freq);
			return -EINVAL;
		}

		ret = clk_set_rate(ov7695->xclk, xclk_freq);
		if (ret) {
			dev_err(dev, "could not set xclk frequency\n");
			return ret;
		}
	}

	if (of_property_read_bool(np, "external-power-supplies"))
		ov7695->external_power_supplies = 1;
	else
		ov7695->external_power_supplies = 0;

	if (!ov7695->external_power_supplies) {
		for (i = 0; i < OV7695_NUM_SUPPLIES; i++)
			ov7695->supplies[i].supply = ov7695_supply_name[i];

		ret = devm_regulator_bulk_get(dev, OV7695_NUM_SUPPLIES,
						  ov7695->supplies);
		if (ret < 0)
			return ret;
	}

	ov7695->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ov7695->enable_gpio)) {
		dev_err(dev, "cannot get enable gpio\n");
		return PTR_ERR(ov7695->enable_gpio);
	}

	ov7695->rst_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ov7695->rst_gpio)) {
		dev_err(dev, "cannot get reset gpio\n");
		return PTR_ERR(ov7695->rst_gpio);
	}

	ov7695->mclk_enable_gpio = devm_gpiod_get(dev, "mclk-enable", GPIOD_OUT_LOW);

	mutex_init(&ov7695->power_lock);

	v4l2_ctrl_handler_init(&ov7695->ctrls, 9);

	ov7695->pixel_clock = v4l2_ctrl_new_std(&ov7695->ctrls,
						&ov7695_ctrl_ops,
						V4L2_CID_PIXEL_RATE,
						1, INT_MAX, 1, 1);
	ov7695->link_freq = v4l2_ctrl_new_int_menu(&ov7695->ctrls,
						   &ov7695_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(link_freq) - 1,
						   0, link_freq);
	if (ov7695->link_freq)
		ov7695->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ov7695->sd.ctrl_handler = &ov7695->ctrls;

	if (ov7695->ctrls.error) {
		dev_err(dev, "%s: control initialization error %d\n",
		       __func__, ov7695->ctrls.error);
		ret = ov7695->ctrls.error;
		goto free_ctrl;
	}

	v4l2_i2c_subdev_init(&ov7695->sd, client, &ov7695_subdev_ops);
	ov7695->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov7695->pad.flags = MEDIA_PAD_FL_SOURCE;
	ov7695->sd.entity.ops = &ov7695_sd_media_ops;
	ov7695->sd.dev = &client->dev;
	ov7695->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&ov7695->sd.entity, 1, &ov7695->pad);
	if (ret < 0) {
		dev_err(dev, "could not register media entity\n");
		goto free_ctrl;
	}

	ret = ov7695_s_power(&ov7695->sd, true);
	if (ret < 0) {
		dev_err(dev, "could not power up OV7695\n");
		goto free_entity;
	}

	ret = ov7695_read_reg(ov7695, OV7695_CHIP_ID_HIGH, &chip_id_high);
	if (ret < 0 || chip_id_high != OV7695_CHIP_ID_HIGH_BYTE) {
		dev_err(dev, "could not read ID high\n");
		ret = -ENODEV;
		goto power_down;
	}
	ret = ov7695_read_reg(ov7695, OV7695_CHIP_ID_LOW, &chip_id_low);
	if (ret < 0 || chip_id_low != OV7695_CHIP_ID_LOW_BYTE) {
		dev_err(dev, "could not read ID low\n");
		ret = -ENODEV;
		goto power_down;
	}

	dev_info(dev, "OV7695 detected at address 0x%02x\n", client->addr);

	ov7695_s_power(&ov7695->sd, false);

	ret = v4l2_async_register_subdev(&ov7695->sd);
	if (ret < 0) {
		dev_err(dev, "could not register v4l2 device\n");
		goto free_entity;
	}

	ov7695_entity_init_cfg(&ov7695->sd, NULL);

	return 0;

power_down:
	ov7695_s_power(&ov7695->sd, false);
free_entity:
	media_entity_cleanup(&ov7695->sd.entity);
free_ctrl:
	v4l2_ctrl_handler_free(&ov7695->ctrls);
	mutex_destroy(&ov7695->power_lock);

	return ret;
}

static void ov7695_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov7695 *ov7695 = to_ov7695(sd);

	v4l2_async_unregister_subdev(&ov7695->sd);
	media_entity_cleanup(&ov7695->sd.entity);
	v4l2_ctrl_handler_free(&ov7695->ctrls);
	mutex_destroy(&ov7695->power_lock);
}

static const struct i2c_device_id ov7695_id[] = {
	{ "ov7695", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ov7695_id);

static const struct of_device_id ov7695_of_match[] = {
	{ .compatible = "ovti,ov7695" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov7695_of_match);

static struct i2c_driver ov7695_i2c_driver = {
	.driver = {
		.of_match_table = ov7695_of_match,
		.name  = "ov7695",
	},
	.probe_new = ov7695_probe,
	.remove = ov7695_remove,
	.id_table = ov7695_id,
};

module_i2c_driver(ov7695_i2c_driver);

MODULE_DESCRIPTION("Omnivision OV7695 Camera Driver");
MODULE_AUTHOR("Todor Tomov <todor.tomov@linaro.org>");
MODULE_LICENSE("GPL v2");
