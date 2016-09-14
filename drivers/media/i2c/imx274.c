/*
 * imx274.c - imx274 sensor driver
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
 * Copyright (c) 2016, Leopard Imaging, Inc.  All rights reserved.
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

#define DEBUG

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/v4l2-mediabus.h>
#include <media/camera_common.h>
#include <media/imx274.h>

#include "camera/camera_gpio.h"

#include "imx274_mode_tbls.h"

#define IMX274_MAX_COARSE_DIFF (12)

#define IMX274_GAIN_REG_MAX	(1957)

#define IMX274_MIN_GAIN		(0)
#define IMX274_MAX_GAIN		(40)
#define IMX274_MAX_FRAME_LENGTH	(0xffff)
#define IMX274_MIN_EXPOSURE_COARSE	(0x0004)
#define IMX274_MAX_EXPOSURE_COARSE	(IMX274_MAX_FRAME_LENGTH - IMX274_MAX_COARSE_DIFF)
#define IMX274_MIN_FRAME_LENGTH		(4550)

#define IMX274_DEFAULT_GAIN		(0x100)
#define IMX274_DEFAULT_FRAME_LENGTH	(4550)
#define IMX274_DEFAULT_EXPOSURE_COARSE	(IMX274_DEFAULT_FRAME_LENGTH - IMX274_MAX_COARSE_DIFF)

#define IMX274_DEFAULT_MODE	IMX274_MODE_3840X2160
#define IMX274_DEFAULT_WIDTH	3840
#define IMX274_DEFAULT_HEIGHT	2160
#define IMX274_DEFAULT_DATAFMT	MEDIA_BUS_FMT_SRGGB10_1X10
#define IMX274_DEFAULT_CLK_FREQ	24000000

struct imx274 {
	struct camera_common_power_rail	power;
	int				num_ctrls;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	struct media_pad		pad;

	int				reg_offset;
	u32	frame_length;
	s32				group_hold_prev;
	bool				group_hold_en;
	struct regmap			*regmap;
	struct camera_common_data	*s_data;
	struct camera_common_pdata	*pdata;
	struct v4l2_ctrl		*ctrls[];
};

static struct regmap_config imx274_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

static int imx274_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
static int imx274_s_ctrl(struct v4l2_ctrl *ctrl);

static const struct v4l2_ctrl_ops imx274_ctrl_ops = {
	.g_volatile_ctrl = imx274_g_volatile_ctrl,
	.s_ctrl		= imx274_s_ctrl,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
/* Do not change the name field for the controls! */
	{
		.ops = &imx274_ctrl_ops,
		.id = V4L2_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX274_MIN_GAIN * FIXED_POINT_SCALING_FACTOR,
		.max = IMX274_MAX_GAIN * FIXED_POINT_SCALING_FACTOR,
		.def = IMX274_DEFAULT_GAIN * FIXED_POINT_SCALING_FACTOR,
		.step = 1,
	},
	{
		.ops = &imx274_ctrl_ops,
		.id = V4L2_CID_EXPOSURE,
		.name = "Exposure",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 30 * FIXED_POINT_SCALING_FACTOR,
		.max = (s64)33000LL * (s64)FIXED_POINT_SCALING_FACTOR,
		.def = 16 * FIXED_POINT_SCALING_FACTOR,
		.step = 1,
	},
	{
		.ops = &imx274_ctrl_ops,
		.id = V4L2_CID_FRAME_RATE,
		.name = "Frame Rate",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 30 * FIXED_POINT_SCALING_FACTOR,
		.max = 60 * FIXED_POINT_SCALING_FACTOR,
		.def = 60 * FIXED_POINT_SCALING_FACTOR,
		.step = 1,
	},
	{
		.ops = &imx274_ctrl_ops,
		.id = V4L2_CID_FUSE_ID,
		.name = "Fuse ID",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = IMX274_FUSE_ID_STR_SIZE,
		.step = 2,
	},
	{
		.ops = &imx274_ctrl_ops,
		.id = V4L2_CID_SENSOR_MODE_ID,
		.name = "Sensor Mode",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = (s64)0xFFFFFFFFFFFFFFFF,
		.def = 0,
		.step = 1,
	},
	{
		.ops = &imx274_ctrl_ops,
		.id = V4L2_CID_GROUP_HOLD,
		.name = "Group Hold",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
	{
		.ops = &imx274_ctrl_ops,
		.id = V4L2_CID_HDR_EN,
		.name = "HDR enable",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
};

static inline void imx274_get_frame_length_regs(imx274_reg *regs,
				u32 frame_length)
{
	regs->addr = IMX274_FRAME_LENGTH_ADDR_1;
	regs->val = (frame_length >> 16) & 0x01;
	(regs + 1)->addr = IMX274_FRAME_LENGTH_ADDR_2;
	(regs + 1)->val = (frame_length >> 8) & 0xff;
	(regs + 2)->addr = IMX274_FRAME_LENGTH_ADDR_3;
	(regs + 2)->val = (frame_length) & 0xff;
}

static inline void imx274_get_coarse_time_regs(imx274_reg *regs,
				u32 coarse_time)
{
	regs->addr = IMX274_COARSE_TIME_ADDR_MSB;
	regs->val = (coarse_time >> 8) & 0x00ff;
	(regs + 1)->addr = IMX274_COARSE_TIME_ADDR_LSB;
	(regs + 1)->val = (coarse_time ) & 0x00ff;
}

static inline void imx274_get_gain_regs(imx274_reg *regs,
				u16 gain)
{
	regs->addr = IMX274_ANALOG_GAIN_ADDR_MSB;
	regs->val = (gain >> 8) & 0x07;

	(regs + 1)->addr = IMX274_ANALOG_GAIN_ADDR_LSB;
	(regs + 1)->val = (gain) & 0xff;
}

static int test_mode;
module_param(test_mode, int, 0644);

static int imx274_set_gain(struct imx274 *priv, s64 val);
static int imx274_set_frame_length(struct imx274 *priv, u32 val);
static int imx274_set_frame_rate(struct imx274 *priv, s64 val);
static int imx274_set_exposure(struct imx274 *priv, s64 val);

static inline int imx274_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	struct imx274 *priv = (struct imx274 *)s_data->priv;
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(priv->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

static int imx274_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
	int err;
	struct imx274 *priv = (struct imx274 *)s_data->priv;

	err = regmap_write(priv->regmap, addr, val);
	if (err)
		pr_err("%s:i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}

static int imx274_write_table(struct imx274 *priv,
			      const imx274_reg table[])
{
	return regmap_util_write_table_8(priv->regmap,
					 table,
					 NULL, 0,
					 IMX274_TABLE_WAIT_MS,
					 IMX274_TABLE_END);
}

static int imx274_clamp_coarse_time(struct imx274 *priv, s32 *val)
{
	if (*val > (priv->frame_length - IMX274_MAX_COARSE_DIFF)) {
		dev_dbg(&priv->i2c_client->dev,
			 "%s: %d to %d\n", __func__, *val,
			 priv->frame_length - IMX274_MAX_COARSE_DIFF);
		*val = priv->frame_length - IMX274_MAX_COARSE_DIFF;
	} else if ( *val < IMX274_MIN_EXPOSURE_COARSE)
		*val = IMX274_MIN_EXPOSURE_COARSE;

	*val = priv->frame_length - *val;

	return 0;
}

static void imx274_gpio_set(struct imx274 *priv,
			    unsigned int gpio, int val)
{
	if (priv->pdata->use_cam_gpio)
		cam_gpio_ctrl(priv->i2c_client, gpio, val, 1);
	else {
		if (gpio_cansleep(gpio))
			gpio_set_value_cansleep(gpio, val);
		else
			gpio_set_value(gpio, val);
	}
}
static int imx274_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct imx274 *priv = (struct imx274 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power on\n", __func__);

	if (priv->pdata && priv->pdata->power_on) {
		err = priv->pdata->power_on(pw);
		if (err)
			pr_err("%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	/* sleeps calls in the sequence below are for internal device
	 * signal propagation as specified by sensor vendor */

	if (pw->avdd)
		err = regulator_enable(pw->avdd);
	if (err)
		goto imx274_avdd_fail;

	if (pw->iovdd)
		err = regulator_enable(pw->iovdd);
	if (err)
		goto imx274_iovdd_fail;

	usleep_range(1, 2);
	if (pw->pwdn_gpio)
		imx274_gpio_set(priv, pw->pwdn_gpio, 1);

	/* datasheet 2.9: reset requires ~2ms settling time
	 * a power on reset is generated after core power becomes stable */
	usleep_range(2000, 2010);

	if (pw->reset_gpio)
		imx274_gpio_set(priv, pw->reset_gpio, 1);

	/* datasheet fig 2-9: t3 */
	usleep_range(1350, 1360);

	pw->state = SWITCH_ON;
	return 0;

imx274_iovdd_fail:
	if (pw->avdd)
		regulator_disable(pw->avdd);

imx274_avdd_fail:
	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

static int imx274_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct imx274 *priv = (struct imx274 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power off\n", __func__);

	if (priv->pdata && priv->pdata->power_on) {
		err = priv->pdata->power_off(pw);
		if (!err)
			pw->state = SWITCH_OFF;
		else
			pr_err("%s failed.\n", __func__);
		return err;
	}

	/* sleeps calls in the sequence below are for internal device
	 * signal propagation as specified by sensor vendor */

	usleep_range(21, 25);
	if (pw->pwdn_gpio)
		imx274_gpio_set(priv, pw->pwdn_gpio, 0);
	usleep_range(1, 2);
	if (pw->reset_gpio)
		imx274_gpio_set(priv, pw->reset_gpio, 0);

	/* datasheet 2.9: reset requires ~2ms settling time*/
	usleep_range(2000, 2010);

	if (pw->iovdd)
		regulator_disable(pw->iovdd);
	if (pw->avdd)
		regulator_disable(pw->avdd);

	return 0;
}

static int imx274_power_put(struct imx274 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s\n", __func__);

	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->avdd))
		regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		regulator_put(pw->iovdd);

	pw->avdd = NULL;
	pw->iovdd = NULL;

	if (priv->pdata->use_cam_gpio && pw->pwdn_gpio)
		cam_gpio_deregister(priv->i2c_client, pw->pwdn_gpio);
	else {
		if (pw->pwdn_gpio)
			gpio_free(pw->pwdn_gpio);
		if (pw->reset_gpio)
			gpio_free(pw->reset_gpio);
	}

	return 0;
}

static int imx274_power_get(struct imx274 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	struct camera_common_pdata *pdata = priv->pdata;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0;

	dev_dbg(&priv->i2c_client->dev, "%s\n", __func__);

	mclk_name = priv->pdata->mclk_name ?
		    priv->pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(&priv->i2c_client->dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(&priv->i2c_client->dev,
			"unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parentclk_name = priv->pdata->parentclk_name;
	if (parentclk_name) {
		parent = devm_clk_get(&priv->i2c_client->dev, parentclk_name);
		if (IS_ERR(parent))
			dev_err(&priv->i2c_client->dev,
				"unable to get parent clcok %s",
				parentclk_name);
		else
			clk_set_parent(pw->mclk, parent);
	}

	/* analog 2.8v */
	if (pdata->regulators.avdd) {
		err = camera_common_regulator_get(priv->i2c_client,
						  &pw->avdd, pdata->regulators.avdd);
		if (err) {
			dev_err(&priv->i2c_client->dev,
				"%s: err %d getting avdd\n",
				__func__, err);
			return err;
		}
	} else
		pw->avdd = NULL;
	/* IO 1.8v */
	if (pdata->regulators.iovdd) {
		err = camera_common_regulator_get(priv->i2c_client,
						  &pw->iovdd, pdata->regulators.iovdd);
		if (err) {
			dev_err(&priv->i2c_client->dev,
				"%s: err %d getting iovdd\n",
				__func__, err);
			if (pw->avdd)
				regulator_put(pw->avdd);
			return err;
		}
	} else
		pw->iovdd = NULL;

	pw->reset_gpio = pdata->reset_gpio;
	pw->pwdn_gpio = pdata->pwdn_gpio;

	if (priv->pdata->use_cam_gpio && pw->pwdn_gpio) {
		err = cam_gpio_register(priv->i2c_client, pw->pwdn_gpio);
		if (err)
			dev_err(&priv->i2c_client->dev,
				"%s ERR can't register cam gpio %u!\n",
				 __func__, pw->pwdn_gpio);
	} else {
		if (pw->pwdn_gpio)
			gpio_request(pw->pwdn_gpio, "cam_pwdn_gpio");
		if (pw->reset_gpio)
			gpio_request(pw->reset_gpio, "cam_reset_gpio");
	}

	pw->state = SWITCH_OFF;
	return err;
}


static int imx274_start_stream(struct imx274* priv, int mode)
{
	int err=0;

	err = imx274_write_table(priv, mode_table[IMX274_MODE_START_STREAM_1]);
	if (err)
		return err;

	err = imx274_write_table(priv, mode_table[IMX274_MODE_START_STREAM_2]);
	if (err)
		return err;

	err = imx274_write_table(priv, mode_table[mode]);
	if (err)
		return err;

	msleep(20);
	err = imx274_write_table(priv, mode_table[IMX274_MODE_START_STREAM_3]);
	if (err)
		return err;

	msleep(20);
	err = imx274_write_table(priv, mode_table[IMX274_MODE_START_STREAM_4]);
	if (err)
		return err;

	return 0;
}

static int imx274_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct imx274 *priv = (struct imx274 *)s_data->priv;
	//struct v4l2_control control;
	int err;
	struct v4l2_ext_controls ctrls;
	struct v4l2_ext_control control[3];

	dev_dbg(&client->dev, "%s++ enable %d\n", __func__, enable);

	if (!enable)
		return imx274_write_table(priv,
			mode_table[IMX274_MODE_STOP_STREAM]);

	err = imx274_start_stream(priv, s_data->mode);
	if (err)
		goto exit;

	/* write list of override regs for the asking gain, */
	/* frame rate and exposure time    */
	memset(&ctrls, 0, sizeof(ctrls));
	ctrls.ctrl_class = V4L2_CTRL_ID2CLASS(V4L2_CID_GAIN);
	ctrls.count = 3;
	ctrls.controls = control;

	control[0].id = V4L2_CID_GAIN;
	control[1].id = V4L2_CID_FRAME_RATE;
	control[2].id = V4L2_CID_EXPOSURE;
	err = v4l2_g_ext_ctrls(&priv->ctrl_handler, &ctrls);
	if (err == 0) {
		err |= imx274_set_gain(priv, control[0].value64);
		if (err)
			dev_err(&client->dev,
				"%s: error gain override\n", __func__);

		err |= imx274_set_frame_rate(priv, control[1].value64);
		if (err)
			dev_err(&client->dev,
				"%s: error frame length override\n", __func__);

		err |= imx274_set_exposure(priv, control[2].value64);
		if (err)
			dev_err(&client->dev,
				"%s: error exposure override\n", __func__);
	} else {
		dev_err(&client->dev, "%s: faile to get overrides\n", __func__);
	}

	return 0;
exit:
	dev_dbg(&client->dev, "%s: error setting stream\n", __func__);
	return err;
}

static int imx274_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct imx274 *priv = (struct imx274 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	*status = pw->state == SWITCH_ON;
	return 0;
}

static struct v4l2_subdev_video_ops imx274_subdev_video_ops = {
	.s_stream	= imx274_s_stream,
	.g_mbus_config	= camera_common_g_mbus_config,
	.g_input_status	= imx274_g_input_status,
};

static struct v4l2_subdev_core_ops imx274_subdev_core_ops = {
	.s_power	= camera_common_s_power,
};

static int imx274_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static int imx274_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_format *format)
{
	int ret;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		ret = camera_common_try_fmt(sd, &format->format);
	else
		ret = camera_common_s_fmt(sd, &format->format);

	return ret;
}

static struct v4l2_subdev_pad_ops imx274_subdev_pad_ops = {
	.set_fmt	= imx274_set_fmt,
	.get_fmt	= imx274_get_fmt,
	.enum_mbus_code = camera_common_enum_mbus_code,
	.enum_frame_size        = camera_common_enum_framesizes,
	.enum_frame_interval    = camera_common_enum_frameintervals,
};


static struct v4l2_subdev_ops imx274_subdev_ops = {
	.core	= &imx274_subdev_core_ops,
	.video	= &imx274_subdev_video_ops,
	.pad	= &imx274_subdev_pad_ops,
};

static struct of_device_id imx274_of_match[] = {
	{ .compatible = "nvidia,imx274", },
	{ },
};

static struct camera_common_sensor_ops imx274_common_ops = {
	.power_on = imx274_power_on,
	.power_off = imx274_power_off,
	.write_reg = imx274_write_reg,
	.read_reg = imx274_read_reg,
};

static int imx274_set_group_hold(struct imx274 *priv)
{
	int err;
	int gh_prev = switch_ctrl_qmenu[priv->group_hold_prev];

	//dev_dbg(&priv->i2c_client->dev, "%s\n", __func__);

	if (priv->group_hold_en == true && gh_prev == SWITCH_OFF) {
		/* enter group hold */
		err = imx274_write_reg(priv->s_data,
				       IMX274_GROUP_HOLD_ADDR, 0x01);
		if (err)
			goto fail;

		priv->group_hold_prev = 1;

		//dev_dbg(&priv->i2c_client->dev,
		//	 "%s: enter group hold\n", __func__);
	} else if (priv->group_hold_en == false && gh_prev == SWITCH_ON) {
		/* leave group hold */
		err = imx274_write_reg(priv->s_data,
				       IMX274_GROUP_HOLD_ADDR, 0x00);
		if (err)
			goto fail;

		priv->group_hold_prev = 0;

		//dev_dbg(&priv->i2c_client->dev,
		//	 "%s: leave group hold\n", __func__);
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: Group hold control error\n", __func__);
	return err;
}

#define IMX274_GAIN_SHIFT 8
static int log_fun_table[23] = {
       0,
       222,
       421,
       598,
       755,
       896,
       1021,
       1133,
       1232,
       1321,
       1400,
       1470,
       1533,
       1589,
       1639,
      1683,
      1723,
      1758,
      1790,
      1818,
      1843,
      1865,
      1885
};

static int imx274_set_dgain(struct imx274 *priv, u16 dgain)
{
	imx274_write_reg(priv->s_data, 0x3012, dgain);
	return 0;
}

static int imx274_set_gain(struct imx274 *priv, s64 val)
{
	imx274_reg reg_list[2];
	int err;
	u16 gain;
	int i;

	dev_dbg(&priv->i2c_client->dev, "%s - val = %lld\n", __func__, val);

	if (!priv->group_hold_prev)
		imx274_set_group_hold(priv);


	val = val / FIXED_POINT_SCALING_FACTOR;
	dev_dbg(&priv->i2c_client->dev, "input gain value: %lld\n", val);

	if (val < IMX274_MIN_GAIN)
		val = IMX274_MIN_GAIN;
	else if (val > IMX274_MAX_GAIN)
		val = IMX274_MAX_GAIN;
	if (val > 18) {
		imx274_set_dgain(priv, 3);
		val -= 18;
	} else if (val > 12) {
		imx274_set_dgain(priv, 2);
		val -= 12;
	} else if (val > 6) {
		imx274_set_dgain(priv, 1);
		val -= 6;
	} else {
		imx274_set_dgain(priv, 0);
	}

	gain = log_fun_table[val];
	imx274_get_gain_regs(reg_list, gain);
	dev_dbg(&priv->i2c_client->dev,
		 "%s: gain: %04x\n", __func__, gain);

	for (i = 0; i < 2; i++) {
		err = imx274_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: GAIN control error\n", __func__);
	return err;
}

static int imx274_set_frame_length(struct imx274 *priv, u32 val)
{
	imx274_reg reg_list[3];
	int err;
	u32 frame_length;
	int i;

	dev_dbg(&priv->i2c_client->dev, "%s length = %d\n", __func__, val);

	if (!priv->group_hold_prev)
		imx274_set_group_hold(priv);

	frame_length = (u32)val;
	priv->frame_length = frame_length;

	imx274_get_frame_length_regs(reg_list, frame_length);
	dev_dbg(&priv->i2c_client->dev,
		 "%s: val: %d\n", __func__, frame_length);

	for (i = 0; i < 3; i++) {
		err = imx274_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: FRAME_LENGTH control error\n", __func__);
	return err;
}

static int imx274_set_frame_rate(struct imx274 *priv, s64 val)
{
	struct camera_common_data *s_data = priv->s_data;
	struct camera_common_mode_info *mode = priv->pdata->mode_info;
	int err;
	s64 frame_length;

	dev_dbg(&priv->i2c_client->dev,
		 "%s: val: %lld\n", __func__, val);

	frame_length = mode[s_data->mode].pixel_clock *
		FIXED_POINT_SCALING_FACTOR /
		mode[s_data->mode].line_length / val;
	err = imx274_set_frame_length(priv, frame_length);
	if (err)
		goto fail;

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: FRAME_LENGTH control error\n", __func__);
	return err;
}

static int imx274_set_coarse_time(struct imx274 *priv, s32 val)
{
	imx274_reg reg_list[2];
	int err;
	u32 coarse_time;
	int i;

	dev_dbg(&priv->i2c_client->dev, "%s\n", __func__);

	if (priv->frame_length == 0)
		priv->frame_length = IMX274_MIN_FRAME_LENGTH;

	if (!priv->group_hold_prev)
		imx274_set_group_hold(priv);

	coarse_time = (u32)val;
	dev_dbg(&priv->i2c_client->dev,
		 "%s: input val: %d\n", __func__, coarse_time);

	 imx274_clamp_coarse_time(priv, &coarse_time);

	imx274_get_coarse_time_regs(reg_list, coarse_time);
	dev_dbg(&priv->i2c_client->dev,
		 "%s: set val: %d\n", __func__, coarse_time);

	for (i = 0; i < 2; i++) {
		err = imx274_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}
	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: COARSE_TIME control error\n", __func__);
	return err;
}


static int imx274_set_exposure(struct imx274 *priv, s64 val)
{
	int err;
	s64 coarse_time;

	//coarse_time = mode[s_data->mode].pixel_clock * val /
	//	priv->frame_length / FIXED_POINT_SCALING_FACTOR;
	coarse_time = val * 9230 / 139810;

	dev_dbg(&priv->i2c_client->dev,
		 "%s: val: %lld, frame_length = %d, coarse_time = %lld\n", __func__,
		val, priv->frame_length, coarse_time);

	err = imx274_set_coarse_time(priv, (s32) coarse_time);
	if (err)
		dev_dbg(&priv->i2c_client->dev,
		"%s: error coarse time SHS1 override\n", __func__);

	return err;
}

static int imx274_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx274 *priv =
		container_of(ctrl->handler, struct imx274, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {

	default:
			pr_err("%s: unknown ctrl id.\n", __func__);
			return -EINVAL;
	}

	return err;
}

static int imx274_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx274 *priv =
		container_of(ctrl->handler, struct imx274, ctrl_handler);
	//s32 clamp_val = ctrl->val;
	int err = 0;
	struct camera_common_data	*s_data = priv->s_data;

	//dev_dbg(&priv->i2c_client->dev, "%s, ctrl->id = 0x%x, ctrl->val = %d\n", __func__, ctrl->id, ctrl->val);

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		err = imx274_set_gain(priv, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		err = imx274_set_exposure(priv, *ctrl->p_new.p_s64);
		break;
	case V4L2_CID_GROUP_HOLD:
		if (switch_ctrl_qmenu[ctrl->val] == SWITCH_ON) {
			priv->group_hold_en = true;
		} else {
			priv->group_hold_en = false;
			err = imx274_set_group_hold(priv);
		}
		break;
	case V4L2_CID_FRAME_RATE:
		err = imx274_set_frame_rate(priv, ctrl->val);
		break;
	case V4L2_CID_SENSOR_MODE_ID:
		s_data->sensor_mode_id = (int) (*ctrl->p_new.p_s64);
		break;
	case V4L2_CID_HDR_EN:
		break;
	default:
		pr_err("%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	//if (clamp_val != ctrl->val)  {
	//	ctrl->val = clamp_val;
	//	return -EINVAL;
	//}

	return err;
}

static int imx274_ctrls_init(struct imx274 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct v4l2_ctrl *ctrl;
	int num_ctrls;
	int err;
	int i;

	dev_dbg(&client->dev, "%s++\n", __func__);

	num_ctrls = ARRAY_SIZE(ctrl_config_list);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, num_ctrls);

	for (i = 0; i < num_ctrls; i++) {
		ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,
			&ctrl_config_list[i], NULL);
		if (ctrl == NULL) {
			dev_err(&client->dev, "Failed to init %s ctrl, err=%d\n",
				ctrl_config_list[i].name, priv->ctrl_handler.error);
			continue;
		}

		if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
			ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->p_new.p_char = devm_kzalloc(&client->dev,
				ctrl_config_list[i].max + 1, GFP_KERNEL);
			if (!ctrl->p_new.p_char) {
				dev_err(&client->dev,
					"Failed to allocate data\n");
				return -ENOMEM;
			}
		}
		priv->ctrls[i] = ctrl;
	}

	priv->num_ctrls = num_ctrls;
	priv->subdev->ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
			priv->ctrl_handler.error);
		err = priv->ctrl_handler.error;
		goto error;
	}

	err = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (err) {
		dev_err(&client->dev,
			"Error %d setting default controls\n", err);
		goto error;
	}

	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

MODULE_DEVICE_TABLE(of, imx274_of_match);

static struct camera_common_pdata *imx274_parse_dt(struct imx274 *priv,
				struct i2c_client *client,
				struct camera_common_data *s_data)
{
	struct device_node *node = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int gpio;
	int err;
	const char *str;

	if (!node)
		return NULL;

	match = of_match_device(imx274_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, " Failed to find matching dt id\n");
		return NULL;
	}

	of_property_read_string(node, "use_sensor_mode_id", &str);
	if (!strcmp(str, "true"))
		s_data->use_sensor_mode_id = true;
	else
		s_data->use_sensor_mode_id = false;

	board_priv_pdata = devm_kzalloc(&client->dev,
			   sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata) {
		dev_err(&client->dev, "Failed to allocate pdata\n");
		return NULL;
	}

	err = camera_common_parse_clocks(client, board_priv_pdata);
	if (err) {
		dev_err(&client->dev, "Failed to find clocks\n");
		goto error;
	}

	gpio = of_get_named_gpio(node, "pwdn-gpios", 0);
	if (gpio < 0) {
		dev_dbg(&client->dev, "pwdn gpios not in DT\n");
		gpio = 0;
	}
	board_priv_pdata->pwdn_gpio = (unsigned int)gpio;

	gpio = of_get_named_gpio(node, "reset-gpios", 0);
	if (gpio < 0) {
		/* reset-gpio is not absoluctly needed */
		dev_dbg(&client->dev, "reset gpios not in DT\n");
		gpio = 0;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	board_priv_pdata->use_cam_gpio =
		of_property_read_bool(node, "cam,use-cam-gpio");

	err = of_property_read_string(node, "avdd-reg",
			&board_priv_pdata->regulators.avdd);
	if (err) {
		dev_dbg(&client->dev, "avdd-reg not in DT\n");
		board_priv_pdata->regulators.avdd = NULL;
	}
	err = of_property_read_string(node, "iovdd-reg",
			&board_priv_pdata->regulators.iovdd);
	if (err) {
		dev_dbg(&client->dev, "iovdd-reg not in DT\n");
		board_priv_pdata->regulators.iovdd = NULL;
	}

	err = camera_common_parse_sensor_mode(client, board_priv_pdata);
	if (err)
		dev_err(&client->dev, "Failed to load mode info %d\n", err);

	return board_priv_pdata;

error:
	devm_kfree(&client->dev, board_priv_pdata);
	return NULL;
}


static int imx274_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx274_subdev_internal_ops = {
	.open = imx274_open,
};

static const struct media_entity_operations imx274_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int imx274_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct device_node *node = client->dev.of_node;
	struct imx274 *priv;
	char debugfs_name[10];
	int err;

	dev_dbg(&client->dev, "%s \n", __func__);

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	common_data = devm_kzalloc(&client->dev,
			    sizeof(struct camera_common_data), GFP_KERNEL);
	if (!common_data)
		return -ENOMEM;

	priv = devm_kzalloc(&client->dev,
			    sizeof(struct imx274) + sizeof(struct v4l2_ctrl *) *
			    ARRAY_SIZE(ctrl_config_list),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regmap = devm_regmap_init_i2c(client, &imx274_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	priv->pdata = imx274_parse_dt(priv, client, common_data);
	if (!priv->pdata) {
		dev_err(&client->dev, "unable to get platform data\n");
		return -EFAULT;
	}

	common_data->ops		= &imx274_common_ops;
	common_data->ctrl_handler	= &priv->ctrl_handler;
	common_data->i2c_client		= client;
	common_data->frmfmt		= &imx274_frmfmt[0];
	common_data->colorfmt		= camera_common_find_datafmt(
					  IMX274_DEFAULT_DATAFMT);
	common_data->power		= &priv->power;
	common_data->ctrls		= priv->ctrls;
	common_data->priv		= (void *)priv;
	common_data->numctrls		= ARRAY_SIZE(ctrl_config_list);
	common_data->numfmts		= ARRAY_SIZE(imx274_frmfmt);
	common_data->def_mode		= IMX274_DEFAULT_MODE;
	common_data->def_width		= IMX274_DEFAULT_WIDTH;
	common_data->def_height		= IMX274_DEFAULT_HEIGHT;
	common_data->fmt_width		= common_data->def_width;
	common_data->fmt_height		= common_data->def_height;
	common_data->def_clk_freq	= IMX274_DEFAULT_CLK_FREQ;

	priv->i2c_client = client;
	priv->s_data			= common_data;
	priv->subdev			= &common_data->subdev;
	priv->subdev->dev		= &client->dev;
	priv->s_data->dev		= &client->dev;

	err = imx274_power_get(priv);
	if (err)
		return err;

	err = camera_common_parse_ports(client, common_data);
	if (err) {
		dev_err(&client->dev, "Failed to find port info\n");
		return err;
	}
	sprintf(debugfs_name, "imx274_%c", common_data->csi_port + 'a');
	dev_dbg(&client->dev, "%s: name %s\n", __func__, debugfs_name);
	camera_common_create_debugfs(common_data, debugfs_name);

	v4l2_i2c_subdev_init(priv->subdev, client, &imx274_subdev_ops);

	err = imx274_ctrls_init(priv);
	if (err)
		return err;

	priv->subdev->internal_ops = &imx274_subdev_internal_ops;
	priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			       V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	priv->subdev->entity.ops = &imx274_media_ops;
	err = media_entity_init(&priv->subdev->entity, 1, &priv->pad, 0);
	if (err < 0) {
		dev_err(&client->dev, "unable to init media entity\n");
		return err;
	}
#endif

	err = v4l2_async_register_subdev(priv->subdev);
	if (err)
		return err;

	dev_dbg(&client->dev, "Detected IMX274 sensor\n");
	return 0;
}

static int
imx274_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct imx274 *priv = (struct imx274 *)s_data->priv;

	v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif

	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	imx274_power_put(priv);
	camera_common_remove_debugfs(s_data);

	return 0;
}

static const struct i2c_device_id imx274_id[] = {
	{ "imx274", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx274_id);

static struct i2c_driver imx274_i2c_driver = {
	.driver = {
		.name = "imx274",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(imx274_of_match),
	},
	.probe = imx274_probe,
	.remove = imx274_remove,
	.id_table = imx274_id,
};

module_i2c_driver(imx274_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX274");
MODULE_LICENSE("GPL v2");
