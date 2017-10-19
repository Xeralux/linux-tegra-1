/*
 * imx274.c - imx274 sensor driver
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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

#define DEBUG 1
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/camera_common.h>
#include <media/imx274.h>

#include "camera/camera_gpio.h"
#include "imx274_mode_tbls.h"

#define IMX274_MAX_COARSE_DIFF		10

#define IMX274_GAIN_SHIFT		8
#define IMX274_GAIN_REG_MAX			(1957)
#define IMX274_GAIN_SHIFT_MASK			((1 << IMX274_GAIN_SHIFT) - 1)
#define IMX274_MAX_DIGITAL_GAIN			(8)
#define IMX274_MAX_ANALOG_GAIN			(2048 / (2048 - IMX274_GAIN_REG_MAX))
#define IMX274_GAIN_CONST			(2048) /* for gain formula */



#define IMX274_MIN_GAIN		(1)
#define IMX274_MAX_GAIN		(180)
#define IMX274_MIN_FRAME_LENGTH	(0x8ED)
#define IMX274_MAX_FRAME_LENGTH	(0xB292)
#define IMX274_MIN_EXPOSURE_COARSE	(0x0001)
#define IMX274_MAX_EXPOSURE_COARSE	\
	(IMX274_MAX_FRAME_LENGTH-IMX274_MAX_COARSE_DIFF)

#define IMX274_DEFAULT_GAIN		IMX274_MIN_GAIN
#define IMX274_DEFAULT_FRAME_LENGTH	(0x111B)
#define IMX274_DEFAULT_EXPOSURE_COARSE	\
	(IMX274_DEFAULT_FRAME_LENGTH-IMX274_MAX_COARSE_DIFF)

#define IMX274_DEFAULT_MODE	IMX274_MODE_3840X2160

#define IMX274_DEFAULT_WIDTH	3864
#define IMX274_DEFAULT_HEIGHT	2174
#define IMX274_DEFAULT_DATAFMT	MEDIA_BUS_FMT_SRGGB10_1X10
#define IMX274_DEFAULT_CLK_FREQ	24000000
/*
 * shift and mask constants
 */
#define IMX274_SHIFT_8_BITS			(8)
#define IMX274_SHIFT_16_BITS			(16)
#define IMX274_MASK_LSB_2_BITS			(0x03)
#define IMX274_MASK_LSB_3_BITS			(0x07)
#define IMX274_MASK_LSB_4_BITS			(0x0f)
#define IMX274_MASK_LSB_8_BITS			(0x00ff)


#define IMX274_FRAME_LENGTH_ADDR_1		0x30FA /* VMAX, MSB */
#define IMX274_FRAME_LENGTH_ADDR_2		0x30F9 /* VMAX */
#define IMX274_FRAME_LENGTH_ADDR_3		0x30F8 /* VMAX, LSB */
#define IMX274_SVR_REG_MSB			0x300F /* SVR */
#define IMX274_SVR_REG_LSB			0x300E /* SVR */
#define IMX274_HMAX_REG_MSB			0x30F7 /* HMAX */
#define IMX274_HMAX_REG_LSB			0x30F6 /* HMAX */
#define IMX274_COARSE_TIME_ADDR_MSB		0x300D /* SHR */
#define IMX274_COARSE_TIME_ADDR_LSB		0x300C /* SHR */
#define IMX274_ANALOG_GAIN_ADDR_LSB		0x300A /* ANALOG GAIN LSB */
#define IMX274_ANALOG_GAIN_ADDR_MSB		0x300B /* ANALOG GAIN MSB */
#define IMX274_DIGITAL_GAIN_REG			0x3012 /* Digital Gain */
#define IMX274_VFLIP_REG			0x301A /* VERTICAL FLIP */
#define IMX274_STANDBY_REG			0x3000 /* STANDBY */


struct imx274 {
	struct camera_common_power_rail	power;
	int				num_ctrls;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	struct media_pad		pad;
	u32	frame_length;

	s32				group_hold_prev;
	bool				group_hold_en;
	struct regmap			*regmap;
	struct camera_common_data	*s_data;
	struct camera_common_pdata	*pdata;
	struct v4l2_ctrl		*ctrls[];
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

static int imx274_s_ctrl(struct v4l2_ctrl *ctrl);

static const struct v4l2_ctrl_ops imx274_ctrl_ops = {
	.s_ctrl		= imx274_s_ctrl,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
/* Do not change the name field for the controls! */
	{
		.ops = &imx274_ctrl_ops,
		.id = V4L2_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX274_MIN_GAIN * 100,
		.max = IMX274_MAX_GAIN * 100,
		.def = IMX274_DEFAULT_GAIN * 100,
		.step = 1,
	},
	{
		.ops = &imx274_ctrl_ops,
		.id = V4L2_CID_FRAME_LENGTH,
		.name = "Frame Length",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX274_MIN_FRAME_LENGTH,
		.max = IMX274_MAX_FRAME_LENGTH,
		.def = IMX274_DEFAULT_FRAME_LENGTH,
		.step = 1,
	},
	{
		.ops = &imx274_ctrl_ops,
		.id = V4L2_CID_COARSE_TIME,
		.name = "Coarse Time",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX274_MIN_EXPOSURE_COARSE,
		.max = IMX274_MAX_EXPOSURE_COARSE,
		.def = IMX274_DEFAULT_EXPOSURE_COARSE,
		.step = 1,
	},
	{
		.ops = &imx274_ctrl_ops,
		.id = V4L2_CID_COARSE_TIME_SHORT,
		.name = "Coarse Time Short",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX274_MIN_EXPOSURE_COARSE,
		.max = IMX274_MAX_EXPOSURE_COARSE,
		.def = IMX274_DEFAULT_EXPOSURE_COARSE,
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

static inline void imx274_get_vmax_regs(imx274_reg *regs,
				u32 vmax)
{
	regs->addr = IMX274_VMAX_ADDR_MSB;
	regs->val = (vmax >> 8) & 0xff;
	(regs + 1)->addr = IMX274_VMAX_ADDR_LSB;
	(regs + 1)->val = (vmax) & 0xff;
}

static inline void imx274_get_shr_regs(imx274_reg *regs,
				u32 shr)
{
	regs->addr = IMX274_SHR_ADDR_MSB;
	regs->val = (shr >> 8) & 0xff;
	(regs + 1)->addr = IMX274_SHR_ADDR_LSB;
	(regs + 1)->val = (shr) & 0xff;
}

static inline void imx274_get_gain_reg(imx274_reg *regs,
				u16 gain)
{
	regs->addr = IMX274_GAIN_ADDR_MSB;
	regs->val = (gain >> 8) & 0xff;
	(regs + 1)->addr = IMX274_GAIN_ADDR_LSB;
	(regs + 1)->val = (gain) & 0xff;
}

static int test_mode;
module_param(test_mode, int, 0644);

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

	if (likely(pw->dvdd))
		regulator_put(pw->dvdd);

	pw->avdd = NULL;
	pw->iovdd = NULL;
	pw->dvdd = NULL;

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
		if (IS_ERR(parent)) {
			dev_err(&priv->i2c_client->dev,
				"unable to get parent clcok %s",
				parentclk_name);
		} else
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

static int imx274_set_gain(struct imx274 *priv, s32 val);
static int imx274_set_frame_length(struct imx274 *priv, s32 val);
static int imx274_set_coarse_time(struct imx274 *priv, s32 val);

static int imx274_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct imx274 *priv = (struct imx274 *)s_data->priv;
	struct v4l2_control control;
	int err;

	dev_dbg(&client->dev, "%s++\n", __func__);

	imx274_write_table(priv, mode_table[IMX274_MODE_STOP_STREAM]);

	if (!enable)
		return 0;

	dev_dbg(&client->dev, "%s mode[%d]\n", __func__, s_data->mode);

	err = imx274_write_table(priv, mode_table[s_data->mode]);
	if (err)
		goto exit;


	//if (s_data->override_enable) {
		/* write list of override regs for the asking frame length, */
		/* coarse integration time, and gain.                       */
		control.id = V4L2_CID_GAIN;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= imx274_set_gain(priv, control.value);
		if (err)
			dev_dbg(&client->dev,
				"%s: error gain override\n", __func__);

		control.id = V4L2_CID_FRAME_LENGTH;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= imx274_set_frame_length(priv, control.value);
		if (err)
			dev_dbg(&client->dev,
				"%s: error frame length override\n", __func__);

		control.id = V4L2_CID_COARSE_TIME;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= imx274_set_coarse_time(priv, control.value);
		if (err)
			dev_dbg(&client->dev,
				"%s: error coarse time override\n", __func__);
	//}

	if (test_mode) {
		err = imx274_write_table(priv,
			mode_table[IMX274_MODE_TEST_PATTERN]);
		if (err)
			goto exit;
		}

	err = imx274_write_table(priv, mode_table[IMX274_MODE_START_STREAM]);
	if (err)
		goto exit;


	return 0;
exit:
	dev_dbg(&client->dev, "%s: error setting stream\n", __func__);
	return err;
}

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

	if (priv->group_hold_en == true && gh_prev == SWITCH_OFF) {
		err = imx274_write_reg(priv->s_data,
				       IMX274_GROUP_HOLD_ADDR, 0x1);
		if (err)
			goto fail;
		priv->group_hold_prev = 1;
	} else if (priv->group_hold_en == false && gh_prev == SWITCH_ON) {
		err = imx274_write_reg(priv->s_data,
				       IMX274_GROUP_HOLD_ADDR, 0x0);
		if (err)
			goto fail;
		priv->group_hold_prev = 0;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: Group hold control error\n", __func__);
	return err;
}

static inline void imx274_calculate_gain_regs(imx274_reg *regs, u16 gain)
{
	regs->addr = IMX274_ANALOG_GAIN_ADDR_MSB;
	regs->val = (gain >> IMX274_SHIFT_8_BITS) & IMX274_MASK_LSB_3_BITS;

	(regs + 1)->addr = IMX274_ANALOG_GAIN_ADDR_LSB;
	(regs + 1)->val = (gain) & IMX274_MASK_LSB_8_BITS;
}


/*
 * imx274_set_digital gain - Function called when setting digital gain
 * @priv: Pointer to device structure
 * @dgain: Value of digital gain.
 *
 * Digital gain has only 4 steps: 1x, 2x, 4x, and 8x
 *
 * Return: 0 on success
 */
static int imx274_set_digital_gain(struct imx274 *priv, u32 dgain)
{
	int ret;
	u8 reg_val;

	switch (dgain) {
	case 1:
		reg_val = 0;
		break;
	case 2:
		reg_val = 1;
		break;
	case 4:
		reg_val = 2;
		break;
	case 8:
		reg_val = 3;
		break;
	default:
		return -EINVAL;
	}

	ret = imx274_write_reg(priv->s_data, IMX274_DIGITAL_GAIN_REG,
			       reg_val & IMX274_MASK_LSB_4_BITS);
	return ret;
}


static int imx274_set_gain(struct imx274 *priv, s32 val)
{

	imx274_reg reg_list[2];
	int err;
	u32 gain, analog_gain, digital_gain, gain_reg;
	int i;

	gain = val / 100;

	dev_dbg(&priv->i2c_client->dev,	
		 "%s : input gain = %d.%d\n", __func__,
		 gain >> IMX274_GAIN_SHIFT,
		 ((gain & IMX274_GAIN_SHIFT_MASK) * 100) >> IMX274_GAIN_SHIFT);

	if (gain > IMX274_MAX_DIGITAL_GAIN * IMX274_MAX_ANALOG_GAIN)
		gain = IMX274_MAX_DIGITAL_GAIN * IMX274_MAX_ANALOG_GAIN;
	else if (gain < IMX274_MIN_GAIN)
		gain = IMX274_MIN_GAIN;

	if (gain <= IMX274_MAX_ANALOG_GAIN)
		digital_gain = 1;
	else if (gain <= IMX274_MAX_ANALOG_GAIN * 2)
		digital_gain = 2;
	else if (gain <= IMX274_MAX_ANALOG_GAIN * 4)
		digital_gain = 4;
	else
		digital_gain = IMX274_MAX_DIGITAL_GAIN;

	analog_gain = val / digital_gain;

	dev_dbg(&priv->i2c_client->dev,	
		 "%s : digital gain = %d, analog gain = %d\n",
		 __func__, digital_gain, analog_gain);

	err = imx274_set_digital_gain(priv, digital_gain);
	if (err)
		goto fail;

	/* convert to register value, refer to imx274 datasheet */
	gain_reg = (u32)IMX274_GAIN_CONST -
		(IMX274_GAIN_CONST * 100) / analog_gain;
	if (gain_reg > IMX274_GAIN_REG_MAX)
		gain_reg = IMX274_GAIN_REG_MAX;

	imx274_calculate_gain_regs(reg_list, (u16)gain_reg);

	for (i = 0; i < ARRAY_SIZE(reg_list); i++) {
		err = imx274_write_reg(priv->s_data, reg_list[i].addr,
				       reg_list[i].val);
		if (err)
			goto fail;
	}


	return 0;

fail:
	return err;

#if 0

	imx274_reg reg_list[2];
	int err;
	int i = 0;
	u16 gain;

	dev_dbg(&priv->i2c_client->dev,
		"%s: val: %d\n", __func__, val);

	if (val < IMX274_MIN_GAIN)
		val = IMX274_MIN_GAIN;
	else if (val > IMX274_MAX_GAIN)
		val = IMX274_MAX_GAIN;

	gain = 2048 - (2048 * IMX274_MIN_GAIN / val);

	imx274_get_gain_reg(reg_list, gain);
	imx274_set_group_hold(priv);

	/* writing analog gain */
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
#endif
}

static int imx274_set_frame_length(struct imx274 *priv, s32 val)
{
	imx274_reg reg_list[2];
	int err;
	u32 frame_length;
	u32 frame_rate;
	int i = 0;
	u8 svr;
	u32 vmax;

	dev_dbg(&priv->i2c_client->dev,
		 "%s: val: %u\n", __func__, val);

	frame_length = (u32)val;

	frame_rate = (u32)(IMX274_PIXEL_CLK_HZ /
				(u32)(frame_length * IMX274_LINE_LENGTH));

	imx274_read_reg(priv->s_data, IMX274_SVR_ADDR, &svr);

	vmax = (u32)(72000000 /
			(u32)(frame_rate * IMX274_HMAX * (svr + 1))) - 12;

	imx274_get_vmax_regs(reg_list, vmax);

	imx274_set_group_hold(priv);

	for (i = 0; i < 2; i++) {
		err = imx274_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}
	priv->frame_length = vmax;

	dev_dbg(&priv->i2c_client->dev,
		"%s: frame_rate: %d vmax: %u\n", __func__, frame_rate, vmax);
	return 0;

fail:
	dev_info(&priv->i2c_client->dev,
		 "%s: FRAME_LENGTH control error\n", __func__);
	return err;
}

static int imx274_calculate_shr(struct imx274 *priv, u32 rep)
{
	u8 svr;
	int shr;
	int min;
	int max;
	u8 vmax_l;
	u8 vmax_m;
	u32 vmax;

	imx274_read_reg(priv->s_data, IMX274_SVR_ADDR, &svr);

	imx274_read_reg(priv->s_data, IMX274_VMAX_ADDR_LSB, &vmax_l);
	imx274_read_reg(priv->s_data, IMX274_VMAX_ADDR_MSB, &vmax_m);

	vmax = ((vmax_m << 8) + vmax_l);

	min = IMX274_MODE1_SHR_MIN;
	//max = ((svr + 1) * IMX274_VMAX) - 4;
	max = ((svr + 1) * priv->frame_length) - 4;

	shr = vmax * (svr + 1) -
			(rep * IMX274_ET_FACTOR - IMX274_MODE1_OFFSET) /
			IMX274_HMAX;

	if (shr < min)
		shr = min;

	if (shr > max)
		shr = max;

	dev_dbg(&priv->i2c_client->dev,
		 "%s: shr: %u vmax: %d\n", __func__, shr, vmax);
	return shr;
}

static int imx274_set_coarse_time(struct imx274 *priv, s32 val)
{
	imx274_reg reg_list[2];
	int err;
	u32 coarse_time;
	u32 shr;
	int i = 0;

	coarse_time = val;

	dev_dbg(&priv->i2c_client->dev,
		 "%s: val: %d\n", __func__, coarse_time);

	shr = imx274_calculate_shr(priv, coarse_time);

	imx274_get_shr_regs(reg_list, shr);
	imx274_set_group_hold(priv);

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

static int imx274_verify_streaming(struct imx274 *priv)
{
	int err = 0;

	err = camera_common_s_power(priv->subdev, true);
	if (err)
		return err;

	err = imx274_s_stream(priv->subdev, true);
	if (err)
		goto error;

error:
	imx274_s_stream(priv->subdev, false);
	camera_common_s_power(priv->subdev, false);

	return err;
}

static int imx274_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx274 *priv =
		container_of(ctrl->handler, struct imx274, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		err = imx274_set_gain(priv, ctrl->val);
		break;
	case V4L2_CID_FRAME_LENGTH:
		err = imx274_set_frame_length(priv, ctrl->val);
		break;
	case V4L2_CID_COARSE_TIME:
		err = imx274_set_coarse_time(priv, ctrl->val);
		break;
	case V4L2_CID_COARSE_TIME_SHORT:
		err = imx274_set_coarse_time(priv, ctrl->val);
		break;
	case V4L2_CID_GROUP_HOLD:
		if (switch_ctrl_qmenu[ctrl->val] == SWITCH_ON) {
			priv->group_hold_en = true;
		} else {
			priv->group_hold_en = false;
			err = imx274_set_group_hold(priv);
		}
		break;
	case V4L2_CID_HDR_EN:
		break;
	default:
		pr_err("%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

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
			dev_err(&client->dev, "Failed to init %s ctrl\n",
				ctrl_config_list[i].name);
			continue;
		}

		if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
			ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->p_new.p_char = devm_kzalloc(&client->dev,
				ctrl_config_list[i].max + 1, GFP_KERNEL);
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
	int err;
	int gpio;
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

	err = of_property_read_string(node, "mclk",
				      &board_priv_pdata->mclk_name);
	if (err)
		dev_err(&client->dev, "mclk not in DT\n");

	gpio = of_get_named_gpio(node, "reset-gpios", 0);
	if (gpio < 0)
		gpio = 0;
	board_priv_pdata->reset_gpio = gpio;

	err = of_property_read_string(node, "avdd-reg",
				      &board_priv_pdata->regulators.avdd);
	if (err)
		board_priv_pdata->regulators.avdd = NULL;
	err = of_property_read_string(node, "dvdd-reg",
				      &board_priv_pdata->regulators.dvdd);
	if (err)
		board_priv_pdata->regulators.dvdd = NULL;
	err = of_property_read_string(node, "iovdd-reg",
				      &board_priv_pdata->regulators.iovdd);
	if (err)
		board_priv_pdata->regulators.iovdd = NULL;

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
#ifdef CONFIG_MEDIA_CONTROLLER
	.link_validate = v4l2_subdev_link_validate,
#endif
};

static int imx274_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct device_node *node = client->dev.of_node;
	struct imx274 *priv;
	char debugfs_name[10];
	int err;

	pr_info("[IMX274]: probing v4l2 sensor.\n");

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	common_data = devm_kzalloc(&client->dev,
			    sizeof(struct camera_common_data), GFP_KERNEL);
	if (!common_data) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	priv = devm_kzalloc(&client->dev,
			    sizeof(struct imx274) + sizeof(struct v4l2_ctrl *) *
			    ARRAY_SIZE(ctrl_config_list),
			    GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	priv->regmap = devm_regmap_init_i2c(client, &sensor_regmap_config);
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

	priv->i2c_client		= client;
	priv->s_data			= common_data;
	priv->subdev			= &common_data->subdev;
	priv->subdev->dev		= &client->dev;

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

	err = imx274_verify_streaming(priv);
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
	},
	.probe = imx274_probe,
	.remove = imx274_remove,
	.id_table = imx274_id,
};

module_i2c_driver(imx274_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX274");
MODULE_AUTHOR("Josh Kuo <joshk@nvidia.com>");
MODULE_LICENSE("GPL v2");
