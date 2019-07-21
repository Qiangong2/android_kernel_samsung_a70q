/*
 * Copyright (C) 2018 NXP Semiconductors, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include "tfa9xxx.h"
#include "tfa2_dev.h"
#include "tfa2_haptic.h"
#if defined(TFA_USE_GPIO_FOR_MCLK)
#include <linux/gpio.h>
#endif

#define TFA_HAP_MAX_TIMEOUT (0x7fffff)
#define DEFAULT_PLL_OFF_DELAY 100 /* msec */
#define TIMEOUT 10000
#define MAX_INTENSITY 10000
#define NOMINAL_DURATION 1000

#define USE_SET_TIMER_IN_WORK
#define USE_IMMEDIATE_STOP
#undef USE_SERIALIZED_START
#define USE_EXCLUSIVE_SESSION

static const char *haptic_feature_id_str[]
	= {"wave", "sine", "silence", "illegal"};

#define OFFSET_SET_INDEX	10
#define MAX_NUM_SET_INDEX	13
#define DEFAULT_TONE_SET_INDEX	1
#define SILENCE_SET_INDEX	6
#define LOW_TEMP_INDEX	34 /* special index for low temperature */
#define LOW_TEMP_SET_INDEX	11
static const int set_index_tbl[MAX_NUM_SET_INDEX] = {
	 2,  3, 14, 15, 16,  4, 18, 19, 20, 21,
	22, 23, 24
};

static const char *haptic_feature_id(int id)
{
	if (id > 3)
		id = 3;

	return haptic_feature_id_str[id];
}

static void haptic_dump_obj(struct seq_file *f, struct haptic_tone_object *o)
{
	switch (o->type) {
	case OBJECT_WAVE:
		seq_printf(f, "\t type: %s\n"
			"\t offset: %d\n"
			"\t level: %d%%\n"
			"\t duration_cnt_max: %d (%dms)\n"
			"\t up_samp_sel: %d\n",
			haptic_feature_id(o->type),
			o->freq,
			/* Q1.23, percentage of max */
			(100 * o->level) / 0x7fffff,
			o->duration_cnt_max,
			/* sapmples in time : 1/FS ms */
			o->duration_cnt_max / 48,
			o->boost_brake_on);
		break;
	case OBJECT_TONE:
		seq_printf(f, "\t type: %s\n"
			"\t freq: %dHz\n"
			"\t level: %d%%\n"
			"\t duration_cnt_max: %dms\n"
			"\t boost_brake_on: %d\n"
			"\t tracker_on: %d\n"
			"\t boost_length: %d\n",
			haptic_feature_id(o->type),
			o->freq >> 11, /* Q13.11 */
			/* Q1.23, percentage of max */
			(100 * o->level) / 0x7fffff,
			/* sapmples in time : 1/FS ms */
			o->duration_cnt_max / 48,
			o->boost_brake_on,
			o->tracker_on,
			o->boost_length);
		break;
	case OBJECT_SILENCE:
		seq_printf(f, "\t type: %s\n"
			"\t duration_cnt_max: %dms\n",
			haptic_feature_id(o->type),
			/* sapmples in time : 1/FS ms */
			o->duration_cnt_max / 48);
		break;
	default:
		seq_printf(f, "wrong feature id (%d) in object!\n",
			o->type);
		break;
	}
}

int tfa9xxx_haptic_dump_objs(struct seq_file *f, void *p)
{
	struct tfa9xxx *drv = f->private;
	int i, offset;

	offset = 1;

	for (i = 0; i < FW_XMEM_NR_OBJECTS1; i++) {
		struct haptic_tone_object *o
			= (struct haptic_tone_object *)
			&drv->tfa->hap_data.object_table1_cache[i][0];

		if (o->duration_cnt_max == 0) /* assume object is empty */
			continue;

		seq_printf(f, "object[%d]---------\n", offset + i);
		haptic_dump_obj(f, o);
	}

	offset += i;

	for (i = 0; i < FW_XMEM_NR_OBJECTS2; i++) {
		struct haptic_tone_object *o
			= (struct haptic_tone_object *)
			&drv->tfa->hap_data.object_table2_cache[i][0];

		if (o->duration_cnt_max == 0) /* assume object is empty */
			continue;

		seq_printf(f, "object[%d]---------\n", offset + i);
		haptic_dump_obj(f, o);
	}

	seq_printf(f, "\ndelay_attack: %dms\n",
		drv->tfa->hap_data.delay_attack);

	return 0;
}

static enum hrtimer_restart tfa_haptic_timer_func(struct hrtimer *timer)
{
	struct drv_object *obj = container_of(timer,
		struct drv_object, active_timer);
	struct tfa9xxx *drv = container_of(obj,
		struct tfa9xxx, tone_object);
#if defined(USE_IMMEDIATE_STOP)
	struct tfa2_device *tfa = drv->tfa;
#endif

#if defined(PARALLEL_OBJECTS)
	if (obj->type == OBJECT_TONE)
		drv = container_of(obj, struct tfa9xxx, tone_object);
	else
		drv = container_of(obj, struct tfa9xxx, wave_object);
#endif

	dev_info(&drv->i2c->dev, "%s: type=%d\n", __func__, (int)obj->type);

	obj->state = STATE_STOP;
	drv->is_index_triggered = false;
#if defined(USE_IMMEDIATE_STOP)
	tfa->hap_state = TFA_HAP_STATE_STOPPED;
#endif
	dev_dbg(&drv->i2c->dev, "%s: schedule_work (object state=%d)\n",
		__func__, obj->state);

	schedule_work(&obj->update_work);

	return HRTIMER_NORESTART;
}

static void tfa_haptic_pll_off(struct work_struct *work)
{
	struct tfa9xxx *drv = container_of(work,
		struct tfa9xxx, pll_off_work.work);

	dev_info(&drv->i2c->dev, "%s\n", __func__);

#if defined(TFA_CONTROL_MCLK)
	if (drv->clk_users > 0) {
		dev_dbg(&drv->i2c->dev,
			"%s: skip disabling amplifier / mclk - clk_users %d\n",
			__func__, drv->clk_users);
		return;
	}
#endif /* TFA_CONTROL_MCLK */

	/* turn off PLL */
	tfa2_dev_stop(drv->tfa);
	tfa9xxx_mclk_disable(drv);
}

static void tfa9xxx_haptic_clock(struct tfa9xxx *drv, bool on)
{
	int ret;
#if defined(USE_EXCLUSIVE_SESSION)
	static bool clk_by_haptic;
#endif

	dev_dbg(&drv->i2c->dev, "%s: on=%d, clk_users=%d\n",
		__func__, (int)on, drv->clk_users);

	/* cancel delayed turning off of the PLL */
	if (delayed_work_pending(&drv->pll_off_work))
		cancel_delayed_work_sync(&drv->pll_off_work);

	if (on) {
		ret = tfa9xxx_mclk_enable(drv);
		if (ret == 0) {
			/* moved update of clock_users from tfa9xxx_haptic */
#if !defined(USE_EXCLUSIVE_SESSION)
			drv->clk_users++;
#else
			/* disallow to increase clk_users when overlapped */
			if (!clk_by_haptic)
				drv->clk_users++;
#endif
		}

		/* turn on PLL */
#if !defined(USE_EXCLUSIVE_SESSION)
		if (drv->clk_users == 1) {
#else
		if ((drv->clk_users == 1) && (!clk_by_haptic)) {
#endif
			ret = tfa2_dev_set_state(drv->tfa, TFA_STATE_POWERUP);
			if (ret < 0) {
				dev_err(&drv->i2c->dev, "%s: error in powering up (%d)\n",
					__func__, ret);
				drv->clk_users = 0;
				return;
			}
			ret = tfa2_dev_clock_stable_wait(drv->tfa);
			if (ret < 0) {
				dev_err(&drv->i2c->dev, "%s: clock is not stable\n",
					__func__);
				drv->clk_users = 0;
				return;
			}
		}
#if defined(USE_EXCLUSIVE_SESSION)
		clk_by_haptic = true;
#endif
	} else if (drv->clk_users > 0) {
#if !defined(USE_EXCLUSIVE_SESSION)
		drv->clk_users--;
#else
		/* allow to decrease clk_users when called in pair */
		if (clk_by_haptic)
			drv->clk_users--;
		clk_by_haptic = false;
#endif
		if (drv->clk_users <= 0) {
			drv->clk_users = 0;
			/* turn off PLL with a delay */
			schedule_delayed_work(&drv->pll_off_work,
				msecs_to_jiffies(drv->pll_off_delay));
		}
	}
}

int tfa9xxx_disable_f0_tracking(struct tfa9xxx *drv, bool disable)
{
	int ret = 0;

	dev_dbg(&drv->i2c->dev, "%s: disable=%d, f0_trc_users=%d\n",
		__func__, (int)disable, drv->f0_trc_users);

	if (disable) {
		if (drv->f0_trc_users == 0)
			/* disable*/
			ret = tfa2_haptic_disable_f0_trc(drv->i2c, 1);
		drv->f0_trc_users++;
	} else if (drv->f0_trc_users > 0) {
		drv->f0_trc_users--;
		if (drv->f0_trc_users == 0)
			/* enable again */
			ret = tfa2_haptic_disable_f0_trc(drv->i2c, 0);
	}

	return ret;
}

static int tfa9xxx_update_duration(struct tfa9xxx *drv, int index)
{
	int rc;
	struct tfa2_device *tfa = drv->tfa;

	drv->update_object_index = false;

	if (index < 0) {
#if 0
		rc = tfa9xxx_store_value(dev, attr, buf, count);
		if (rc < 0)
			return rc;
#endif
	} else if ((index > 0)
		&& (index <= FW_HB_SEQ_OBJ + tfa->hap_data.seq_max)) {
		/*
		 * If the value is less or equal than number of objects in table
		 * plus the number of virtual objetcs, we assume it is an
		 * object index from App layer. Note that Android is not able
		 * too pass a 0 value, so the object index has an offset of 1
		 */
		drv->object_index = index - 1;
		drv->update_object_index = true;
		drv->is_index_triggered = true;
		tfa->hap_data.index = drv->object_index;
		dev_dbg(&drv->i2c->dev, "%s: new object index is %d\n",
			__func__, drv->object_index);
	} else if (index > FW_HB_SEQ_OBJ + tfa->hap_data.seq_max) {
		dev_dbg(&drv->i2c->dev, "%s: duration = %d\n", __func__, index);
		rc = tfa2_haptic_update_duration(&tfa->hap_data, index);
		if (rc < 0)
			return rc;
	}
	
	return 0;
}

static void tfa9xxx_update_led_class(struct work_struct *work)
{
	struct drv_object *obj = container_of(work,
		struct drv_object, update_work);
	struct tfa9xxx *drv = container_of(obj,
		struct tfa9xxx, tone_object);
	struct haptic_data *data;

#if defined(PARALLEL_OBJECTS)
	if (obj->type == OBJECT_TONE)
		drv = container_of(obj, struct tfa9xxx, tone_object);
	else
		drv = container_of(obj, struct tfa9xxx, wave_object);
#endif

	/* should have been checked before scheduling this work */
	BUG_ON(!drv->patch_loaded);

	data = &drv->tfa->hap_data;

	dev_info(&drv->i2c->dev, "[VIB] %s: type: %d, index: %d, state: %d\n",
		__func__, (int)obj->type, obj->index, obj->state);

	if (obj->state != STATE_STOP) {
		if (obj->state == STATE_RESTART)
			tfa2_haptic_stop(drv->tfa, data, obj->index);
		else
			tfa9xxx_haptic_clock(drv, true);

#if defined(USE_SET_TIMER_IN_WORK)
		/* run ms timer */
		hrtimer_start(&obj->active_timer,
			ktime_set(drv->object_duration / 1000,
			(drv->object_duration % 1000) * 1000000),
			HRTIMER_MODE_REL);

		dev_dbg(&drv->i2c->dev, "%s: start active %d timer of %d msecs\n",
			__func__, (int)obj->type, drv->object_duration);
#endif /* USE_SET_TIMER_IN_WORK */

		if (drv->update_object_intensity)
			data->amplitude = drv->object_intensity;
		drv->running_index = obj->index;
		if (tfa2_haptic_start(drv->tfa, data, obj->index))
			dev_err(&drv->i2c->dev, "[VIB] %s: problem when starting\n",
				__func__);
	} else {
		drv->update_object_intensity = false;
		drv->running_index = 0; /* reset */
		tfa2_haptic_stop(drv->tfa, data, obj->index);
		tfa9xxx_haptic_clock(drv, false);
	}
}

static void tfa9xxx_set_intensity(struct tfa9xxx *drv,
	enum led_brightness brightness)
{
	dev_dbg(&drv->i2c->dev, "%s: brightness = %d\n",
		__func__, brightness);

	/* set amplitude scaled to 0 - 100% */
	drv->update_object_intensity = true;
	drv->object_intensity = (100 * brightness + LED_HALF) / LED_FULL;
}

static ssize_t tfa9xxx_store_value(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct tfa9xxx *drv = container_of(cdev, struct tfa9xxx, led_dev);
	struct tfa2_device *tfa = drv->tfa;
	struct haptic_data *data = &tfa->hap_data;
	long value = 0;
	int ret;

	ret = kstrtol(buf, 10, &value);
	if (ret)
		return ret;

	ret = tfa2_haptic_parse_value(data, value);
	if (ret)
		return -EINVAL;

	dev_info(&drv->i2c->dev, "%s: index=%d, amplitude=%d, frequency=%d\n",
		__func__, data->index, data->amplitude, data->frequency);

	return count;
}

static ssize_t tfa9xxx_show_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct tfa9xxx *drv = container_of(cdev, struct tfa9xxx, led_dev);
	struct tfa2_device *tfa = drv->tfa;
	int size;

	size = snprintf(buf, 80,
		"index=%d, amplitude=%d, frequency=%d, duration=%d\n",
		tfa->hap_data.index + 1, tfa->hap_data.amplitude,
		tfa->hap_data.frequency, tfa->hap_data.duration);

	return size;
}

static ssize_t tfa9xxx_store_state(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t tfa9xxx_show_state(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct tfa9xxx *drv = container_of(cdev, struct tfa9xxx, led_dev);
	int state = (drv->tone_object.state != STATE_STOP);
	int size;

#if defined(PARALLEL_OBJECTS)
	state = state || (drv->wave_object.state != STATE_STOP);
#endif

	size = snprintf(buf, 10, "%d\n", state);

	return size;
}

static int vibrator_enable(struct tfa9xxx *drv, int val)
{
	struct drv_object *obj = &drv->tone_object;
	struct tfa2_device *tfa = drv->tfa;
	int state;
	int duration;
#if defined(PARALLEL_OBJECTS)
	bool object_is_tone;
#endif

	pr_info("[VIB] %s: value: %d ms\n", __func__, val);

	if (!drv->patch_loaded) {
		dev_err(&drv->i2c->dev, "%s: patch not loaded\n", __func__);
		return -EINVAL;
	}

	hrtimer_cancel(&obj->active_timer);

	state = (val != 0);

	dev_dbg(&drv->i2c->dev, "%s: update_object_index=%d, val=%d\n",
		__func__, (int)drv->update_object_index, val);

#if defined(PARALLEL_OBJECTS)
	object_is_tone = (tfa2_haptic_object_type(&tfa->hap_data,
		drv->object_index) == OBJECT_TONE);
	if (object_is_tone)
		obj = &drv->tone_object;
	else
		obj = &drv->wave_object;
#endif

	if (drv->update_object_index == true) {
		pr_info("[VIB] %s: index is updated\n", __func__);
		drv->update_object_index = false;
		obj->index = drv->object_index;
		return 0;
	}

#if 0
	/* when no bck and no mclk, we can not play vibrations */
#if (defined(TFA_CONTROL_MCLK) && defined(TFA_USE_GPIO_FOR_MCLK))
	if (!drv->bck_running) {
		if (!gpio_is_valid(drv->mclk_gpio)) {
			dev_warn(&drv->i2c->dev,
				"%s: no active clock\n", __func__);
			return -ENODEV;
		}

		if (gpio_get_value_cansleep(drv->mclk_gpio) == 0) {
			dev_warn(&drv->i2c->dev,
				"%s: no active clock\n", __func__);
			return -ENODEV;
		}

		dev_dbg(&drv->i2c->dev, "%s: mclk is active\n", __func__);
	} else {
		dev_dbg(&drv->i2c->dev, "%s: bck is active\n", __func__);
	}
#else
	if (!drv->bck_running && !drv->mclk) {
		dev_warn(&drv->i2c->dev, "%s: no active clock\n", __func__);
		return -ENODEV;
	}
#endif
#endif

	dev_dbg(&drv->i2c->dev, "%s: state=%d, object state=%d\n",
		__func__, state, obj->state);

	if (state == 0) {
		if (obj->state == STATE_STOP) {
			pr_info("[VIB]: OFF\n");
			drv->is_index_triggered = false;
			return 0;
		}
		obj->state = STATE_STOP;
	} else {
		if (obj->state == STATE_STOP) {
			obj->state = STATE_START;
		} else {
			dev_dbg(&drv->i2c->dev,
				"%s: restart, clk_users=%d\n",
				__func__, drv->clk_users);
			obj->state = STATE_RESTART;
		}
	}

	if (obj->state != STATE_STOP) {
		dev_dbg(&drv->i2c->dev,
			"%s: state=%d, running index=%d\n",
			__func__, obj->state, drv->running_index);
#if defined(USE_SERIALIZED_START)
		if (drv->running_index < FW_HB_SEQ_OBJ) {
			dev_dbg(&drv->i2c->dev,
				"%s: flush queued work\n", __func__);
			/* serialize non-blocking objects */
			flush_work(&obj->update_work);
		}
#endif

		if (drv->object_index == DEFAULT_TONE_SET_INDEX - 1) {
			/* default tone - SS service call */
			duration = val;
			duration = (duration > TIMEOUT)
				? TIMEOUT : duration;
		} else {
			/* object duration + 5 msecs */
			duration = tfa2_haptic_get_duration
				(tfa, obj->index) + 5;
			/* clip value to max */
			duration = (duration > drv->timeout)
				? drv->timeout : duration;
		}

#if defined(USE_SET_TIMER_IN_WORK)
		drv->object_duration = duration;
#else
		/* run ms timer */
		hrtimer_start(&obj->active_timer,
			ktime_set(duration / 1000,
			(duration % 1000) * 1000000),
			HRTIMER_MODE_REL);

		dev_dbg(&drv->i2c->dev, "%s: start active %d timer of %d msecs\n",
			__func__, (int)obj->type, duration);
#endif /* USE_SET_TIMER_IN_WORK */
	}

#if defined(USE_IMMEDIATE_STOP)
	if (obj->state == STATE_STOP) {
		dev_dbg(&drv->i2c->dev, "%s: stop (object state=%d)\n",
			__func__, obj->state);

		drv->update_object_intensity = false;
		drv->running_index = 0; /* reset */
		tfa2_haptic_stop(tfa, &tfa->hap_data, obj->index);
		tfa9xxx_haptic_clock(drv, false);

		return 0;
	}
#endif /* USE_IMMEDIATE_STOP */

	dev_dbg(&drv->i2c->dev, "%s: schedule_work (object state=%d)\n",
		__func__, obj->state);
	schedule_work(&obj->update_work);

	return 0;
}

static ssize_t enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct tfa9xxx *drv = dev_get_drvdata(dev);
	u32 val;
	static bool checked_nominal_call;
	int rc;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val == 0)
		checked_nominal_call = false;

	if ((!checked_nominal_call)
		&& (drv->object_index == DEFAULT_TONE_SET_INDEX - 1)
		&& (val == NOMINAL_DURATION)) {
		/* default tone - SS service call */
		dev_dbg(&drv->i2c->dev, "%s: skip nominal call (index=%d)\n",
			__func__, drv->object_index);
		checked_nominal_call = true;
		return count;
	}

	checked_nominal_call = false;

	vibrator_enable(drv, val);

	return count;
}

int tfa9xxx_bck_starts(struct tfa9xxx *drv)
{
#if defined(TFA_USE_GPIO_FOR_MCLK)
	dev_dbg(&drv->i2c->dev, "%s: bck_running=%d, mclk_gpio=%d\n",
		__func__, (int)drv->bck_running,
		(int)gpio_is_valid(drv->mclk_gpio));
#else
	dev_dbg(&drv->i2c->dev, "%s: bck_running=%d, mclk=%p\n", __func__,
		(int)drv->bck_running, drv->mclk);
#endif

	if (drv->bck_running)
		return 0;

	dev_info(&drv->i2c->dev, "%s: clk_users %d\n",
		__func__, drv->clk_users);

	/* clock enabling is handled in tfa_start(), so update reference count
	 * instead of calling tfa9xxx_haptic_clock(drv, true)
	 */
	/* moved the update of clock_users to tfa9xxx */
#if !defined(TFA_CONTROL_MCLK)
	drv->clk_users++;
#endif

	drv->bck_running = true;

	return 0;
}

int tfa9xxx_bck_stops(struct tfa9xxx *drv)
{
	struct drv_object *obj;

#if defined(TFA_USE_GPIO_FOR_MCLK)
	dev_dbg(&drv->i2c->dev, "%s: bck_running=%d, mclk_gpio=%d\n",
		__func__, (int)drv->bck_running,
		(int)gpio_is_valid(drv->mclk_gpio));
#else
	dev_dbg(&drv->i2c->dev, "%s: bck_running=%d, mclk=%p\n", __func__,
		(int)drv->bck_running, drv->mclk);
#endif

	if (!drv->bck_running)
		return 0;

	dev_info(&drv->i2c->dev, "%s: clk_users %d\n",
		__func__, drv->clk_users);

	drv->bck_running = false;

	/* clock disabling is handled in tfa_stop(), so update reference count
	 * instead of calling tfa9xxx_haptic_clock(drv, false)
	 */
	/* moved the update of clock_users to tfa9xxx */
#if !defined(TFA_CONTROL_MCLK)
	drv->clk_users--;
	if (drv->clk_users < 0)
		drv->clk_users = 0;
#endif

#if defined(TFA_USE_GPIO_FOR_MCLK)
	if (gpio_is_valid(drv->mclk_gpio))
		return 0;
#else
	if (drv->mclk)
		return 0;
#endif

	/* When there is no mclk, stop running objects */
#if defined(PARALLEL_OBJECTS)
	obj = &drv->wave_object;
	if (obj->state != STATE_STOP) {
		obj->state = STATE_STOP;
		hrtimer_cancel(&obj->active_timer);
		schedule_work(&obj->update_work);
		flush_work(&obj->update_work);
	}
#endif
	obj = &drv->tone_object;
	if (obj->state != STATE_STOP) {
		obj->state = STATE_STOP;
		hrtimer_cancel(&obj->active_timer);
		schedule_work(&obj->update_work);
		flush_work(&obj->update_work);
	}

	return 0;
}

static ssize_t tfa9xxx_motor_type_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct tfa9xxx *drv = dev_get_drvdata(dev);

	pr_info("%s: %s\n", __func__, drv->vib_type);
	return snprintf(buf, MAX_LEN_VIB_TYPE, "%s\n", drv->vib_type);
}

static DEVICE_ATTR(motor_type, 0660, tfa9xxx_motor_type_show, NULL);

static ssize_t tfa9xxx_pattern_length_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct tfa9xxx *drv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", drv->pattern_duration);
}

static ssize_t tfa9xxx_pattern_length_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct tfa9xxx *drv = dev_get_drvdata(dev);
//	struct tfa2_device *tfa = drv->tfa;
	int ret;
	unsigned int index = 0;

	ret = kstrtou32(buf, 10, &index);
	if (ret)
		return -EINVAL;
#if 0
	if (index == LOW_TEMP_INDEX) {
		pr_info("%s low temperature vibration pattern\n", __func__);
		set_index = LOW_TEMP_SET_INDEX;
	} else if (index == 0) {
		set_index = 1; /* default vibration */
	} else if (index >= OFFSET_SET_INDEX + MAX_NUM_SET_INDEX) {
		set_index = 6; /* silence */
	} else if (index >= OFFSET_SET_INDEX
		&& index < OFFSET_SET_INDEX + MAX_NUM_SET_INDEX) {
		set_index = set_index_tbl[index - OFFSET_SET_INDEX];
	}

	pr_info("%s index: %u set_index: %u \n", __func__, index, set_index);

	drv->pattern_duration = tfa2_haptic_get_duration(tfa, set_index);

	dev_dbg(&drv->i2c->dev, "%s: update_object_index=%d, index=%d duration=%d\n",
		__func__, (int)drv->update_object_index, set_index, drv->pattern_duration);
#endif

	if (index == 10)
		drv->pattern_duration = 7;
	else if (index == 11)
		drv->pattern_duration = 7;
	else if (index == 12)
		drv->pattern_duration = 100;
	else if (index == 13)
		drv->pattern_duration = 180;
	else if (index == 14)
		drv->pattern_duration = 85;
	else if (index == 15)
		drv->pattern_duration = 7;
	else if (index == 16)
		drv->pattern_duration = 475;
	else if (index == 17)
		drv->pattern_duration = 500;
	else if (index == 18)
		drv->pattern_duration = 1500;
	else if (index == 19)
		drv->pattern_duration = 1000;
	else if (index == 20)
		drv->pattern_duration = 3500;
	else if (index == 21)
		drv->pattern_duration = 2000;
	else if (index == 22)
		drv->pattern_duration = 1600;

	return count;
}

static DEVICE_ATTR(pattern_length, 0660, tfa9xxx_pattern_length_show, tfa9xxx_pattern_length_store);

static ssize_t tfa9xxx_cp_trigger_index_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct tfa9xxx *drv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", drv->cp_trigger_index);
}

static ssize_t tfa9xxx_cp_trigger_index_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct tfa9xxx *drv = dev_get_drvdata(dev);
	struct drv_object *obj = &drv->tone_object;
#if defined(PARALLEL_OBJECTS)
	struct tfa2_device *tfa = drv->tfa;
#endif
	int ret;
	unsigned int index, set_index;

	ret = kstrtou32(buf, 10, &index);
	if (ret)
		return -EINVAL;

	if (index == LOW_TEMP_INDEX) {
		pr_info("%s low temperature vibration pattern\n", __func__);
		set_index = LOW_TEMP_SET_INDEX;
	} else if (index == 0) {
		set_index = DEFAULT_TONE_SET_INDEX; /* default vibration */
	}	else if (index >= OFFSET_SET_INDEX + MAX_NUM_SET_INDEX) {
		set_index = SILENCE_SET_INDEX; /* silence */
	} else if (index >= OFFSET_SET_INDEX
		&& index < OFFSET_SET_INDEX + MAX_NUM_SET_INDEX) {
		set_index = set_index_tbl[index - OFFSET_SET_INDEX];
	}

	pr_info("%s index: %u set_index: %u \n", __func__, index, set_index);

	tfa9xxx_update_duration(drv, set_index);
	drv->cp_trigger_index = index;

	dev_dbg(&drv->i2c->dev, "%s: update_object_index=%d, index=%d\n",
		__func__, (int)drv->update_object_index, set_index);

#if defined(PARALLEL_OBJECTS)
	object_is_tone = (tfa2_haptic_object_type(&tfa->hap_data,
		drv->object_index) == OBJECT_TONE);
	if (object_is_tone)
		obj = &drv->tone_object;
	else
		obj = &drv->wave_object;
#endif

	if (drv->update_object_index == true) {
		pr_info("[VIB] %s: index is updated\n", __func__);
		drv->update_object_index = false;
		obj->index = drv->object_index;
	}

	return count;
}

static DEVICE_ATTR(cp_trigger_index, 0660, tfa9xxx_cp_trigger_index_show, tfa9xxx_cp_trigger_index_store);

static ssize_t enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tfa9xxx *drv = dev_get_drvdata(dev);
	int size;

#if defined(PARALLEL_OBJECTS)
	int state = (drv->tone_object.state != STATE_STOP) ||
		(drv->wave_object.state != STATE_STOP);
#else
	int state = (drv->tone_object.state != STATE_STOP);
#endif

	size = snprintf(buf, 10, "%d\n", state);

	return size;
}

static DEVICE_ATTR(enable, 0660, enable_show, enable_store);

static ssize_t intensity_store(struct device *dev,
		struct device_attribute *devattr, const char *buf, size_t count)
{
	struct tfa9xxx *drv = dev_get_drvdata(dev);
	int ret = 0, set_intensity = 0;

	ret = kstrtoint(buf, 0, &set_intensity);
	if (ret) {
		pr_err("[VIB]: %s failed to get intensity", __func__);
		return ret;
	}

	if ((set_intensity < 0) || (set_intensity > MAX_INTENSITY)) {
		pr_err("[VIB]: %sout of rage\n", __func__);
		return -EINVAL;
	}

	drv->intensity = set_intensity;
	set_intensity = (drv->tfa2_fw_intensity_max * set_intensity) / MAX_INTENSITY;

	pr_info("[VIB] %s intensity : %d brightness : %d, max_brightness: %d\n",
	  __func__, drv->intensity, set_intensity, drv->tfa2_fw_intensity_max);

	tfa9xxx_set_intensity(drv, set_intensity);

	return count;
}

static ssize_t intensity_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tfa9xxx *drv = dev_get_drvdata(dev);

	return sprintf(buf, "intensity: %u\n", drv->intensity);
}

static DEVICE_ATTR(intensity, 0660, intensity_show, intensity_store);

static ssize_t tfa9xxx_store_duration(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	int rc;
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct tfa9xxx *drv = container_of(cdev, struct tfa9xxx, led_dev);

	rc = kstrtoint(buf, 0, &val);
	if (rc < 0)
		return rc;

	dev_dbg(&drv->i2c->dev, "%s: val=%d\n", __func__, val);

	tfa9xxx_update_duration(drv, val);

	return count;
}

static ssize_t tfa9xxx_show_duration(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return tfa9xxx_show_value(dev, attr, buf);
}

static ssize_t tfa9xxx_show_f0(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tfa9xxx *drv = dev_get_drvdata(dev);
	int ret, f0;
	int size;

	ret = tfa2_haptic_read_f0(drv->i2c, &f0);
	if (ret)
		return -EIO;

	size = snprintf(buf, 15, "%d.%03d\n",
		TFA2_HAPTIC_FP_INT(f0, FW_XMEM_F0_SHIFT),
		TFA2_HAPTIC_FP_FRAC(f0, FW_XMEM_F0_SHIFT));

	return size;
}

static ssize_t tfa9xxx_store_pll_off_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct tfa9xxx *drv = dev_get_drvdata(dev);
	int err;

	err = kstrtouint(buf, 10, &drv->pll_off_delay);
	if (err)
		return err;

	return count;
}

static ssize_t tfa9xxx_show_pll_off_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tfa9xxx *drv = dev_get_drvdata(dev);
	int size;

	size = snprintf(buf, 10, "%u\n", drv->pll_off_delay);

	return size;
}

static struct device_attribute tfa9xxx_haptic_attrs[] = {
	__ATTR(value, 0664, tfa9xxx_show_value, tfa9xxx_store_value),
	__ATTR(state, 0664, tfa9xxx_show_state, tfa9xxx_store_state),
	__ATTR(duration, 0664, tfa9xxx_show_duration, tfa9xxx_store_duration),
};

static DEVICE_ATTR(f0, 0444, tfa9xxx_show_f0, NULL);
static DEVICE_ATTR(pll_off_delay, 0644, tfa9xxx_show_pll_off_delay,
	tfa9xxx_store_pll_off_delay);


int tfa9xxx_haptic_probe(struct tfa9xxx *drv)
{
	struct i2c_client *i2c = drv->i2c;
	int failed_attr_idx = 0;
	int rc, i;

	dev_info(&drv->i2c->dev, "%s\n", __func__);

	drv->timeout = TFA_HAP_MAX_TIMEOUT;
	drv->pll_off_delay = DEFAULT_PLL_OFF_DELAY;

	INIT_WORK(&drv->tone_object.update_work, tfa9xxx_update_led_class);
#if defined(PARALLEL_OBJECTS)
	INIT_WORK(&drv->wave_object.update_work, tfa9xxx_update_led_class);
#endif
	INIT_DELAYED_WORK(&drv->pll_off_work, tfa_haptic_pll_off);

	drv->tone_object.type = OBJECT_TONE;
#if defined(PARALLEL_OBJECTS)
	drv->wave_object.type = OBJECT_WAVE;
#endif
	/* add "f0" entry to i2c device */
	rc = device_create_file(&i2c->dev, &dev_attr_f0);
	if (rc < 0) {
		dev_err(&drv->i2c->dev, "%s: Error, creating sysfs f0 entry\n",
			__func__);
		goto error_sysfs_f0;
	}

	/* add "pll_off_delay" entry to i2c device */
	rc = device_create_file(&i2c->dev, &dev_attr_pll_off_delay);
	if (rc < 0) {
		dev_err(&drv->i2c->dev, "%s: Error, creating sysfs pll_off_delay entry\n",
			__func__);
		goto error_sysfs_pll;
	}

	/* active timers for led driver */
	hrtimer_init(&drv->tone_object.active_timer,
		CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	drv->tone_object.active_timer.function = tfa_haptic_timer_func;

#if defined(PARALLEL_OBJECTS)
	hrtimer_init(&drv->wave_object.active_timer,
		CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	drv->wave_object.active_timer.function = tfa_haptic_timer_func;
#endif

	/* led driver */
	drv->led_dev.name = "vibrator";
	drv->led_dev.default_trigger = "none";

	rc = led_classdev_register(&i2c->dev, &drv->led_dev);
	if (rc < 0) {
		dev_err(&drv->i2c->dev, "%s: Error, led device\n", __func__);
		goto error_led_class;
	}

	for (i = 0; i < ARRAY_SIZE(tfa9xxx_haptic_attrs); i++) {
		rc = sysfs_create_file(&drv->led_dev.dev->kobj,
			&tfa9xxx_haptic_attrs[i].attr);
		if (rc < 0) {
			dev_err(&drv->i2c->dev,
				"%s: Error, creating sysfs entry %d rc=%d\n",
				__func__, i, rc);
			failed_attr_idx = i;
			goto error_sysfs_led;
		}
	}

	drv->to_class = class_create(THIS_MODULE, "timed_output");
	if (IS_ERR(drv->to_class))
		pr_err("[VIB]: timed_output classs create fail\n");

	drv->to_dev = device_create(drv->to_class, NULL, 0, drv, "vibrator");
	if (IS_ERR(drv->to_dev))
		return PTR_ERR(drv->to_dev);

	rc = sysfs_create_file(&drv->to_dev->kobj, &dev_attr_enable.attr);
	if (rc < 0)
		pr_err("[VIB]: Failed to register sysfs enable: %d\n", rc);

	rc = sysfs_create_file(&drv->to_dev->kobj, &dev_attr_intensity.attr);
	if (rc < 0)
		pr_err("[VIB]: Failed to register sysfs intensity: %d\n", rc);

	rc = sysfs_create_file(&drv->to_dev->kobj, &dev_attr_cp_trigger_index.attr);
	if (rc < 0)
		pr_err("[VIB]: Failed to register sysfs cp_trigger_index: %d\n", rc);

	rc = sysfs_create_file(&drv->to_dev->kobj, &dev_attr_motor_type.attr);
	if (rc < 0)
		pr_err("[VIB]: Failed to register sysfs motor type: %d\n", rc);

	rc = sysfs_create_file(&drv->to_dev->kobj, &dev_attr_pattern_length.attr);
	if (rc < 0)
		pr_err("[VIB]: Failed to register sysfs pattern length: %d\n", rc);

	drv->intensity = MAX_INTENSITY;

	return 0;

error_sysfs_led:
	for (i = 0; i < failed_attr_idx; i++)
		sysfs_remove_file(&drv->led_dev.dev->kobj,
			&tfa9xxx_haptic_attrs[i].attr);
	led_classdev_unregister(&drv->led_dev);

error_led_class:
	device_remove_file(&i2c->dev, &dev_attr_pll_off_delay);

error_sysfs_pll:
	device_remove_file(&i2c->dev, &dev_attr_f0);

error_sysfs_f0:
	return rc;
}

int tfa9xxx_haptic_remove(struct tfa9xxx *drv)
{
	struct i2c_client *i2c = drv->i2c;
	int i;

	hrtimer_cancel(&drv->tone_object.active_timer);
#if defined(PARALLEL_OBJECTS)
	hrtimer_cancel(&drv->wave_object.active_timer);
#endif

	device_remove_file(&i2c->dev, &dev_attr_f0);
	device_remove_file(&i2c->dev, &dev_attr_pll_off_delay);

	for (i = 0; i < ARRAY_SIZE(tfa9xxx_haptic_attrs); i++) {
		sysfs_remove_file(&drv->led_dev.dev->kobj,
			&tfa9xxx_haptic_attrs[i].attr);
	}

	led_classdev_unregister(&drv->led_dev);

	cancel_work_sync(&drv->tone_object.update_work);
#if defined(PARALLEL_OBJECTS)
	cancel_work_sync(&drv->wave_object.update_work);
#endif
	cancel_delayed_work_sync(&drv->pll_off_work);

	return 0;
}

int __init tfa9xxx_haptic_init(void)
{
	return 0;
}

void __exit tfa9xxx_haptic_exit(void)
{
}

