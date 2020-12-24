/*
	MIT License

	Copyright (c) 2020 Vasily Kiniv

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/input/mt.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

#include "fbtft.h"

#define DRVNAME				"fb_gud"
#define WIDTH					256
#define HEIGHT				128
#define TOUCH_SW_NUM 	32
#define TOUCH_X_MAX 	7
#define TOUCH_Y_MAX 	3

#define DATA_WRITE_CMD 0x44
#define DATA_READ_CMD 0x54
#define STATUS_READ_CMD 0x58

struct gud_data {
	struct input_dev		*input_dev;
	struct touchscreen_properties props;
	struct timer_list timer;
	struct fbtft_par *par;
	struct work_struct poll_work;
	bool lock;
	unsigned int		irq;
};

static uint32_t      m_touch_status = 0;

static void register_onboard_backlight(struct fbtft_par *par);
static void thread_function(struct work_struct *work);

static u8 available(struct fbtft_par *par)
{
	struct gud_data *data = par->extra;
	u8 txbuf[8] = { 0 };
	u8 rxbuf[8] = { 0 };
	int res = 0;

	struct spi_transfer t = {
		.tx_buf = txbuf,
		.rx_buf = rxbuf,
		.len = 8,
		.speed_hz = par->spi->max_speed_hz,
	};
	struct spi_message m;

	txbuf[0] = STATUS_READ_CMD;

	if (!par->spi) {
		dev_err(par->info->device,
			"%s: par->spi is unexpectedly NULL\n", __func__);
		return -1;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	while(gpio_get_value(par->gpio.dc));
	res = spi_sync(par->spi, &m);

	return rxbuf[6];
}

static int read(struct fbtft_par *par, void *buf, size_t len)
{
	struct gud_data *data = par->extra;
	u8 txbuf[256] = { 0 };
	u8 rxbuf[256] = { 0 };
	int res = 0;

	struct spi_transfer t = {
		.tx_buf = txbuf,
		.rx_buf = rxbuf,
		.len = len + 3,
		.speed_hz = par->spi->max_speed_hz,
	};
	struct spi_message m;

	txbuf[0] = DATA_READ_CMD;

	if (!par->spi) {
		dev_err(par->info->device,
			"%s: par->spi is unexpectedly NULL\n", __func__);
		return -1;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	while(gpio_get_value(par->gpio.dc));
	// data->lock = true;
	res = spi_sync(par->spi, &m);
	// data->lock = false;

	memcpy(buf, &rxbuf[2], len);

	return res;
}

static int write(struct fbtft_par *par, void *buf, size_t len)
{
	struct gud_data *data = par->extra;
	int res;
	u8 *txbuf = buf;
	size_t i;

	struct spi_transfer t = {
		.tx_buf = txbuf,
		.len = len + 1,
		.speed_hz = par->spi->max_speed_hz,
	};
	struct spi_message m;

	for (i = len; i > 0; i--) {
		txbuf[i] = txbuf[i - 1];
	}

	txbuf[0] = DATA_WRITE_CMD;

	if (!par->spi) {
		dev_err(par->info->device,
			"%s: par->spi is unexpectedly NULL\n", __func__);
		return -1;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	while(gpio_get_value(par->gpio.dc));
	res = spi_sync(par->spi, &m);

	return res;
}

static void setup_touch_input(struct fbtft_par *par)
{
	struct gud_data *data = par->extra;
	struct input_dev *input_dev;
	int error;

	input_dev = devm_input_allocate_device(&par->spi->dev);
	if (!data || !input_dev) {
		dev_err(&par->spi->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	data->input_dev = input_dev;

	input_dev->name = "gud_ts";
	input_dev->phys = "input/ts";

	input_set_abs_params(input_dev, ABS_X, 0, TOUCH_X_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, TOUCH_Y_MAX, 0, 0);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, TOUCH_X_MAX, 0, 0);
	input_abs_set_res(input_dev, ABS_MT_POSITION_X, TOUCH_X_MAX);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, TOUCH_Y_MAX, 0, 0);
	input_abs_set_res(input_dev, ABS_MT_POSITION_Y, TOUCH_Y_MAX);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MINOR, 0, 1, 0, 0);

	input_mt_init_slots(input_dev, TOUCH_SW_NUM, INPUT_MT_DIRECT);
	touchscreen_parse_properties(input_dev, false, &data->props);


	error = input_register_device(data->input_dev);
	if (error) {
		dev_err(&par->spi->dev, "Failed to register interrupt\n");
	}
}

static u32 read_touch_status(struct fbtft_par *par, bool *valid) {
	struct gud_data *data = par->extra;
	uint32_t status = 0;
	uint8_t rx[512] = {0};
	uint8_t len = 0;

	if(data->lock) {
		*valid = false;
		return 0;

	}
	data->lock = true;

	write_reg(par, 0x1f, 0x4b, 0x10);
	len = available(par);
	read(par, rx, len);

	if(rx[0] == 0x10 && rx[1] == 4 && len == 6) {
		status =  (rx[2] <<  24)  |
							(rx[3] <<  16)  |
							(rx[4] <<  8)   |
							(rx[5] <<  0);
		*valid = true;
	} else {
		*valid = false;
	}

	data->lock = false;

	return status;
}

static bool check_switch(uint32_t status, uint8_t switch_num)
{
  return status & (1 << switch_num);
}

static void touch_cb(struct gud_data *data)
{
	struct input_dev *dev = data->input_dev;
  uint32_t status = 0;
  uint16_t x = 0;
  uint16_t y = 0;
	int i = 0;
	bool valid = false;

  status = read_touch_status(data->par, &valid);

	if(dev == NULL) {
		return;
	}

  if(status != m_touch_status && valid) {
    for (i = 0; i < TOUCH_SW_NUM; i++)
    {
      bool prev = check_switch(m_touch_status, i);
      bool curr = check_switch(status, i);

      if(prev != curr) {
        y = (i / 8) % 4;
        x = i % 8;

				input_mt_slot(dev, i);
				touchscreen_report_pos(dev, &data->props, x, y, true);
				input_mt_report_slot_state(dev, MT_TOOL_FINGER, curr);
      }
    }

		input_mt_sync_frame(dev);
		input_sync(dev);

    m_touch_status = status;
  }
}

static void thread_function(struct work_struct *work){
	struct gud_data *data = container_of(work, struct gud_data, poll_work);

	set_current_state(TASK_INTERRUPTIBLE);

	schedule_timeout(HZ / 20);
	touch_cb(data);
	
	schedule_work(&data->poll_work);

	return;
}

static int init_display(struct fbtft_par *par)
{
	int err;
	struct gud_data *data;

	data = devm_kzalloc(&par->spi->dev, sizeof(*data), GFP_KERNEL);
	data->lock = false;
	data->par = par;
	par->extra = data;

	if (par->pdata &&
	    par->pdata->display.backlight == FBTFT_ONBOARD_BACKLIGHT) {
		par->fbtftops.register_backlight = register_onboard_backlight;
	}

	par->fbtftops.reset(par);

	write_reg(par, 0x1b, 0x40); // init
	write_reg(par, 0x0c);				// clear
	write_reg(par, 0x1f, 0x4b, 0x70, 0x00, 0x01); // configure touch

	setup_touch_input(par);
	data = par->extra;


	INIT_WORK(&data->poll_work, thread_function);
	schedule_work(&data->poll_work);

	return 0;
}

static void set_addr_win(struct fbtft_par *par, int xs, int ys, int xe, int ye)
{}

static int write_vmem(struct fbtft_par *par, size_t offset, size_t len)
{
	struct gud_data *data = par->extra;
	u16 *vmem16 = (u16 *)(par->info->screen_buffer);
	u32 xres = par->info->var.xres;
	u32 yres = par->info->var.yres;
	u8 *buf = par->txbuf.buf;
	int x, y;
	int ret = 0;
	int destPos = 0;
  unsigned char b;

	if(data->lock) {
		return 0;
	}
	data->lock = true;

	write_reg(par, 0x1f, 0x28, 'd', 0x21);
	write_reg(par, 0, 0, 0, 0);
	write_reg(par, xres, xres >> 8, yres, yres >> 8);
	write_reg(par, 0x01);

  for (x = 0; x < xres; x++) {
     b = 0;
     for (y = 0; y < yres; y++) {
			b = (unsigned char)((b << 1) | (vmem16[xres * y + x] > 0));
       if (y % 8 == 7) {
         /* just did a full byte */
         buf[destPos++] = b;
       } /* if */
     } /* for */
     if (yres % 8 != 0) {
       b <<= (8 - yres % 8);
       buf[destPos++] = b;
     } /* if */
   } /* for */

	if(destPos != xres * yres / 8) {
		pr_alert("%s(WARN, e=%d, a=%d, len=%d)", __func__, xres * yres / 8, destPos, len);
	}

	ret = par->fbtftops.write(par, par->txbuf.buf, (xres * yres / 8));
	if (ret < 0)
		dev_err(par->info->device, "write failed and returned: %d\n",
			ret);

	data->lock = false;

	return ret;
}

static int set_var(struct fbtft_par *par)
{
	return 0;
}

static int blank(struct fbtft_par *par, bool on)
{
	fbtft_par_dbg(DEBUG_BLANK, par, "(%s=%s)\n",
		      __func__, on ? "true" : "false");

	write_reg(par, 0x1f, 0x28, 0x61, 0x40, on ? 0x00 : 0x01);
	return 0;
}

static unsigned long
request_gpios_match(struct fbtft_par *par, const struct fbtft_gpio *gpio)
{
	if (strcasecmp(gpio->name, "dc") == 0) {
		par->gpio.dc = gpio->gpio;
		return GPIOF_IN;
	}

	return FBTFT_GPIO_NO_MATCH;
}

#ifdef CONFIG_FB_BACKLIGHT
void unregister_backlight(struct fbtft_par *par)
{
	struct gud_data *data = par->extra;
	if (par->info->bl_dev) {
		par->info->bl_dev->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(par->info->bl_dev);
		backlight_device_unregister(par->info->bl_dev);
		par->info->bl_dev = NULL;
	}
	if(data != NULL) {
		cancel_work_sync(&data->poll_work);
		pr_alert("GUD DEINIT\n");
	} else {
		pr_alert("GUD DEINIT data=NULL\n");
	}
	
}

static int update_onboard_backlight(struct backlight_device *bd)
{
	struct fbtft_par *par = bl_get_data(bd);
	int level = bd->props.brightness;

	fbtft_par_dbg(DEBUG_BACKLIGHT, par, "%s: level=%d\n", __func__, level);

	if(level < 1 || level > 0x08) {
    return -1;
  }

  write_reg(par, 3, 0x1f, 'X', level);

	return 0;
}

static const struct backlight_ops bl_ops = {
	.update_status = update_onboard_backlight,
};

static void register_onboard_backlight(struct fbtft_par *par)
{
	struct backlight_device *bd;
	struct backlight_properties bl_props = { 
		.brightness = 8,
		.max_brightness = 8,
		.type = BACKLIGHT_RAW,
	};

	bl_props.type = BACKLIGHT_RAW;
	bl_props.power = FB_BLANK_POWERDOWN;

	bd = backlight_device_register(dev_driver_string(par->info->device),
				       par->info->device, par, &bl_ops,
				       &bl_props);
	if (IS_ERR(bd)) {
		dev_err(par->info->device,
			"cannot register backlight device (%ld)\n",
			PTR_ERR(bd));
		return;
	}
	par->info->bl_dev = bd;

	if (!par->fbtftops.unregister_backlight)
		par->fbtftops.unregister_backlight = unregister_backlight;
}
#else
static void register_onboard_backlight(struct fbtft_par *par) { };
#endif

static struct fbtft_display display = {
	.regwidth = 8,
	.width = WIDTH,
	.height = HEIGHT,
	.gamma_num = 1,
	.gamma_len = 1,
	.gamma = "00",
	.fbtftops = {
		.read = read,
		.write = write,
		.write_vmem = write_vmem,
		.init_display = init_display,
		.set_addr_win = set_addr_win,
		.request_gpios_match = request_gpios_match,
		.blank = blank,
		.set_var = set_var,
	},
};

FBTFT_REGISTER_DRIVER(DRVNAME, "noritake,gud", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("spi:gud");
MODULE_ALIAS("platform:gud");

MODULE_DESCRIPTION("Noritake graphic VFD Driver");
MODULE_AUTHOR("Vasily Kiniv");
MODULE_LICENSE("GPL");
