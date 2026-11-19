// SPDX-License-Identifier: GPL-2.0-only
/*
 * Lenovo Yoga Tab Plus (TB520FU) pogo keyboard and touchpad
 *
 * The keyboard cover talks Lenovo's "one-wire bus" protocol over a
 * half-duplex UART at 921600 baud.  Every frame is
 *
 *	7e 7e ff <seed> <scrambled payload> <sum8> 7e
 *
 * with 7e/7d escaped as 7d 5e/7d 5d.  The payload is XORed with a fixed
 * 64-byte key starting at offset <seed>; its first byte is the event or
 * command type.  The tablet only drives the shared line while tx-en is high,
 * and it hears its own transmissions echoed back.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/random.h>
#include <linux/serdev.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define TBKB_BAUD			921600

#define TBKB_FLAG			0x7e
#define TBKB_FLAG2			0xff
#define TBKB_ESC			0x7d
#define TBKB_FLAG_ESC			0x5e
#define TBKB_ESC_ESC			0x5d
#define TBKB_MAX_FRAME			256

#define TBKB_EVT_KEYPRESS		0x11
#define TBKB_EVT_MMKEY			0x12
#define TBKB_EVT_TOUCH			0x13
#define TBKB_EVT_SYNC			0x16
#define TBKB_CMD_SET_PARAM		0x6a

#define TBKB_PARAM_CAPSLOCK_LED		0x27
#define TBKB_PARAM_SOFT_POWER		0x36

#define TBKB_TP_X_MAX			3211
#define TBKB_TP_Y_MAX			1879
#define TBKB_TP_RES			30
#define TBKB_TP_SLOTS			4

static const u8 tbkb_key[64] = {
	0xb8, 0x02, 0xd9, 0x57, 0xc6, 0x48, 0xc2, 0x13,
	0xd7, 0xa6, 0x30, 0xbf, 0x40, 0xd3, 0x07, 0xce,
	0xc3, 0xab, 0xfa, 0x51, 0x62, 0x18, 0xbd, 0x84,
	0x7d, 0xca, 0xe0, 0xcf, 0xf2, 0xac, 0xbc, 0x12,
	0xf5, 0x14, 0xd0, 0x72, 0xdc, 0x1a, 0x50, 0x00,
	0x00, 0x32, 0xd1, 0x7b, 0x33, 0xb6, 0x10, 0xdd,
	0x6a, 0x21, 0xeb, 0x7e, 0x52, 0x5b, 0x3d, 0x9f,
	0x85, 0xcb, 0x1f, 0x66, 0x73, 0x96, 0xa1, 0x60,
};

/* HID keyboard usage to Linux keycode, as in usbkbd. */
static const unsigned char tbkb_hid_keycode[256] = {
	  0,  0,  0,  0, 30, 48, 46, 32, 18, 33, 34, 35, 23, 36, 37, 38,
	 50, 49, 24, 25, 16, 19, 31, 20, 22, 47, 17, 45, 21, 44,  2,  3,
	  4,  5,  6,  7,  8,  9, 10, 11, 28,  1, 14, 15, 57, 12, 13, 26,
	 27, 43, 43, 39, 40, 41, 51, 52, 53, 58, 59, 60, 61, 62, 63, 64,
	 65, 66, 67, 68, 87, 88, 99, 70,119,110,102,104,111,107,109,106,
	105,108,103, 69, 98, 55, 74, 78, 96, 79, 80, 81, 75, 76, 77, 71,
	 72, 73, 82, 83, 86,127,116,117,183,184,185,186,187,188,189,190,
	191,192,193,194,134,138,130,132,128,129,131,137,133,135,136,113,
	115,114,  0,  0,  0,121,  0, 89, 93,124, 92, 94, 95,  0,  0,  0,
	122,123, 90, 91, 85,  0,  0,  0,  0,  0,  0,  0,111,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,179,180,  0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,111,  0,  0,  0,  0,  0,  0,  0,
	 29, 42, 56,125, 97, 54,100,126,164,166,165,163,161,115,114,113,
	150,158,159,128,136,177,178,176,142,152,173,140,  0,  0,  0,  0,
};

/* HID consumer usages sent in TBKB_EVT_MMKEY reports. */
static const struct {
	u16 usage;
	u16 keycode;
} tbkb_media_keys[] = {
	{ 0x00e2, KEY_MUTE },
	{ 0x00ea, KEY_VOLUMEDOWN },
	{ 0x00e9, KEY_VOLUMEUP },
	{ 0x0224, KEY_BACK },
	{ 0x0070, KEY_BRIGHTNESSDOWN },
	{ 0x006f, KEY_BRIGHTNESSUP },
	{ 0x038e, KEY_SCREENLOCK },
	{ 0x0390, KEY_LANGUAGE },
	{ 0x0391, KEY_MICMUTE },
	{ 0x0392, KEY_TOUCHPAD_TOGGLE },
	{ 0x0393, KEY_SEARCH },
	{ 0x0394, KEY_ZOOM },
	{ 0x0395, KEY_PROG3 },
	{ 0x0397, KEY_PROG4 },
	{ 0x0398, KEY_PROG1 },
	{ 0x0399, KEY_PROG2 },
};

enum tbkb_rx_state {
	TBKB_RX_FLAG1,
	TBKB_RX_FLAG2,
	TBKB_RX_BODY,
};

struct tbkb {
	struct serdev_device *serdev;
	struct device *dev;
	struct gpio_desc *power_gpio;
	struct gpio_desc *tx_en_gpio;
	struct gpio_desc *plug_gpio;
	int plug_irq;

	/* Protects the input devices and connection state. */
	struct mutex lock;
	/* Serializes tx-en and frame writes on the shared line. */
	struct mutex tx_lock;
	struct input_dev *keyboard;
	struct input_dev *touchpad;
	bool connected;
	bool capslock;

	struct work_struct connect_work;
	struct work_struct led_work;
	struct delayed_work plug_work;

	enum tbkb_rx_state rx_state;
	u8 rx_buf[TBKB_MAX_FRAME];
	size_t rx_len;

	u8 keys_old[8];
	u16 media_old[2];
};

static void tbkb_scramble(u8 *dst, const u8 *src, size_t len, u8 seed)
{
	size_t i;

	for (i = 0; i < len; i++)
		dst[i] = src[i] ^ tbkb_key[(seed + i) % ARRAY_SIZE(tbkb_key)];
}

static size_t tbkb_put_escaped(u8 *out, u8 c)
{
	if (c == TBKB_FLAG || c == TBKB_ESC) {
		out[0] = TBKB_ESC;
		out[1] = c == TBKB_FLAG ? TBKB_FLAG_ESC : TBKB_ESC_ESC;
		return 2;
	}
	out[0] = c;
	return 1;
}

static int tbkb_send(struct tbkb *kb, const u8 *payload, size_t len)
{
	u8 body[64], frame[2 * sizeof(body) + 8];
	size_t n = 0, i, step;
	u8 seed, sum = 0;
	int ret;

	if (len + 1 > sizeof(body))
		return -EINVAL;

	seed = get_random_u8() & 0x7f;
	body[0] = seed;
	tbkb_scramble(body + 1, payload, len, seed);

	frame[n++] = TBKB_FLAG;
	frame[n++] = TBKB_FLAG;
	frame[n++] = TBKB_FLAG2;
	for (i = 0; i < len + 1; i++) {
		step = tbkb_put_escaped(frame + n, body[i]);
		for (; step; step--)
			sum += frame[n++];
	}
	n += tbkb_put_escaped(frame + n, sum);
	frame[n++] = TBKB_FLAG;

	mutex_lock(&kb->tx_lock);
	gpiod_set_value_cansleep(kb->tx_en_gpio, 1);
	usleep_range(450, 500);
	ret = serdev_device_write(kb->serdev, frame, n, msecs_to_jiffies(100));
	serdev_device_wait_until_sent(kb->serdev, msecs_to_jiffies(100));
	gpiod_set_value_cansleep(kb->tx_en_gpio, 0);
	mutex_unlock(&kb->tx_lock);

	return ret < 0 ? ret : 0;
}

static int tbkb_set_param(struct tbkb *kb, u8 param, u8 value)
{
	const u8 cmd[] = { TBKB_CMD_SET_PARAM, 0x03, param, 0x01, value };

	return tbkb_send(kb, cmd, sizeof(cmd));
}

static void tbkb_led_work(struct work_struct *work)
{
	struct tbkb *kb = container_of(work, struct tbkb, led_work);

	if (kb->connected)
		tbkb_set_param(kb, TBKB_PARAM_CAPSLOCK_LED, kb->capslock);
}

static int tbkb_input_event(struct input_dev *input, unsigned int type,
			    unsigned int code, int value)
{
	struct tbkb *kb = input_get_drvdata(input);

	if (type != EV_LED || code != LED_CAPSL)
		return -EINVAL;

	kb->capslock = !!value;
	schedule_work(&kb->led_work);
	return 0;
}

static int tbkb_create_inputs(struct tbkb *kb)
{
	struct input_dev *kbd, *tp;
	int i, ret;

	kbd = input_allocate_device();
	tp = input_allocate_device();
	if (!kbd || !tp) {
		ret = -ENOMEM;
		goto err_free;
	}

	kbd->name = "Lenovo TB520FU Keyboard";
	kbd->phys = "serial0/input0";
	kbd->id.bustype = BUS_RS232;
	kbd->id.vendor = 0x17ef;
	kbd->id.product = 0x61ba;
	kbd->dev.parent = kb->dev;
	kbd->event = tbkb_input_event;
	input_set_drvdata(kbd, kb);

	__set_bit(EV_KEY, kbd->evbit);
	__set_bit(EV_LED, kbd->evbit);
	__set_bit(EV_REP, kbd->evbit);
	__set_bit(LED_CAPSL, kbd->ledbit);
	for (i = 0; i < ARRAY_SIZE(tbkb_hid_keycode); i++)
		if (tbkb_hid_keycode[i])
			__set_bit(tbkb_hid_keycode[i], kbd->keybit);
	for (i = 0; i < ARRAY_SIZE(tbkb_media_keys); i++)
		__set_bit(tbkb_media_keys[i].keycode, kbd->keybit);

	tp->name = "Lenovo TB520FU Touchpad";
	tp->phys = "serial0/input1";
	tp->id.bustype = BUS_RS232;
	tp->id.vendor = 0x17ef;
	tp->id.product = 0x61ba;
	tp->dev.parent = kb->dev;

	input_set_capability(tp, EV_KEY, BTN_LEFT);
	input_set_capability(tp, EV_KEY, BTN_RIGHT);
	input_set_abs_params(tp, ABS_MT_POSITION_X, 0, TBKB_TP_X_MAX, 0, 0);
	input_set_abs_params(tp, ABS_MT_POSITION_Y, 0, TBKB_TP_Y_MAX, 0, 0);
	input_abs_set_res(tp, ABS_MT_POSITION_X, TBKB_TP_RES);
	input_abs_set_res(tp, ABS_MT_POSITION_Y, TBKB_TP_RES);
	__set_bit(INPUT_PROP_BUTTONPAD, tp->propbit);
	ret = input_mt_init_slots(tp, TBKB_TP_SLOTS, INPUT_MT_POINTER);
	if (ret)
		goto err_free;

	ret = input_register_device(kbd);
	if (ret)
		goto err_free;
	ret = input_register_device(tp);
	if (ret) {
		input_unregister_device(kbd);
		input_free_device(tp);
		return ret;
	}

	kb->keyboard = kbd;
	kb->touchpad = tp;
	return 0;

err_free:
	input_free_device(kbd);
	input_free_device(tp);
	return ret;
}

static void tbkb_disconnect_locked(struct tbkb *kb)
{
	if (!kb->connected)
		return;

	kb->connected = false;
	input_unregister_device(kb->touchpad);
	input_unregister_device(kb->keyboard);
	kb->touchpad = NULL;
	kb->keyboard = NULL;
	memset(kb->keys_old, 0, sizeof(kb->keys_old));
	memset(kb->media_old, 0, sizeof(kb->media_old));
	dev_info(kb->dev, "keyboard disconnected\n");
}

static void tbkb_connect_work(struct work_struct *work)
{
	struct tbkb *kb = container_of(work, struct tbkb, connect_work);
	int ret;

	mutex_lock(&kb->lock);
	if (!kb->connected) {
		ret = tbkb_create_inputs(kb);
		if (ret) {
			dev_err(kb->dev, "failed to register input devices: %d\n", ret);
		} else {
			kb->connected = true;
			dev_info(kb->dev, "keyboard connected\n");
		}
	}
	mutex_unlock(&kb->lock);

	if (kb->connected) {
		tbkb_set_param(kb, TBKB_PARAM_SOFT_POWER, 0);
		tbkb_set_param(kb, TBKB_PARAM_CAPSLOCK_LED, kb->capslock);
	}
}

static void tbkb_report_keys(struct tbkb *kb, const u8 *rep, size_t len)
{
	struct input_dev *input = kb->keyboard;
	int i;

	if (len < 8)
		return;

	for (i = 0; i < 8; i++)
		input_report_key(input, tbkb_hid_keycode[224 + i], (rep[1] >> i) & 1);

	for (i = 2; i < 8; i++) {
		if (kb->keys_old[i] > 3 && !memchr(rep + 2, kb->keys_old[i], 6) &&
		    tbkb_hid_keycode[kb->keys_old[i]])
			input_report_key(input, tbkb_hid_keycode[kb->keys_old[i]], 0);

		if (rep[i] > 3 && !memchr(kb->keys_old + 2, rep[i], 6) &&
		    tbkb_hid_keycode[rep[i]])
			input_report_key(input, tbkb_hid_keycode[rep[i]], 1);
	}

	input_sync(input);
	memcpy(kb->keys_old, rep, 8);
}

static unsigned int tbkb_media_keycode(u16 usage)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tbkb_media_keys); i++)
		if (tbkb_media_keys[i].usage == usage)
			return tbkb_media_keys[i].keycode;
	return KEY_RESERVED;
}

static void tbkb_report_media(struct tbkb *kb, const u8 *rep, size_t len)
{
	u16 now[2];
	int i, j;

	if (len < 5)
		return;

	now[0] = get_unaligned_le16(rep + 1);
	now[1] = get_unaligned_le16(rep + 3);

	for (i = 0; i < 2; i++) {
		unsigned int code;
		bool held_old = false, held_new = false;

		for (j = 0; j < 2; j++) {
			held_old |= kb->media_old[i] && kb->media_old[i] == now[j];
			held_new |= now[i] && now[i] == kb->media_old[j];
		}

		code = tbkb_media_keycode(kb->media_old[i]);
		if (kb->media_old[i] && !held_old && code != KEY_RESERVED)
			input_report_key(kb->keyboard, code, 0);

		code = tbkb_media_keycode(now[i]);
		if (now[i] && !held_new && code != KEY_RESERVED)
			input_report_key(kb->keyboard, code, 1);
	}

	input_sync(kb->keyboard);
	memcpy(kb->media_old, now, sizeof(now));
}

/* rep: len, buttons, then 5 bytes per finger: id<<4 | down, x16, y16. */
static void tbkb_report_touch(struct tbkb *kb, const u8 *rep, size_t len)
{
	struct input_dev *input = kb->touchpad;
	int fingers, i;

	if (len < 2 || rep[0] < 1)
		return;

	fingers = (rep[0] - 1) / 5;
	if (len < 2 + fingers * 5)
		return;

	for (i = 0; i < fingers; i++) {
		const u8 *f = rep + 2 + 5 * i;
		int slot = f[0] >> 4;
		bool down = f[0] & 0x01;

		if (slot >= TBKB_TP_SLOTS)
			continue;

		input_mt_slot(input, slot);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, down);
		if (down) {
			input_report_abs(input, ABS_MT_POSITION_X, get_unaligned_le16(f + 1));
			input_report_abs(input, ABS_MT_POSITION_Y, get_unaligned_le16(f + 3));
		}
	}

	input_mt_sync_frame(input);
	input_report_key(input, BTN_LEFT, rep[1] & BIT(0));
	input_report_key(input, BTN_RIGHT, rep[1] & BIT(1));
	input_sync(input);
}

static void tbkb_handle_packet(struct tbkb *kb, const u8 *pkt, size_t len)
{
	if (!len)
		return;

	/* The cover stops syncing while idle, so only the plug sensor disconnects it. */
	if (pkt[0] == TBKB_EVT_SYNC) {
		if (!kb->connected)
			schedule_work(&kb->connect_work);
		return;
	}

	mutex_lock(&kb->lock);
	if (kb->connected) {
		switch (pkt[0]) {
		case TBKB_EVT_KEYPRESS:
			tbkb_report_keys(kb, pkt + 1, len - 1);
			break;
		case TBKB_EVT_MMKEY:
			tbkb_report_media(kb, pkt + 1, len - 1);
			break;
		case TBKB_EVT_TOUCH:
			tbkb_report_touch(kb, pkt + 1, len - 1);
			break;
		default:
			/* Echoes of our own commands and replies to them. */
			break;
		}
	}
	mutex_unlock(&kb->lock);
}

/* rx_buf holds "ff <escaped body> <sum> 7e"; returns the decoded length or -EINVAL. */
static int tbkb_decode(struct tbkb *kb, u8 *out)
{
	const u8 *p = kb->rx_buf + 1;
	size_t enc_len = kb->rx_len - 2;
	size_t i = 0, n = 0, body_end;
	u8 sum = 0, crc;

	if (kb->rx_len < 5)
		return -EINVAL;

	/* The checksum byte may itself be escaped. */
	if (enc_len >= 2 && p[enc_len - 2] == TBKB_ESC) {
		crc = p[enc_len - 1] == TBKB_FLAG_ESC ? TBKB_FLAG : TBKB_ESC;
		body_end = enc_len - 2;
	} else {
		crc = p[enc_len - 1];
		body_end = enc_len - 1;
	}

	while (i < body_end) {
		sum += p[i];
		if (p[i] == TBKB_ESC) {
			if (i + 1 >= body_end)
				return -EINVAL;
			sum += p[i + 1];
			if (p[i + 1] == TBKB_FLAG_ESC)
				out[n++] = TBKB_FLAG;
			else if (p[i + 1] == TBKB_ESC_ESC)
				out[n++] = TBKB_ESC;
			else
				return -EINVAL;
			i += 2;
		} else {
			out[n++] = p[i++];
		}
	}

	return sum == crc && n >= 2 ? n : -EINVAL;
}

static size_t tbkb_receive(struct serdev_device *serdev, const u8 *data, size_t count)
{
	struct tbkb *kb = serdev_device_get_drvdata(serdev);
	u8 decoded[TBKB_MAX_FRAME], pkt[TBKB_MAX_FRAME];
	size_t i;
	int n;

	for (i = 0; i < count; i++) {
		u8 c = data[i];

		switch (kb->rx_state) {
		case TBKB_RX_FLAG1:
			if (c == TBKB_FLAG)
				kb->rx_state = TBKB_RX_FLAG2;
			break;
		case TBKB_RX_FLAG2:
			if (c == TBKB_FLAG2) {
				kb->rx_buf[0] = c;
				kb->rx_len = 1;
				kb->rx_state = TBKB_RX_BODY;
			} else if (c != TBKB_FLAG) {
				kb->rx_state = TBKB_RX_FLAG1;
			}
			break;
		case TBKB_RX_BODY:
			kb->rx_buf[kb->rx_len++] = c;
			if (c == TBKB_FLAG) {
				n = tbkb_decode(kb, decoded);
				if (n > 0) {
					tbkb_scramble(pkt, decoded + 1, n - 1, decoded[0]);
					tbkb_handle_packet(kb, pkt, n - 1);
				}
				/* A closing flag can also start the next frame. */
				kb->rx_state = TBKB_RX_FLAG2;
			} else if (kb->rx_len == sizeof(kb->rx_buf)) {
				kb->rx_state = TBKB_RX_FLAG1;
			}
			break;
		}
	}

	return count;
}

static const struct serdev_device_ops tbkb_serdev_ops = {
	.receive_buf = tbkb_receive,
	.write_wakeup = serdev_device_write_wakeup,
};

static void tbkb_plug_work(struct work_struct *work)
{
	struct tbkb *kb = container_of(to_delayed_work(work), struct tbkb, plug_work);
	bool present = gpiod_get_value_cansleep(kb->plug_gpio);

	gpiod_set_value_cansleep(kb->power_gpio, present);
	if (!present) {
		mutex_lock(&kb->lock);
		tbkb_disconnect_locked(kb);
		mutex_unlock(&kb->lock);
	}
	dev_dbg(kb->dev, "cover %s\n", present ? "attached" : "detached");
}

static irqreturn_t tbkb_plug_irq(int irq, void *data)
{
	struct tbkb *kb = data;

	mod_delayed_work(system_dfl_wq, &kb->plug_work, msecs_to_jiffies(50));
	return IRQ_HANDLED;
}

static void tbkb_cleanup(void *data)
{
	struct tbkb *kb = data;

	cancel_delayed_work_sync(&kb->plug_work);
	cancel_work_sync(&kb->connect_work);
	cancel_work_sync(&kb->led_work);
	mutex_lock(&kb->lock);
	tbkb_disconnect_locked(kb);
	mutex_unlock(&kb->lock);
	gpiod_set_value_cansleep(kb->power_gpio, 0);
}

static int tbkb_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct tbkb *kb;
	int ret;

	kb = devm_kzalloc(dev, sizeof(*kb), GFP_KERNEL);
	if (!kb)
		return -ENOMEM;

	kb->serdev = serdev;
	kb->dev = dev;
	mutex_init(&kb->lock);
	mutex_init(&kb->tx_lock);
	INIT_WORK(&kb->connect_work, tbkb_connect_work);
	INIT_WORK(&kb->led_work, tbkb_led_work);
	INIT_DELAYED_WORK(&kb->plug_work, tbkb_plug_work);
	serdev_device_set_drvdata(serdev, kb);

	kb->power_gpio = devm_gpiod_get(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(kb->power_gpio))
		return dev_err_probe(dev, PTR_ERR(kb->power_gpio), "power gpio\n");

	kb->tx_en_gpio = devm_gpiod_get(dev, "tx-en", GPIOD_OUT_LOW);
	if (IS_ERR(kb->tx_en_gpio))
		return dev_err_probe(dev, PTR_ERR(kb->tx_en_gpio), "tx-en gpio\n");

	kb->plug_gpio = devm_gpiod_get(dev, "plug", GPIOD_IN);
	if (IS_ERR(kb->plug_gpio))
		return dev_err_probe(dev, PTR_ERR(kb->plug_gpio), "plug gpio\n");

	serdev_device_set_client_ops(serdev, &tbkb_serdev_ops);
	ret = devm_serdev_device_open(dev, serdev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to open UART\n");

	serdev_device_set_baudrate(serdev, TBKB_BAUD);
	serdev_device_set_flow_control(serdev, false);
	ret = serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set parity\n");

	ret = devm_add_action_or_reset(dev, tbkb_cleanup, kb);
	if (ret)
		return ret;

	kb->plug_irq = gpiod_to_irq(kb->plug_gpio);
	if (kb->plug_irq < 0)
		return dev_err_probe(dev, kb->plug_irq, "plug irq\n");

	ret = devm_request_irq(dev, kb->plug_irq, tbkb_plug_irq,
			       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			       "tb520fu-pogo-plug", kb);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request plug irq\n");

	schedule_delayed_work(&kb->plug_work, 0);
	return 0;
}

static const struct of_device_id tbkb_of_match[] = {
	{ .compatible = "lenovo,tb520fu-pogo-keyboard" },
	{ }
};
MODULE_DEVICE_TABLE(of, tbkb_of_match);

static struct serdev_device_driver tbkb_driver = {
	.probe = tbkb_probe,
	.driver = {
		.name = "lenovo-tb520fu-pogo",
		.of_match_table = tbkb_of_match,
	},
};
module_serdev_device_driver(tbkb_driver);

MODULE_DESCRIPTION("Lenovo Yoga Tab Plus pogo keyboard and touchpad");
MODULE_LICENSE("GPL");
