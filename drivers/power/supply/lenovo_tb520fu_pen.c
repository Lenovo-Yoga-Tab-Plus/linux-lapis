// SPDX-License-Identifier: GPL-2.0-only
/*
 * Lenovo Yoga Tab Plus (TB520FU) pen dock.
 *
 * The pen charges wirelessly while it sits on the tablet edge. The ADSP
 * charger firmware owns the Qi transmitter and talks to the pen; the kernel
 * tells it when a Hall switch sees the pen, raises the transmitter enable
 * line, and reads the pen's level, charging state and Bluetooth address back
 * through the vendor "HQ" property opcodes of the battery manager service.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/completion.h>
#include <linux/etherdevice.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/spinlock.h>
#include <linux/string_choices.h>
#include <linux/workqueue.h>

#define PEN_NAME		"tb520fu-pen"
#define PEN_ACK_MS		1000
#define PEN_SETTLE_MS		1500
#define PEN_SEARCH_MS		2000
#define PEN_QUIET_MS		30000
#define PEN_POLL_MS		60000
#define PEN_MISSES		3

#define BATTMGR_NOTIFICATION	0x07
#define HQ_PROPERTY_GET		0x60
#define HQ_PROPERTY_SET		0x61
#define HQ_NOTIFY_PEN		0x65

#define HQ_PEN_CHARGE_STATE	0x2b
#define HQ_PEN_LEVEL		0x2c
#define HQ_PEN_TX_ENABLE	0x2d
#define HQ_PEN_TX_IIN		0x2f
#define HQ_PEN_MAC_LOW		0x30
#define HQ_PEN_MAC_HIGH		0x31

#define PEN_HALL_1		BIT(0)
#define PEN_HALL_2		BIT(1)

enum pen_charge_state {
	PEN_NOT_CHARGING,
	PEN_CHARGING,
	PEN_FULL,
	PEN_HOT,
	PEN_COLD,
	PEN_BATTERY_LOW,
	PEN_UNKNOWN,
};

static const char * const pen_charge_names[] = {
	"not-charging", "charging", "full", "hot", "cold", "battery-low",
	"unknown",
};

struct pen_request {
	struct pmic_glink_hdr hdr;
	__le32 battery;
	__le32 property;
	__le32 value;
};

struct pen_response {
	struct pmic_glink_hdr hdr;
	__le32 property;
	__le32 value;
	__le32 result;
};

struct pen_notification {
	struct pmic_glink_hdr hdr;
	__le32 notification;
};

struct tb_pen {
	struct device *dev;
	struct pmic_glink_client *glink;
	struct power_supply *psy;
	struct gpio_desc *enable;

	struct mutex lock;		/* serialises requests and pen state */
	spinlock_t ack_lock;		/* protects the fields up to "halls" */
	struct completion ack;
	bool up, stopping, waiting;
	u32 wait_opcode, wait_property;
	u32 reply_value, reply_result;
	unsigned long halls;
	unsigned long quiet_until;

	struct work_struct dock_work;
	struct delayed_work refresh_work;

	bool docked, tx_on, present;
	u32 level, charge_state, tx_iin;
	u32 raw_level, raw_state;
	u8 mac[ETH_ALEN];
	char serial[18];
	u32 notifications, refreshes, misses;
	int last_error;
};

static struct platform_device *pen_device;
static struct tb_pen *pen_instance;
static int pen_probe_result = -ENODEV;

static bool charge = true;

static void pen_kick(struct tb_pen *pen)
{
	unsigned long flags;

	spin_lock_irqsave(&pen->ack_lock, flags);
	if (!pen->stopping)
		queue_work(system_dfl_wq, &pen->dock_work);
	spin_unlock_irqrestore(&pen->ack_lock, flags);
}

static int pen_charge_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_bool(val, kp);

	if (!ret && pen_instance)
		pen_kick(pen_instance);
	return ret;
}

static const struct kernel_param_ops pen_charge_ops = {
	.set = pen_charge_set,
	.get = param_get_bool,
};
module_param_cb(charge, &pen_charge_ops, &charge, 0644);
MODULE_PARM_DESC(charge, "Charge the pen while it is docked (default on)");

static void pen_callback(const void *data, size_t len, void *priv)
{
	const struct pen_notification *notification = data;
	const struct pen_response *resp = data;
	struct tb_pen *pen = priv;
	unsigned long flags;
	u32 opcode;

	if (len < sizeof(resp->hdr))
		return;
	opcode = le32_to_cpu(resp->hdr.opcode);

	spin_lock_irqsave(&pen->ack_lock, flags);
	if (opcode == BATTMGR_NOTIFICATION) {
		if (len == sizeof(*notification) && !pen->stopping &&
		    (le32_to_cpu(notification->notification) & 0xff) == HQ_NOTIFY_PEN) {
			unsigned long delay = 0;

			/* The firmware repeats this about once a second. */
			if (time_before(jiffies, pen->quiet_until))
				delay = pen->quiet_until - jiffies;
			pen->notifications++;
			mod_delayed_work(system_dfl_wq, &pen->refresh_work,
					 delay);
		}
	} else if ((opcode == HQ_PROPERTY_GET || opcode == HQ_PROPERTY_SET) &&
		   len == sizeof(*resp) && pen->waiting &&
		   pen->wait_opcode == opcode &&
		   pen->wait_property == le32_to_cpu(resp->property)) {
		pen->reply_value = le32_to_cpu(resp->value);
		pen->reply_result = le32_to_cpu(resp->result);
		pen->waiting = false;
		complete(&pen->ack);
	}
	spin_unlock_irqrestore(&pen->ack_lock, flags);
}

static void pen_transport(void *priv, int state)
{
	struct tb_pen *pen = priv;
	unsigned long flags;

	spin_lock_irqsave(&pen->ack_lock, flags);
	pen->up = state == SERVREG_SERVICE_STATE_UP;
	if (!pen->stopping)
		queue_work(system_dfl_wq, &pen->dock_work);
	spin_unlock_irqrestore(&pen->ack_lock, flags);
}

static int pen_request(struct tb_pen *pen, u32 opcode, u32 property, u32 value,
		       u32 *out)
{
	struct pen_request req = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(opcode),
		.property = cpu_to_le32(property),
		.value = cpu_to_le32(value),
	};
	unsigned long flags;
	int ret;

	lockdep_assert_held(&pen->lock);

	spin_lock_irqsave(&pen->ack_lock, flags);
	if (!pen->up) {
		spin_unlock_irqrestore(&pen->ack_lock, flags);
		return -ENOTCONN;
	}
	reinit_completion(&pen->ack);
	pen->wait_opcode = opcode;
	pen->wait_property = property;
	pen->waiting = true;
	spin_unlock_irqrestore(&pen->ack_lock, flags);

	ret = pmic_glink_send(pen->glink, &req, sizeof(req));
	if (!ret && !wait_for_completion_timeout(&pen->ack,
						 msecs_to_jiffies(PEN_ACK_MS)))
		ret = -ETIMEDOUT;

	spin_lock_irqsave(&pen->ack_lock, flags);
	pen->waiting = false;
	if (!ret && pen->reply_result)
		ret = -EIO;
	if (!ret && out)
		*out = pen->reply_value;
	spin_unlock_irqrestore(&pen->ack_lock, flags);

	return ret;
}

static void pen_forget(struct tb_pen *pen)
{
	pen->present = false;
	pen->level = 0;
	pen->charge_state = PEN_NOT_CHARGING;
	pen->tx_iin = 0;
	memset(pen->mac, 0, sizeof(pen->mac));
	pen->serial[0] = '\0';
}

static void pen_dock_work(struct work_struct *work)
{
	struct tb_pen *pen = container_of(work, struct tb_pen, dock_work);
	unsigned long flags;
	bool docked, up, want;
	int ret;

	spin_lock_irqsave(&pen->ack_lock, flags);
	docked = pen->halls;
	up = pen->up;
	spin_unlock_irqrestore(&pen->ack_lock, flags);

	mutex_lock(&pen->lock);
	pen->docked = docked;
	want = docked && up && charge;
	if (!up) {
		/* The charger firmware restarted and forgot the transmitter. */
		gpiod_set_value_cansleep(pen->enable, 0);
		pen->tx_on = false;
	} else if (want != pen->tx_on) {
		gpiod_set_value_cansleep(pen->enable, want);
		ret = pen_request(pen, HQ_PROPERTY_SET, HQ_PEN_TX_ENABLE, want,
				  NULL);
		if (ret) {
			dev_warn(pen->dev, "transmitter %s request failed: %d\n",
				 str_on_off(want), ret);
			pen->last_error = ret;
			gpiod_set_value_cansleep(pen->enable, 0);
			want = false;
		}
		pen->tx_on = want;
	}
	mutex_unlock(&pen->lock);

	spin_lock_irqsave(&pen->ack_lock, flags);
	if (!pen->stopping)
		mod_delayed_work(system_dfl_wq, &pen->refresh_work,
				 docked ? msecs_to_jiffies(PEN_SETTLE_MS) : 0);
	spin_unlock_irqrestore(&pen->ack_lock, flags);
}

static const struct power_supply_desc pen_psy_desc;

/*
 * The supply exists only while a pen is on the dock, so userspace sees it
 * appear with valid data and disappear when the pen is taken away. It carries
 * no serial number on purpose: UPower merges supplies whose serial equals a
 * Bluetooth address with the BlueZ battery of that device and then loses the
 * BlueZ one, which would hide the pen's battery level while it is undocked.
 */
static void pen_publish(struct tb_pen *pen, bool present, bool changed)
{
	struct power_supply_config cfg = { .drv_data = pen };
	struct power_supply *psy;

	if (present && !pen->psy) {
		psy = power_supply_register(pen->dev, &pen_psy_desc, &cfg);
		if (IS_ERR(psy)) {
			dev_warn(pen->dev, "power supply: %pe\n", psy);
			return;
		}
		pen->psy = psy;
	} else if (!present && pen->psy) {
		power_supply_unregister(pen->psy);
		pen->psy = NULL;
	} else if (pen->psy && changed) {
		power_supply_changed(pen->psy);
	}
}

static int pen_read(struct tb_pen *pen)
{
	u32 low, high, level, state, iin;
	u8 mac[ETH_ALEN];
	int ret;

	ret = pen_request(pen, HQ_PROPERTY_GET, HQ_PEN_LEVEL, 0, &level);
	if (ret)
		return ret;
	ret = pen_request(pen, HQ_PROPERTY_GET, HQ_PEN_CHARGE_STATE, 0, &state);
	if (ret)
		return ret;
	ret = pen_request(pen, HQ_PROPERTY_GET, HQ_PEN_TX_IIN, 0, &iin);
	if (ret)
		return ret;

	if (!pen->present) {
		ret = pen_request(pen, HQ_PROPERTY_GET, HQ_PEN_MAC_LOW, 0, &low);
		if (ret)
			return ret;
		ret = pen_request(pen, HQ_PROPERTY_GET, HQ_PEN_MAC_HIGH, 0,
				  &high);
		if (ret)
			return ret;

		/* Three address bytes per property. */
		mac[0] = high >> 16;
		mac[1] = high >> 8;
		mac[2] = high;
		mac[3] = low >> 16;
		mac[4] = low >> 8;
		mac[5] = low;
		if (is_zero_ether_addr(mac))
			return -ENODEV;
		ether_addr_copy(pen->mac, mac);
		snprintf(pen->serial, sizeof(pen->serial), "%pM", pen->mac);
		pen->present = true;
	}

	pen->raw_level = level;
	pen->raw_state = state;
	pen->level = min(level, 100U);
	pen->charge_state = state < PEN_UNKNOWN ? state : PEN_UNKNOWN;
	pen->tx_iin = iin;
	return 0;
}

static void pen_refresh_work(struct work_struct *work)
{
	struct tb_pen *pen = container_of(to_delayed_work(work), struct tb_pen,
					  refresh_work);
	bool docked, present, changed;
	u32 level, state;
	unsigned long flags, next;
	int ret;

	mutex_lock(&pen->lock);
	docked = pen->docked;
	level = pen->level;
	state = pen->charge_state;
	pen->refreshes++;
	if (!docked) {
		pen->misses = 0;
		pen_forget(pen);
	} else {
		ret = pen_read(pen);
		if (!ret) {
			pen->misses = 0;
		} else {
			if (ret != -ENODEV)
				pen->last_error = ret;
			if (++pen->misses >= PEN_MISSES)
				pen_forget(pen);
		}
	}
	present = pen->present;
	changed = level != pen->level || state != pen->charge_state;
	mutex_unlock(&pen->lock);

	pen_publish(pen, present, changed);

	next = msecs_to_jiffies(present ? PEN_POLL_MS : PEN_SEARCH_MS);
	spin_lock_irqsave(&pen->ack_lock, flags);
	pen->quiet_until = jiffies +
		msecs_to_jiffies(present ? PEN_QUIET_MS : PEN_SEARCH_MS / 4);
	if (docked && !pen->stopping)
		mod_delayed_work(system_dfl_wq, &pen->refresh_work, next);
	spin_unlock_irqrestore(&pen->ack_lock, flags);
}

static enum power_supply_property pen_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static int pen_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct tb_pen *pen = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&pen->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = pen->present;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (!pen->present)
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		else if (pen->charge_state == PEN_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (pen->charge_state == PEN_FULL)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else if (pen->charge_state == PEN_UNKNOWN)
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (pen->present)
			val->intval = pen->level;
		else
			ret = -ENODATA;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (!pen->present)
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		else if (pen->charge_state == PEN_HOT)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (pen->charge_state == PEN_COLD)
			val->intval = POWER_SUPPLY_HEALTH_COLD;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "Tab Pen Pro";
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Lenovo";
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&pen->lock);
	return ret;
}

static const struct power_supply_desc pen_psy_desc = {
	.name = PEN_NAME,
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = pen_props,
	.num_properties = ARRAY_SIZE(pen_props),
	.get_property = pen_get_property,
};

static ssize_t status_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct tb_pen *pen = dev_get_drvdata(dev);
	unsigned long flags, halls;
	ssize_t count;
	bool up;

	spin_lock_irqsave(&pen->ack_lock, flags);
	halls = pen->halls;
	up = pen->up;
	spin_unlock_irqrestore(&pen->ack_lock, flags);

	mutex_lock(&pen->lock);
	count = sysfs_emit(buf,
		"transport_up=%u hall1=%u hall2=%u docked=%u charge=%u transmitter=%u enable_gpio=%d\n"
		"present=%u level=%u charge_state=%s tx_iin=%u address=%s\n"
		"raw_level=%u raw_state=%u notifications=%u refreshes=%u last_error=%d\n",
		up, !!(halls & PEN_HALL_1), !!(halls & PEN_HALL_2), pen->docked,
		charge, pen->tx_on, gpiod_get_value_cansleep(pen->enable),
		pen->present, pen->level, pen_charge_names[pen->charge_state],
		pen->tx_iin, pen->serial[0] ? pen->serial : "none",
		pen->raw_level, pen->raw_state, pen->notifications,
		pen->refreshes, pen->last_error);
	mutex_unlock(&pen->lock);
	return count;
}
static DEVICE_ATTR_RO(status);

static struct attribute *pen_attrs[] = {
	&dev_attr_status.attr,
	NULL,
};
ATTRIBUTE_GROUPS(pen);

/* The Hall switches belong to gpio-keys; follow their switch events. */
static void pen_hall_update(struct tb_pen *pen, unsigned int code, int value)
{
	unsigned long bit, flags;

	if (code == SW_PEN_INSERTED)
		bit = PEN_HALL_1;
	else if (code == SW_LID)
		bit = PEN_HALL_2;
	else
		return;

	spin_lock_irqsave(&pen->ack_lock, flags);
	if (value)
		pen->halls |= bit;
	else
		pen->halls &= ~bit;
	if (!pen->stopping)
		queue_work(system_dfl_wq, &pen->dock_work);
	spin_unlock_irqrestore(&pen->ack_lock, flags);
}

static void pen_input_event(struct input_handle *handle, unsigned int type,
			    unsigned int code, int value)
{
	if (type == EV_SW)
		pen_hall_update(handle->handler->private, code, value);
}

static bool pen_input_match(struct input_handler *handler,
			    struct input_dev *dev)
{
	return dev->name && !strcmp(dev->name, "gpio-keys");
}

static int pen_input_connect(struct input_handler *handler,
			     struct input_dev *dev,
			     const struct input_device_id *id)
{
	struct tb_pen *pen = handler->private;
	struct input_handle *handle;
	bool hall1, hall2;
	int ret;

	handle = kzalloc_obj(*handle);
	if (!handle)
		return -ENOMEM;
	handle->dev = dev;
	handle->handler = handler;
	handle->name = PEN_NAME;

	ret = input_register_handle(handle);
	if (ret)
		goto free_handle;
	ret = input_open_device(handle);
	if (ret)
		goto unregister_handle;

	spin_lock_irq(&dev->event_lock);
	hall1 = test_bit(SW_PEN_INSERTED, dev->sw);
	hall2 = test_bit(SW_LID, dev->swbit) && test_bit(SW_LID, dev->sw);
	spin_unlock_irq(&dev->event_lock);

	pen_hall_update(pen, SW_PEN_INSERTED, hall1);
	pen_hall_update(pen, SW_LID, hall2);
	return 0;

unregister_handle:
	input_unregister_handle(handle);
free_handle:
	kfree(handle);
	return ret;
}

static void pen_input_disconnect(struct input_handle *handle)
{
	struct tb_pen *pen = handle->handler->private;

	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);

	pen_hall_update(pen, SW_PEN_INSERTED, 0);
	pen_hall_update(pen, SW_LID, 0);
}

static const struct input_device_id pen_input_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_SWBIT,
		.evbit = { BIT_MASK(EV_SW) },
		.swbit = { [BIT_WORD(SW_PEN_INSERTED)] =
				BIT_MASK(SW_PEN_INSERTED) },
	},
	{ }
};

static struct input_handler pen_input_handler = {
	.event = pen_input_event,
	.match = pen_input_match,
	.connect = pen_input_connect,
	.disconnect = pen_input_disconnect,
	.name = PEN_NAME,
	.id_table = pen_input_ids,
};

static struct gpiod_lookup_table pen_gpios = {
	.dev_id = PEN_NAME,
	.table = {
		GPIO_LOOKUP("f100000.pinctrl", 178, "enable", GPIO_ACTIVE_HIGH),
		{ }
	},
};

static int pen_setup(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tb_pen *pen;
	int ret;

	if (!dev->parent || !device_is_bound(dev->parent) ||
	    !of_device_is_compatible(dev->parent->of_node, "qcom,pmic-glink"))
		return -ENODEV;

	pen = devm_kzalloc(dev, sizeof(*pen), GFP_KERNEL);
	if (!pen)
		return -ENOMEM;
	pen->dev = dev;
	mutex_init(&pen->lock);
	spin_lock_init(&pen->ack_lock);
	init_completion(&pen->ack);
	INIT_WORK(&pen->dock_work, pen_dock_work);
	INIT_DELAYED_WORK(&pen->refresh_work, pen_refresh_work);
	platform_set_drvdata(pdev, pen);

	pen->enable = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(pen->enable))
		return dev_err_probe(dev, PTR_ERR(pen->enable), "enable GPIO\n");

	pen->glink = devm_pmic_glink_client_alloc(dev, PMIC_GLINK_OWNER_BATTMGR,
						  pen_callback, pen_transport,
						  pen);
	if (IS_ERR(pen->glink))
		return PTR_ERR(pen->glink);
	pmic_glink_client_register(pen->glink);

	pen_input_handler.private = pen;
	ret = input_register_handler(&pen_input_handler);
	if (ret)
		return dev_err_probe(dev, ret, "Hall switch handler\n");

	pen_instance = pen;
	dev_info(dev, "pen dock ready\n");
	return 0;
}

static int pen_probe(struct platform_device *pdev)
{
	pen_probe_result = pen_setup(pdev);
	return pen_probe_result;
}

static void pen_stop(struct platform_device *pdev)
{
	struct tb_pen *pen = platform_get_drvdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&pen->ack_lock, flags);
	pen->stopping = true;
	spin_unlock_irqrestore(&pen->ack_lock, flags);
	cancel_work_sync(&pen->dock_work);
	cancel_delayed_work_sync(&pen->refresh_work);

	mutex_lock(&pen->lock);
	if (pen->tx_on) {
		gpiod_set_value_cansleep(pen->enable, 0);
		pen_request(pen, HQ_PROPERTY_SET, HQ_PEN_TX_ENABLE, 0, NULL);
		pen->tx_on = false;
	}
	pen_forget(pen);
	mutex_unlock(&pen->lock);
	pen_publish(pen, false, false);
}

static void pen_remove(struct platform_device *pdev)
{
	pen_instance = NULL;
	input_unregister_handler(&pen_input_handler);
	pen_stop(pdev);
}

static struct platform_driver pen_driver = {
	.probe = pen_probe,
	.remove = pen_remove,
	.shutdown = pen_stop,
	.prevent_deferred_probe = true,
	.driver = {
		.name = PEN_NAME,
		.probe_type = PROBE_FORCE_SYNCHRONOUS,
		.suppress_bind_attrs = true,
		.dev_groups = pen_groups,
	},
};

static int __init pen_init(void)
{
	struct platform_device *parent;
	struct device_node *np;
	int ret = -ENODEV;

	if (!of_machine_is_compatible("lenovo,tb520fu"))
		return ret;
	np = of_find_node_by_path("/pmic-glink");
	if (!np)
		return ret;
	parent = of_find_device_by_node(np);
	if (!parent)
		goto put_node;
	if (!device_trylock(&parent->dev)) {
		ret = -EBUSY;
		goto put_parent;
	}
	if (!device_is_bound(&parent->dev) ||
	    !of_device_is_compatible(parent->dev.of_node, "qcom,pmic-glink"))
		goto unlock_parent;
	pen_device = platform_device_alloc(PEN_NAME, PLATFORM_DEVID_NONE);
	if (!pen_device) {
		ret = -ENOMEM;
		goto unlock_parent;
	}
	pen_device->dev.parent = &parent->dev;
	ret = platform_device_add(pen_device);
	if (ret) {
		platform_device_put(pen_device);
		pen_device = NULL;
		goto unlock_parent;
	}
	if (!device_link_add(&pen_device->dev, &parent->dev,
			     DL_FLAG_AUTOPROBE_CONSUMER))
		ret = -EINVAL;
unlock_parent:
	device_unlock(&parent->dev);
put_parent:
	put_device(&parent->dev);
put_node:
	of_node_put(np);
	if (ret)
		goto unregister_device;

	gpiod_add_lookup_table(&pen_gpios);
	ret = platform_driver_register(&pen_driver);
	if (!ret && pen_probe_result) {
		ret = pen_probe_result;
		platform_driver_unregister(&pen_driver);
	}
	if (ret) {
		gpiod_remove_lookup_table(&pen_gpios);
		goto unregister_device;
	}
	return 0;

unregister_device:
	if (pen_device)
		platform_device_unregister(pen_device);
	return ret;
}
module_init(pen_init);

static void __exit pen_exit(void)
{
	platform_driver_unregister(&pen_driver);
	platform_device_unregister(pen_device);
	gpiod_remove_lookup_table(&pen_gpios);
}
module_exit(pen_exit);

MODULE_DESCRIPTION("Lenovo Yoga Tab Plus (TB520FU) pen dock charger");
MODULE_LICENSE("GPL");
