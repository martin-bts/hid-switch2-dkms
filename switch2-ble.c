// SPDX-License-Identifier: GPL-2.0+
/*
 * HID driver for Nintendo Switch controllers — BLE transport
 *
 * Implements the BLE transport for controllers connected via the BlueZ
 * gatt-uhid bridge, which creates a uhid HID device and forwards all
 * GATT traffic bidirectionally.
 *
 * Report format (defined by gatt-uhid bridge):
 *   byte 0:    HID report ID (0x01)
 *   byte 1-2:  GATT handle, little-endian
 *   byte 3+:   payload (raw GATT notification or command frame)
 *
 * Input routing (by GATT handle):
 *   - Input characteristic (e.g. 0x000e): 63-byte controller reports.
 *     The payload is raw [seq][status][buttons...sticks...] with no
 *     report type byte.  We prepend the report type (determined from
 *     controller type) before passing to switch2_event().
 *   - ACK characteristic (e.g. 0x001a): command responses with 0x91
 *     framing.  Routed to switch2_receive_command().
 *   - Other handles: logged and ignored.
 *
 * Output path (send_command / send_rumble):
 *   Prepends [handle_lo][handle_hi] to the 0x91 frame and calls
 *   hid_hw_output_report().  The gatt-uhid bridge reads the handle
 *   and writes the frame to that GATT characteristic.
 *
 * The init sequence, calibration, and input parsing are all handled by
 * hid-switch2.c — this file only adapts the transport framing.
 */

#include "hid-switch2.h"
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/slab.h>

/* GATT handle prefix size in bridge reports */
#define GATT_HANDLE_SIZE	2

/* GATT handle for the command/write characteristic.
 * This is a property of the controller's GATT service layout. */
#define NS2_GATT_CMD_HANDLE	0x0014

/* GATT handle for the haptic data characteristic.
 * Raw 64-byte HD-rumble frames are written here — no 0x91 wrapping. */
#define NS2_GATT_HAPTIC_HANDLE	0x0012

/*
 * Extended per-BLE-connection state.  Wraps the upstream switch2_ble
 * (which contains cfg as its first member) with GATT handle routing.
 */
struct switch2_ble_ext {
	struct switch2_ble base;	/* upstream struct, must be first */
	uint16_t input_handle;		/* GATT handle for input notifications */
	uint16_t ack_handle;		/* GATT handle for ACK notifications */
	bool init_pending;		/* waiting for first raw_event to init */
	struct work_struct init_work;	/* deferred init work */
};

/* ------------------------------------------------------------------ */
/* Output: prepend GATT handle to outgoing frames                       */
/* ------------------------------------------------------------------ */

static int switch2_ble_send_cmd(enum switch2_cmd command, uint8_t subcommand,
	const void *msg, size_t len, struct switch2_cfg_intf *intf)
{
	struct switch2_controller *ns2 = intf->parent;
	struct switch2_cmd_header header = {
		command, NS2_DIR_OUT | NS2_FLAG_OK, NS2_TRANS_BT,
		subcommand, 0, len
	};
	uint8_t *frame;
	size_t frame_len;
	int ret;

	if (WARN_ON(len > 56))
		return -EINVAL;

	if (!ns2->hdev)
		return -ENODEV;

	frame_len = GATT_HANDLE_SIZE + sizeof(header) + len;
	frame = kzalloc(frame_len, GFP_KERNEL);
	if (!frame)
		return -ENOMEM;

	/* Little-endian GATT handle prefix */
	frame[0] = NS2_GATT_CMD_HANDLE & 0xff;
	frame[1] = (NS2_GATT_CMD_HANDLE >> 8) & 0xff;

	memcpy(frame + GATT_HANDLE_SIZE, &header, sizeof(header));
	if (msg && len)
		memcpy(frame + GATT_HANDLE_SIZE + sizeof(header), msg, len);

	ret = hid_hw_output_report(ns2->hdev, frame, frame_len);
	hid_dbg(ns2->hdev, "send_cmd: cmd=0x%02x sub=0x%02x len=%zu ret=%d\n",
		command, subcommand, frame_len, ret);
	kfree(frame);
	return ret;
}

static int switch2_ble_send_rumble(const uint8_t *buf, size_t len,
	struct switch2_cfg_intf *intf)
{
	struct switch2_controller *ns2 = intf->parent;
	uint8_t frame[GATT_HANDLE_SIZE + 64];

	if (!ns2->hdev)
		return -ENODEV;

	if (len < 64)
		return -EINVAL;

	/* GC may use ERM — not implemented for BLE yet */
	if (buf[0] == 3)
		return 0;

	/* Little-endian GATT handle prefix — haptic data goes to 0x0012 */
	frame[0] = NS2_GATT_HAPTIC_HANDLE & 0xff;
	frame[1] = (NS2_GATT_HAPTIC_HANDLE >> 8) & 0xff;

	/* Send the raw 64-byte haptic buffer as-is (no 0x91 wrapping).
	 * The buffer already contains the 0x50|seq header, L/R channels,
	 * and HD-rumble encoding — exactly what handle 0x0012 expects. */
	memcpy(frame + GATT_HANDLE_SIZE, buf, 64);

	return hid_hw_output_report(ns2->hdev, frame, sizeof(frame));
}

/* ------------------------------------------------------------------ */
/* Deferred init: triggered by first raw_event                           */
/* ------------------------------------------------------------------ */

static void switch2_ble_init_work(struct work_struct *work)
{
	struct switch2_ble_ext *ns2_ble =
		container_of(work, struct switch2_ble_ext, init_work);
	struct switch2_controller *ns2 = ns2_ble->base.cfg.parent;
	int ret;

	mutex_lock(&ns2->lock);
	if (!ns2->hdev) {
		mutex_unlock(&ns2->lock);
		return;
	}
	ret = switch2_init_controller(ns2);
	if (ret < 0)
		hid_err(ns2->hdev, "deferred init failed: %d (step %d)\n",
			ret, ns2->init_step);
	mutex_unlock(&ns2->lock);
}

/* ------------------------------------------------------------------ */
/* Input: parse handle prefix, route to appropriate handler              */
/* ------------------------------------------------------------------ */

/*
 * raw_event callback.  hid-core passes the full report buffer including
 * the report ID byte at raw_data[0].  Our bridge format is:
 *   raw_data[0]    = report ID (0x01)
 *   raw_data[1..2] = GATT handle, little-endian
 *   raw_data[3+]   = payload
 */
static int switch2_ble_raw_event(struct hid_device *hdev,
	struct hid_report *report, uint8_t *raw_data, int size)
{
	struct switch2_controller *ns2 = hid_get_drvdata(hdev);
	struct switch2_ble_ext *ns2_ble = (struct switch2_ble_ext *)ns2->cfg;
	uint16_t handle;
	uint8_t *payload;
	int payload_len;
	int ret;

	if (report->type != HID_INPUT_REPORT)
		return 0;

	/* Skip report ID byte (raw_data[0]) */
	if (size < 1 + GATT_HANDLE_SIZE + 1)
		return -EINVAL;

	/*
	 * First raw_event means the BlueZ gatt-uhid bridge has its GLib
	 * I/O watch active and can process UHID_OUTPUT events.  Kick off
	 * init now — it's too early during probe (inside UHID_CREATE2).
	 */
	if (unlikely(ns2_ble->init_pending)) {
		ns2_ble->init_pending = false;
		schedule_work(&ns2_ble->init_work);
	}

	handle = raw_data[1] | ((uint16_t)raw_data[2] << 8);
	payload = raw_data + 1 + GATT_HANDLE_SIZE;
	payload_len = size - 1 - GATT_HANDLE_SIZE;

	if (handle == ns2_ble->ack_handle) {
		/* ACK / command response — route to init state machine */
		hid_dbg(hdev, "ACK: %d bytes, first=%02x\n",
			payload_len, payload_len > 0 ? payload[0] : 0);
		switch2_receive_command(ns2, payload, payload_len);
		return 0;
	}

	if (handle == ns2_ble->input_handle) {
		/*
		 * Input report.  The raw GATT notification has no report type
		 * byte — it starts with [seq][status][buttons...sticks...].
		 * switch2_event() expects raw_data[0] to be the report type
		 * (e.g. 0x09 for NS2_REPORT_PRO) with button/stick data at
		 * the offsets used by the USB path.
		 *
		 * Prepend the report type byte determined by controller type,
		 * and temporarily set report->id to match.
		 */
		uint8_t buf[64];
		unsigned int orig_id = report->id;
		uint8_t report_type;

		switch (ns2->ctlr_type) {
		case NS2_CTLR_TYPE_JCL:
			report_type = NS2_REPORT_JCL;
			break;
		case NS2_CTLR_TYPE_JCR:
			report_type = NS2_REPORT_JCR;
			break;
		case NS2_CTLR_TYPE_GC:
			report_type = NS2_REPORT_GC;
			break;
		default:
			report_type = NS2_REPORT_PRO;
			break;
		}

		if (payload_len > (int)sizeof(buf) - 1)
			payload_len = sizeof(buf) - 1;

		buf[0] = report_type;
		memcpy(&buf[1], payload, payload_len);

		report->id = report_type;
		ret = switch2_event(hdev, report, buf, payload_len + 1);
		report->id = orig_id;
		return ret;
	}

	/* Other handles — ignore */
	hid_dbg(hdev, "unhandled handle 0x%04x, %d bytes\n", handle,
		payload_len);

	return 0;
}

/* ------------------------------------------------------------------ */
/* Probe / remove                                                       */
/* ------------------------------------------------------------------ */

static int switch2_ble_probe(struct hid_device *hdev,
	const struct hid_device_id *id)
{
	struct switch2_controller *ns2;
	struct switch2_ble_ext *ns2_ble;
	char phys[64];
	int ret;

	hid_dbg(hdev, "probe start\n");

	snprintf(phys, sizeof(phys), "bt%s", hdev->uniq);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed %d\n", ret);
		return ret;
	}

	/*
	 * Do not expose a hidraw node for BLE controllers.  Steam/SDL would
	 * otherwise write USB-format 0x91 commands directly to hidraw,
	 * bypassing the BLE transport framing.
	 */
	ret = hid_hw_start(hdev, 0);
	if (ret) {
		hid_err(hdev, "hw_start failed %d\n", ret);
		return ret;
	}

	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "hw_open failed %d\n", ret);
		goto err_stop;
	}

	ns2 = switch2_get_controller(phys);
	if (IS_ERR(ns2)) {
		hid_err(hdev, "get_controller failed\n");
		ret = PTR_ERR(ns2);
		goto err_close;
	}

	ns2_ble = devm_kzalloc(&hdev->dev, sizeof(*ns2_ble), GFP_KERNEL);
	if (!ns2_ble) {
		ret = -ENOMEM;
		goto err_put;
	}

	mutex_lock(&ns2->lock);
	ns2->hdev = hdev;

	ns2->player_id = U32_MAX;
	ret = switch2_alloc_player_id();
	if (ret < 0)
		hid_warn(hdev, "Failed to allocate player ID; ret=%d\n", ret);
	else
		ns2->player_id = ret;

	ns2_ble->base.cfg.parent       = ns2;
	ns2_ble->base.cfg.send_command = switch2_ble_send_cmd;
	ns2_ble->base.cfg.send_rumble  = switch2_ble_send_rumble;

	/*
	 * GATT handles for routing incoming notifications.
	 * These are properties of the controller's GATT service layout.
	 * The bridge forwards all notifications with the handle prefix,
	 * so we can identify them here.
	 *
	 * TODO: these could be discovered from the bridge or passed via
	 * a feature report instead of being hardcoded.
	 */
	ns2_ble->input_handle = 0x000e;
	ns2_ble->ack_handle   = 0x001a;

	ns2->cfg       = &ns2_ble->base.cfg;

	/*
	 * Let the full init state machine run (same as USB path).
	 * init_step starts at 0 (NS2_INIT_NONE) from kzalloc.
	 * switch2_receive_command() calls switch2_init_controller()
	 * on each ACK, advancing through calibration reads etc.
	 *
	 * ctlr_type defaults to PRO; switch2_init_controller will
	 * update it from firmware info during init.
	 */
	ns2->ctlr_type = NS2_CTLR_TYPE_PRO;

#ifdef CONFIG_SWITCH2_FF
	switch2_init_rumble(ns2);
#endif

	/*
	 * Defer init until the first raw_event.  During probe we're inside
	 * BlueZ's UHID_CREATE2 write — the GLib I/O watch on the uhid fd
	 * isn't registered yet, so any UHID_OUTPUT we send now would be
	 * queued but never read.  The first raw_event proves the bridge's
	 * event loop is running and output will be delivered.
	 */
	INIT_WORK(&ns2_ble->init_work, switch2_ble_init_work);
	ns2_ble->init_pending = true;

	hid_set_drvdata(hdev, ns2);

	mutex_unlock(&ns2->lock);
	hid_dbg(hdev, "probe complete\n");
	return 0;

err_put:
	switch2_controller_put(ns2);
err_close:
	hid_hw_close(hdev);
err_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void switch2_ble_remove(struct hid_device *hdev)
{
	struct switch2_controller *ns2 = hid_get_drvdata(hdev);
	struct switch2_ble_ext *ns2_ble = (struct switch2_ble_ext *)ns2->cfg;
#ifdef CONFIG_SWITCH2_FF
	unsigned long flags;

	spin_lock_irqsave(&ns2->rumble_lock, flags);
	cancel_delayed_work_sync(&ns2->rumble_work);
	spin_unlock_irqrestore(&ns2->rumble_lock, flags);
#endif
	/* Ensure deferred init isn't running or pending */
	if (ns2_ble)
		cancel_work_sync(&ns2_ble->init_work);

	hid_dbg(hdev, "remove\n");
	mutex_lock(&ns2->lock);
	ns2->hdev  = NULL;
	ns2->cfg   = NULL;
	mutex_unlock(&ns2->lock);
	hid_hw_close(hdev);
	switch2_free_player_id(ns2->player_id);
	switch2_controller_put(ns2);
	hid_hw_stop(hdev);
}

/* ------------------------------------------------------------------ */
/* Module registration                                                  */
/* ------------------------------------------------------------------ */

static const struct hid_device_id switch2_ble_devices[] = {
	/* uhid devices created by BlueZ gatt-uhid bridge for BLE connections */
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			       USB_DEVICE_ID_NINTENDO_NS2_JOYCONL) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			       USB_DEVICE_ID_NINTENDO_NS2_JOYCONR) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			       USB_DEVICE_ID_NINTENDO_NS2_PROCON) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			       USB_DEVICE_ID_NINTENDO_NS2_GCCON) },
	{}
};
MODULE_DEVICE_TABLE(hid, switch2_ble_devices);

static struct hid_driver switch2_ble_hid_driver = {
	.name      = "switch2-ble",
	.id_table  = switch2_ble_devices,
	.probe     = switch2_ble_probe,
	.remove    = switch2_ble_remove,
	.raw_event = switch2_ble_raw_event,
};
module_hid_driver(switch2_ble_hid_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Martin BTS <martinbts@gmx.net>");
MODULE_DESCRIPTION("BLE transport for Nintendo Switch 2 Controllers");
