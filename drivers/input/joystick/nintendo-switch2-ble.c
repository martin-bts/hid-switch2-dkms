// SPDX-License-Identifier: GPL-2.0+
/*
 * BLE transport for Nintendo Switch 2 controllers
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
 *   - Input characteristic (0x000e): 63-byte controller reports.
 *     The payload is raw [seq][status][buttons...sticks...] with no
 *     report type byte.  We prepend the report type (determined from
 *     controller type) before passing to switch2_event().
 *   - ACK characteristic (0x001a): command responses with 0x91 framing.
 *     Routed to switch2_receive_command().
 *
 * Output path (send_command):
 *   Prepends [handle_lo][handle_hi] to the 0x91 frame and calls
 *   hid_hw_output_report().  The gatt-uhid bridge reads the handle
 *   and writes the frame to that GATT characteristic.
 *
 * Rumble path:
 *   Rumble does not really go through the NS2 transport drivers.
 *   hid-nintendo's rumble directly calls hid_hw_output_report(), which
 *   ultimately calls hdev->ll_driver->output_report().
 *   This means we have to wrap output_report() to add the rumble handle
 *   as a prefix, so that the GATT bridge does the right thing.
 *   Problem: when output_report() is called, we would have to parse the
 *            payload to identify if we are seeing rumble data, or other
 *            data.
 *   Currently, I don't want to interprete payload data, so we go by
 *   payload length. Luckily we don't use hid_hw_output_report() much.
 *
 * Copyright (c) 2025 Martin BTS <martinbts@gmx.net>
 */

#include "../../hid/hid-ids.h"
#include "../../hid/hid-nintendo.h"
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/slab.h>

/* GATT handle prefix size in bridge reports */
#define GATT_HANDLE_SIZE	2

/* GATT handle for the command/write characteristic */
#define NS2_GATT_CMD_HANDLE	0x0014

/* GATT handle for the haptic data characteristic */
#define NS2_GATT_HAPTIC_HANDLE	0x0012

/* GATT handle for input data */
#define NS2_GATT_INPUT_HANDLE	0x000e

/* GATT handle for PDU acks */
#define NS2_GATT_ACK_HANDLE	0x001a

struct switch2_ble_ctx {
	struct switch2_cfg_intf cfg;
	struct switch2_controller *ns2;
	struct hid_device *hdev;
	char phys[64];
	bool cfg_attached;
	struct work_struct init_work;
	/*
	 * Wrapped copy of this hdev's uhid ll_driver.  hdev->ll_driver is
	 * repointed at @ll so we can intercept output_report; @orig_ll keeps
	 * the original uhid driver to forward to.
	 */
	const struct hid_ll_driver *orig_ll;
	struct hid_ll_driver ll;
};

/* ------------------------------------------------------------------ */
/* Output: prepend GATT handle to outgoing frames                      */
/* ------------------------------------------------------------------ */

static int switch2_ble_send_cmd(enum switch2_cmd command, uint8_t subcommand,
	const void *msg, size_t len, struct switch2_cfg_intf *intf)
{
	struct switch2_ble_ctx *ctx = container_of(intf, struct switch2_ble_ctx, cfg);
	struct hid_device *hdev;
	struct switch2_cmd_header header = {
		command, NS2_DIR_OUT | NS2_FLAG_OK, NS2_TRANS_BT,
		subcommand, 0, len
	};
	uint8_t *frame;
	size_t frame_len;
	int ret;

	if (WARN_ON(len > 56))
		return -EINVAL;

	hdev = ctx->hdev;
	if (!hdev)
		return -ENODEV;

	frame_len = GATT_HANDLE_SIZE + sizeof(header) + len;
	frame = kzalloc(frame_len, GFP_KERNEL);
	if (!frame)
		return -ENOMEM;

	frame[0] = NS2_GATT_CMD_HANDLE & 0xff;
	frame[1] = (NS2_GATT_CMD_HANDLE >> 8) & 0xff;

	memcpy(frame + GATT_HANDLE_SIZE, &header, sizeof(header));
	if (msg && len)
		memcpy(frame + GATT_HANDLE_SIZE + sizeof(header), msg, len);

	/* bypass our rumble handle injecting output_report() */
	ret = ctx->orig_ll->output_report(hdev, frame, frame_len);
	hid_dbg(hdev, "send_cmd: cmd=0x%02x sub=0x%02x len=%zu ret=%d\n",
		command, subcommand, frame_len, ret);
	kfree(frame);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Deferred init: triggered by first raw_event via work queue          */
/* ------------------------------------------------------------------ */

static void switch2_ble_init_work(struct work_struct *work)
{
	struct switch2_ble_ctx *ctx =
		container_of(work, struct switch2_ble_ctx, init_work);

	/*
	 * cfg was pre-set during probe via set_cfg.  We use attach_cfg
	 * which will find the same controller by phys, set parent, and
	 * trigger init if hdev is ready.  Clear cfg first to avoid the
	 * WARN in attach_cfg.
	 */
	switch2_controller_set_cfg(ctx->ns2, NULL);
	if (switch2_controller_attach_cfg(ctx->phys, &ctx->cfg) < 0)
		return;

	ctx->cfg_attached = true;
}

/* ------------------------------------------------------------------ */
/* Input: parse handle prefix, route to appropriate handler            */
/* ------------------------------------------------------------------ */

static int switch2_ble_raw_event(struct hid_device *hdev,
	struct hid_report *report, uint8_t *raw_data, int size)
{
	struct switch2_controller *ns2;
	struct switch2_cfg_intf *cfg;
	struct switch2_ble_ctx *ctx;
	uint16_t handle;
	uint8_t *payload;
	int payload_len;

	if (report->type != HID_INPUT_REPORT)
		return 0;

	if (size < 1 + GATT_HANDLE_SIZE + 1)
		return -EINVAL;

	ns2 = hid_get_drvdata(hdev);
	if (!ns2)
		return 0;

	cfg = switch2_controller_get_cfg(ns2);
	if (!cfg)
		return 0;

	ctx = container_of(cfg, struct switch2_ble_ctx, cfg);

	/*
	 * First raw_event means the gatt-uhid bridge event loop is active.
	 * Kick off cfg attachment (and thus init) now — it's too early
	 * during probe (inside UHID_CREATE2).
	 */
	if (unlikely(!ctx->cfg_attached))
		schedule_work(&ctx->init_work);

	handle = raw_data[1] | ((uint16_t)raw_data[2] << 8);
	payload = raw_data + 1 + GATT_HANDLE_SIZE;
	payload_len = size - 1 - GATT_HANDLE_SIZE;

	switch (handle) {
	case NS2_GATT_ACK_HANDLE:
		hid_dbg(hdev, "ACK: %d bytes, first=%02x\n",
			payload_len, payload_len > 0 ? payload[0] : 0);
		switch2_receive_command(ns2, payload, payload_len);
		return 0;
	case NS2_GATT_INPUT_HANDLE:
		uint8_t buf[64];
		unsigned int orig_id = report->id;
		uint8_t report_type;
		int ret;

		switch (switch2_controller_get_type(ns2)) {
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
	default:
		hid_dbg(hdev, "unhandled handle 0x%04x, %d bytes\n", handle,
		payload_len);
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* Output report wrapper: inject the GATT haptic handle on rumble       */
/* ------------------------------------------------------------------ */

/*
 * Intercepts hid_hw_output_report() on this hdev. In hid-nintendo,
 * switch2_rumble_work() emits the raw 64-byte haptic buffer here with no
 * transport framing; the gatt-uhid bridge demuxes outgoing reports by a
 * 2-byte little-endian GATT handle prefix, so prepend the haptic
 * characteristic handle (0x0012).  Command writes go straight to
 * orig_ll->output_report (see switch2_ble_send_cmd()) and never reach this
 * path, so anything arriving here is a rumble frame.
 */
static int switch2_ble_output_report(struct hid_device *hdev, uint8_t *buf,
	size_t len)
{
	struct switch2_ble_ctx *ctx =
		container_of(hdev->ll_driver, struct switch2_ble_ctx, ll);
	uint8_t frame[GATT_HANDLE_SIZE + 64];

	/* Only the raw 64-byte rumble buffer is expected; pass anything
	 * unexpected through unchanged. */
	if (len != 64)
		return ctx->orig_ll->output_report(hdev, buf, len);

	frame[0] = NS2_GATT_HAPTIC_HANDLE & 0xff;
	frame[1] = (NS2_GATT_HAPTIC_HANDLE >> 8) & 0xff;
	memcpy(frame + GATT_HANDLE_SIZE, buf, 64);

	return ctx->orig_ll->output_report(hdev, frame, sizeof(frame));
}

/* ------------------------------------------------------------------ */
/* Probe / remove                                                      */
/* ------------------------------------------------------------------ */

static int switch2_ble_probe(struct hid_device *hdev,
	const struct hid_device_id *id)
{
	struct switch2_controller *ns2;
	struct switch2_ble_ctx *ctx;
	char phys[64];
	int ret;

	snprintf(phys, sizeof(phys), "bt%s", hdev->uniq);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed %d\n", ret);
		return ret;
	}

	/* Do not expose hidraw — Steam/SDL would write USB-format commands */
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

	ns2 = switch2_controller_attach_hdev(phys, hdev);
	if (IS_ERR(ns2)) {
		ret = PTR_ERR(ns2);
		goto err_close;
	}

	ctx = devm_kzalloc(&hdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto err_detach;
	}

	ctx->ns2 = ns2;
	ctx->hdev = hdev;
	strscpy(ctx->phys, phys, sizeof(ctx->phys));
	ctx->cfg.dev = &hdev->dev;
	ctx->cfg.send_command = switch2_ble_send_cmd;

	/* copy the ll_driver and put our output_report() wrapper in place */
	ctx->orig_ll = hdev->ll_driver;
	ctx->ll = *hdev->ll_driver;
	ctx->ll.output_report = switch2_ble_output_report;
	hdev->ll_driver = &ctx->ll;

	INIT_WORK(&ctx->init_work, switch2_ble_init_work);

	/*
	 * Pre-set cfg so raw_event can find ctx via container_of before
	 * attach_cfg runs.  attach_cfg will overwrite this properly
	 * (with the same pointer) and trigger init.
	 */
	switch2_controller_set_cfg(ns2, &ctx->cfg);

	hid_dbg(hdev, "probe complete, waiting for first event\n");
	return 0;

err_detach:
	switch2_controller_detach_hdev(ns2);
err_close:
	hid_hw_close(hdev);
err_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void switch2_ble_remove(struct hid_device *hdev)
{
	struct switch2_controller *ns2 = hid_get_drvdata(hdev);
	struct switch2_cfg_intf *cfg;
	struct switch2_ble_ctx *ctx;

	if (!ns2)
		return;

	cfg = switch2_controller_get_cfg(ns2);
	if (!cfg)
		return;

	ctx = container_of(cfg, struct switch2_ble_ctx, cfg);
	cancel_work_sync(&ctx->init_work);

	if (ctx->orig_ll)
		hdev->ll_driver = ctx->orig_ll;

	if (ctx->cfg_attached)
		switch2_controller_detach_cfg(ns2);
	else
		switch2_controller_set_cfg(ns2, NULL);

	hid_hw_close(hdev);
	switch2_controller_detach_hdev(ns2);
	hid_hw_stop(hdev);
}

/* ------------------------------------------------------------------ */
/* Module registration                                                 */
/* ------------------------------------------------------------------ */

static const struct hid_device_id switch2_ble_devices[] = {
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
	.name      = "nintendo-switch2-ble",
	.id_table  = switch2_ble_devices,
	.probe     = switch2_ble_probe,
	.remove    = switch2_ble_remove,
	.raw_event = switch2_ble_raw_event,
};
module_hid_driver(switch2_ble_hid_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Martin BTS <martinbts@gmx.net>");
MODULE_DESCRIPTION("BLE transport for Nintendo Switch 2 Controllers");
