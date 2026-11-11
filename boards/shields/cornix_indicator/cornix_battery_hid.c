/*
 * Cornix battery status over a second USB HID interface (left half = split central).
 *
 * A vendor-defined HID interface (usage page 0xFF00, usage 0x01) with one 16-byte
 * INPUT report and no report ID. The host polls it with GET_REPORT on the control
 * endpoint; nothing is pushed on the interrupt endpoint.
 *
 * Report layout (little-endian):
 *   [0]     version (0x01)
 *   [1]     flags: bit0 = right half connected
 *                  bit1 = right level valid (connected and a level has arrived
 *                         since the connection; the 0 placeholder does not count)
 *   [2]     right level % (0 if not valid)
 *   [3]     left level %
 *   [4..7]  left uptime in seconds (u32)
 *   [8..11] seconds since the right level was last received (u32, 0xFFFFFFFF if never)
 *   [12..15] 0
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/usb/usb_device.h>

#include <zmk/battery.h>
#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/split/central.h>
#include <zmk/split/transport/central.h>

#define REPORT_SIZE 16
#define STATUS_VERSION 0x01
#define FLAG_PEER_CONNECTED BIT(0)
#define FLAG_PEER_LEVEL_VALID BIT(1)

#define HID_GET_REPORT_TYPE_MASK 0xff00
#define HID_GET_REPORT_ID_MASK 0x00ff
#define HID_REPORT_TYPE_INPUT 0x100

/* Same trick as cornix_indicator.c: the active split transport tells us if the right is connected. */
extern const struct zmk_split_transport_central *active_transport;

static const uint8_t report_desc[] = {
    0x06, 0x00, 0xFF,    /* Usage Page (Vendor 0xFF00) */
    0x09, 0x01,          /* Usage (0x01) */
    0xA1, 0x01,          /* Collection (Application) */
    0x15, 0x00,          /*   Logical Minimum (0) */
    0x26, 0xFF, 0x00,    /*   Logical Maximum (255) */
    0x75, 0x08,          /*   Report Size (8) */
    0x95, REPORT_SIZE,   /*   Report Count (16) */
    0x09, 0x01,          /*   Usage (0x01) */
    0x81, 0x02,          /*   Input (Data, Var, Abs) */
    0xC0,                /* End Collection */
};

/* Uptime (ms) when the right level last arrived; -1 = never. 0 placeholders do not count. */
static volatile int64_t right_level_at_ms = -1;
static volatile bool right_level_valid;

static bool peer_connected(void) {
    if (active_transport == NULL || active_transport->api == NULL ||
        active_transport->api->get_status == NULL) {
        return false;
    }
    struct zmk_split_transport_status status = active_transport->api->get_status();
    return status.connections != ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_DISCONNECTED;
}

static int battery_listener(const zmk_event_t *eh) {
    const struct zmk_peripheral_battery_state_changed *ev =
        as_zmk_peripheral_battery_state_changed(eh);
    if (ev == NULL || ev->source != 0) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    /* Disconnect raises a level-0 placeholder; neither it nor events while disconnected are real. */
    if (ev->state_of_charge == 0 || !peer_connected()) {
        right_level_valid = false;
        return ZMK_EV_EVENT_BUBBLE;
    }

    right_level_at_ms = k_uptime_get();
    right_level_valid = true;
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(cornix_battery_hid, battery_listener);
ZMK_SUBSCRIPTION(cornix_battery_hid, zmk_peripheral_battery_state_changed);

static uint8_t report[REPORT_SIZE];

static void build_report(void) {
    uint8_t flags = 0;
    uint8_t right = 0;
    int64_t now = k_uptime_get();
    int64_t at = right_level_at_ms;
    bool connected = peer_connected();

    if (connected) {
        flags |= FLAG_PEER_CONNECTED;
        if (right_level_valid) {
            uint8_t level = 0;
            if (zmk_split_central_get_peripheral_battery_level(0, &level) == 0 && level > 0) {
                flags |= FLAG_PEER_LEVEL_VALID;
                right = level;
            }
        }
    } else {
        right_level_valid = false;
    }

    uint32_t age = 0xFFFFFFFFu;
    if (at >= 0 && now >= at) {
        age = (uint32_t)((now - at) / 1000);
    }

    memset(report, 0, sizeof(report));
    report[0] = STATUS_VERSION;
    report[1] = flags;
    report[2] = right;
    report[3] = zmk_battery_state_of_charge();
    sys_put_le32((uint32_t)(now / 1000), &report[4]);
    sys_put_le32(age, &report[8]);
}

static int get_report_cb(const struct device *dev, struct usb_setup_packet *setup, int32_t *len,
                         uint8_t **data) {
    if ((setup->wValue & HID_GET_REPORT_TYPE_MASK) != HID_REPORT_TYPE_INPUT ||
        (setup->wValue & HID_GET_REPORT_ID_MASK) != 0) {
        return -ENOTSUP;
    }

    build_report();
    *data = report;
    *len = REPORT_SIZE;
    return 0;
}

static const struct hid_ops battery_hid_ops = {
    .get_report = get_report_cb,
};

/*
 * Must run before zmk_usb_init() (usb_enable, ZMK_USB_INIT_PRIORITY = 96).
 * ZMK's own HID_0 init uses CONFIG_ZMK_USB_HID_INIT_PRIORITY (95) too.
 */
static int cornix_battery_hid_init(void) {
    const struct device *dev = device_get_binding("HID_1");
    if (dev == NULL) {
        return -ENODEV;
    }

    usb_hid_register_device(dev, report_desc, sizeof(report_desc), &battery_hid_ops);
    return usb_hid_init(dev);
}

SYS_INIT(cornix_battery_hid_init, APPLICATION, CONFIG_ZMK_USB_HID_INIT_PRIORITY);
