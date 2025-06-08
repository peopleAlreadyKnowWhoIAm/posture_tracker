#include "app/posture_detection.h"
#include "app/telemetry_storage.h"
#include "services/nus/nus_internal.h"
#include "zephyr/bluetooth/addr.h"
#include "zephyr/bluetooth/bluetooth.h"
#include "zephyr/bluetooth/conn.h"
#include "zephyr/bluetooth/gap.h"
#include "zephyr/bluetooth/services/nus.h"
#include "zephyr/bluetooth/services/nus/inst.h"
#include "zephyr/init.h"
#include "zephyr/kernel.h"
#include "zephyr/logging/log.h"
#include "zephyr/settings/settings.h"

#include "app/bluetooth_support.h"

LOG_MODULE_REGISTER(bt_support, LOG_LEVEL_DBG);

enum bt_adv_type { BT_ADV_NONE, BT_ADV_OPEN, BT_ADV_DIR };

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define POSTURE_NOTIF ((const uint8_t[]){'N', 'P'})
#define MOVEMENT_NOTIF ((const uint8_t[]){'N', 'M'})
#define STATE_MARKER ((const uint8_t)'S')
#define SETTINGS_RESP_MARKER ((const uint8_t)'U')

#define TRANSFER_DONE_MARKER ((const uint8_t[]){'T', 'D'})

#define STATE_REQ_MARKER ((const uint8_t[]){'R', 'S'})
#define SETTINGS_REQ_MARKER ((const uint8_t[]){'R','U'})
#define SETTINGS_MARKER ((const uint8_t[]){'S'})
#define SETTING_CALIBRATION ((const uint8_t[]){'C'})
#define SETTING_WORKING_MARKER ((const uint8_t[]){'W'})
#define SETTING_TIMEOUT_MARKER ((const uint8_t[]){'T'})
#define SETTING_RANGE_MARKER ((const uint8_t[]){'R'})

#define TELEMETRY_MARKER ((const uint8_t[]){'T', 'E', 'L', 'E', 'M'})

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static enum bt_adv_type bt_adv_state;
static struct bt_conn *bt_conn;
static K_MUTEX_DEFINE(bt_conn_mutex);

static void copy_last_bonded_addr(const struct bt_bond_info *info, void *data) {
	bt_addr_le_t *bond_addr = data;

	// Copy the last bonded address
	bt_addr_le_copy(bond_addr, &info->addr);
}
static bool get_paired_peer(bt_addr_le_t *bond_addr) {
	bt_addr_le_copy(bond_addr, BT_ADDR_LE_NONE);
	bt_foreach_bond(BT_ID_DEFAULT, copy_last_bonded_addr, bond_addr);
	return bt_addr_le_cmp(bond_addr, BT_ADDR_LE_NONE) != 0;
}

static int update_advertisement(void) {
	enum bt_adv_type desired_adv_type = BT_ADV_NONE;

	bt_addr_le_t peer_address = {0};
	if (bt_conn == NULL) {
		if (get_paired_peer(&peer_address)) {
			char addr_str[BT_ADDR_LE_STR_LEN];
			bt_addr_le_to_str(&peer_address, addr_str, sizeof(addr_str));

			LOG_DBG("Directed advertising to %s", addr_str);
			desired_adv_type = BT_ADV_DIR;
		} else {
			desired_adv_type = BT_ADV_OPEN;
		}
	}

	LOG_INF("Changing adv state from %d to %d", bt_adv_state, desired_adv_type);

	if (desired_adv_type == bt_adv_state) {
		return 0;
	}

	if (bt_adv_state != BT_ADV_NONE) {
		int err = bt_le_adv_stop();
		if (err) {
			LOG_ERR("Failed to stop advertising (err %d)", err);
			return err;
		}
		bt_adv_state = BT_ADV_NONE;
	}

	if (desired_adv_type == BT_ADV_DIR) {
		// Doesn't work with my smartphone - using open advertisement
		if (bt_conn != NULL) {
			LOG_DBG(
			    "Skipping advertising, profile host is already "
			    "connected");
			return 0;
		}

		// struct bt_le_adv_param adv_param = *BT_LE_ADV_CONN_DIR_LOW_DUTY(&peer_address);
		// adv_param.options |= BT_LE_ADV_OPT_DIR_ADDR_RPA;
		// int err = bt_le_adv_start(&adv_param, NULL, 0, NULL, 0);
		int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
		if (err) {
			LOG_ERR("Advertising failed to start (err %d)", err);
			return err;
		}
		bt_adv_state = BT_ADV_DIR;
	} else if (desired_adv_type == BT_ADV_OPEN) {
		int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
		if (err) {
			LOG_ERR("Advertising failed to start (err %d)", err);
			return err;
		}
		bt_adv_state = BT_ADV_OPEN;
	}
	return 0;
}

static void update_advertising_callback(struct k_work *work) {
	(void)work;
	(void)update_advertisement();
}
K_WORK_DEFINE(update_advertisement_work, &update_advertising_callback);

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
			     uint16_t timeout) {
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_DBG("%s: interval %d latency %d timeout %d", addr, interval, latency, timeout);
}

static void connected(struct bt_conn *conn, uint8_t err) {
	char addr[BT_ADDR_LE_STR_LEN];
	struct bt_conn_info info;

	(void)bt_conn_get_info(conn, &info);

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if (err) {
		LOG_WRN("Failed to connect to %s (%u)", addr, err);
		(void)update_advertisement();
		return;
	}

	k_mutex_lock(&bt_conn_mutex, K_FOREVER);
	// Bt advertisement has been stopped
	bt_adv_state = BT_ADV_NONE;
	bt_conn = bt_conn_ref(conn);
	k_mutex_unlock(&bt_conn_mutex);

	LOG_INF("Connected %s", addr);
	(void)update_advertisement();
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
	char addr[BT_ADDR_LE_STR_LEN];

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	k_mutex_lock(&bt_conn_mutex, K_FOREVER);
	bt_conn_unref(bt_conn);
	bt_conn = NULL;
	k_mutex_unlock(&bt_conn_mutex);
	LOG_INF("Disconnected from %s, reson BT_HCI_ERR_ %d", addr, reason);

	// Update ad as work because bluetooth connection state isn't updated
	// here
	(void)k_work_submit(&update_advertisement_work);
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err) {
	if (err) {
		LOG_ERR("Security failed (err %d)", err);
	} else {
		LOG_INF("Security level changed to %d", level);
	}
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .le_param_updated = le_param_updated,
    .security_changed = security_changed,
};

static void auth_cancel(struct bt_conn *conn) {
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_DBG("Pairing cancelled: %s", addr);
}

static struct bt_conn_auth_cb auth_cbs = {
    .cancel = &auth_cancel,
};

static inline bool is_secure_enough(struct bt_conn *conn) {
	struct bt_conn_info info;
	int err = bt_conn_get_info(conn, &info);
	if (err != 0) {
		LOG_INF("Failed to get connection info (err %d)", err);
		return false;
	}
	return info.security.level >= BT_SECURITY_L2;
}

static void auth_pairing_complete(struct bt_conn *conn, bool bonded) {
	struct bt_conn_info info;
	char addr[BT_ADDR_LE_STR_LEN];
	const bt_addr_le_t *dst = bt_conn_get_dst(conn);

	bt_addr_le_to_str(dst, addr, sizeof(addr));
	bt_conn_get_info(conn, &info);

	if (info.role != BT_CONN_ROLE_PERIPHERAL) {
		LOG_DBG("SKIPPING FOR ROLE %d", info.role);
		return;
	}
	LOG_INF("Pairing completed");

	(void)update_advertisement();
};

static struct bt_conn_auth_info_cb ble_auth_info_cb_display = {
    .pairing_complete = &auth_pairing_complete,
};

static void bluetooth_send_buf(const uint8_t *buf, size_t len, bt_gatt_complete_func_t callback) {
	if (bt_conn == NULL) {
		LOG_INF("No connection, not sending data");
		return;
	}
	k_mutex_lock(&bt_conn_mutex, K_FOREVER);
	if (is_secure_enough(bt_conn)) {
		// Workaround for filling up all tx pool
		struct bt_nus_inst *instance = bt_nus_inst_default();
		struct bt_gatt_notify_params gatt_params = {
		    .attr = &instance->svc->attrs[1],
		    .data = buf,
		    .len = len,
		    .func = callback,
		};
		int err = bt_gatt_notify_cb(bt_conn, &gatt_params);
		if (err != 0) {
			LOG_ERR("Failed to send data (err %d)", err);
		} else {
			LOG_DBG("Sent %zu bytes", len);
		}
	}
	k_mutex_unlock(&bt_conn_mutex);
}

static void bluetooth_support_notify_settings(void) {
	struct posture_settings settings = posture_detection_get_settings();
	uint8_t buf[sizeof(SETTINGS_RESP_MARKER) + sizeof settings] = {SETTINGS_RESP_MARKER};
	memcpy(buf + sizeof(SETTINGS_RESP_MARKER), &settings, sizeof settings );
	bluetooth_send_buf(buf, sizeof buf, NULL);
}

static void parse_setting_payload(const uint8_t *data, size_t len) {
	size_t offset = 0;
	while (len > offset) {
		if (len - offset < sizeof(SETTING_CALIBRATION)) {
			LOG_ERR("Invalid settings payload");
			return;
		}
		if (memcmp(data + offset, SETTING_CALIBRATION, sizeof(SETTING_CALIBRATION)) == 0) {
			LOG_INF("Calibration settings received");
			offset += sizeof(SETTING_CALIBRATION);
			posture_detection_do_calibration();
		} else {
			if (len - offset < sizeof(SETTING_TIMEOUT_MARKER) + 1) {
				LOG_ERR("Invalid settings value");
				return;
			}
			if (memcmp(data + offset, SETTING_TIMEOUT_MARKER,
				   sizeof(SETTING_TIMEOUT_MARKER)) == 0) {
				offset += sizeof(SETTING_TIMEOUT_MARKER);
				LOG_INF("Timeout marker settings received %d", data[offset]);
				posture_detection_set_timeout(data[offset]);
				offset++;
			} else if (memcmp(data + offset, SETTING_WORKING_MARKER,
					  sizeof(SETTING_WORKING_MARKER)) == 0) {
				LOG_INF("Working marker settings received %d", data[offset]);
				offset += sizeof(SETTING_WORKING_MARKER);
				posture_detection_set_enabled(data[offset]);
				offset++;
			} else if (memcmp(data + offset, SETTING_RANGE_MARKER,
					  sizeof(SETTING_RANGE_MARKER)) == 0) {
				LOG_INF("Range marker settings received %d", data[offset]);
				offset += sizeof(SETTING_RANGE_MARKER);
				posture_detection_set_working_range(data[offset]);
				offset++;
			} else {
				LOG_ERR("Unknown setting marker");
				return;
			}
		}
	}
	posture_detection_save_settings();
	bluetooth_support_notify_settings();
}

static void transfer_telemetry_callback(struct bt_conn *, void *);

struct telemetry_transfer_work {
	struct k_work work;
	unsigned piece;
};

static void transfer_telemetry(struct k_work *work) {
	(void)work;
	struct telemetry_transfer_work *telemetry_trans =
	    CONTAINER_OF(work, struct telemetry_transfer_work, work);
	static uint8_t telemetry_buf[500];
	telemetry_buf[0] = telemetry_trans->piece;
	telemetry_trans->piece++;
	size_t len = sizeof(telemetry_buf) - 1;
	int err = telemetry_get_portion(telemetry_buf + 1, &len);
	if (err < 0) {
		LOG_ERR("Failed to get telemetry portion (err %d)", err);
		bluetooth_send_buf(TRANSFER_DONE_MARKER, sizeof(TRANSFER_DONE_MARKER), NULL);
		return;
	}
	if (err == 1 && len == 0) {
		LOG_INF("Done telemetry transfer");
		bluetooth_send_buf(TRANSFER_DONE_MARKER, sizeof(TRANSFER_DONE_MARKER), NULL);
		return;
	}
	bluetooth_send_buf(telemetry_buf, len + 1, transfer_telemetry_callback);
}

static struct telemetry_transfer_work telemetry_work;

static void start_telemetry_transfer(void) {
	k_work_init(&telemetry_work.work, &transfer_telemetry);
	telemetry_work.piece = 0;
	// Reset internal pointer
	(void)telemetry_get_portion(NULL, NULL);
	k_work_submit(&telemetry_work.work);
}

static void transfer_telemetry_callback(struct bt_conn *, void *) {
	k_work_init(&telemetry_work.work, &transfer_telemetry);
	k_work_submit(&telemetry_work.work);
}

void bluetooth_support_notify_state(enum posture_state state) {
	uint8_t send_buf[] = {STATE_MARKER, (uint8_t)state};
	bluetooth_send_buf(send_buf, sizeof send_buf, NULL);
}

static void bt_data_received(struct bt_conn *conn, const void *data, uint16_t len, void *) {
	if (conn != bt_conn) {
		LOG_WRN("Received data from unknown connection");
		return;
	}

	if (!is_secure_enough(conn)) {
		LOG_WRN("Security level too low, ignoring data");
		return;
	}
	LOG_DBG("Received data: %.*s", len, (char *)data);
	if (len > sizeof(SETTINGS_MARKER) &&
	    memcmp(data, SETTINGS_MARKER, sizeof(SETTINGS_MARKER)) == 0) {
		LOG_INF("Settings received");
		parse_setting_payload((const uint8_t *)data + sizeof(SETTINGS_MARKER),
				      len - sizeof(SETTINGS_MARKER));
	} else if (len == sizeof(TELEMETRY_MARKER) &&
		   memcmp(data, TELEMETRY_MARKER, sizeof(TELEMETRY_MARKER)) == 0) {
		LOG_INF("Telemetry marker received");
		start_telemetry_transfer();
	} else if (len == sizeof(STATE_REQ_MARKER) && memcmp(data, STATE_REQ_MARKER, sizeof(STATE_REQ_MARKER)) == 0) {
		LOG_INF("Sending state");
		bluetooth_support_notify_state(posture_detection_get_state());
	} else if (len == sizeof(SETTINGS_REQ_MARKER) && memcmp(data, SETTINGS_REQ_MARKER, sizeof(SETTINGS_REQ_MARKER)) == 0) {
		LOG_INF("Sending sett");
		bluetooth_support_notify_settings();
	} else {
		LOG_INF("Unknown data received");
	}
}

static void notif_enabled(bool notif, void *) {
	LOG_INF("Notif status changed %d", notif);
}

static struct bt_nus_cb nus_callbacks = {
    .notif_enabled = notif_enabled,
    .received = bt_data_received,
};

static int bluetooth_init(void) {
	int err;
	err = bt_enable(NULL);
	if (err != 0) {
		LOG_ERR("Error enabling bluetooth %d", err);
		return err;
	}

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		settings_load();
	}

	bt_conn_cb_register(&conn_callbacks);
	bt_conn_auth_cb_register(&auth_cbs);
	bt_conn_auth_info_cb_register(&ble_auth_info_cb_display);
	bt_nus_cb_register(&nus_callbacks, NULL);
	k_work_submit(&update_advertisement_work);
	return 0;
}
SYS_INIT(bluetooth_init, APPLICATION, 1);

void bluetooth_support_notify_posture(void) {
	bluetooth_send_buf(POSTURE_NOTIF, sizeof(POSTURE_NOTIF), NULL);
}

void bluetooth_support_notify_movement(void) {
	bluetooth_send_buf(MOVEMENT_NOTIF, sizeof(MOVEMENT_NOTIF), NULL);
}

void bluetooth_remove_bonded_peer(void) {
	LOG_INF("Removing bonded peer");
	bt_unpair(BT_ID_DEFAULT, NULL);
	update_advertisement();
}
