#include "zephyr/bluetooth/addr.h"
#include "zephyr/bluetooth/bluetooth.h"
#include "zephyr/bluetooth/conn.h"
#include "zephyr/bluetooth/gap.h"
#include "zephyr/bluetooth/services/nus.h"
#include "zephyr/bluetooth/services/nus/inst.h"
#include "zephyr/init.h"
#include "zephyr/kernel.h"
#include "zephyr/logging/log.h"
#include <stdatomic.h>

LOG_MODULE_REGISTER(bt_support, LOG_LEVEL_DBG);

enum bt_adv_type { BT_ADV_NONE, BT_ADV_OPEN, BT_ADV_DIR };

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};

// TODO maybe add spinlock
static bt_addr_le_t peer_address = {};

static enum bt_adv_type bt_adv_state;

static bool is_paired(void) {
        return bt_addr_le_cmp(&peer_address, BT_ADDR_LE_ANY) != 0;
}

static bool is_paired_peer_connected(void) {
        if (!is_paired()) {
                return false;
        }

        struct bt_conn *conn =
            bt_conn_lookup_addr_le(BT_ID_DEFAULT, &peer_address);
        if (conn == NULL) {
                return false;
        }

        struct bt_conn_info info;

        (void)bt_conn_get_info(conn, &info);

        bt_conn_unref(conn);

        return info.state == BT_CONN_STATE_CONNECTED;
}

static int update_advertisement(void) {
        enum bt_adv_type desired_adv_type = BT_ADV_NONE;

        if (!is_paired()) {
                desired_adv_type = BT_ADV_OPEN;
        } else if (!is_paired_peer_connected()) {
                char addr_str[BT_ADDR_LE_STR_LEN];
                bt_addr_le_to_str(&peer_address, addr_str, sizeof(addr_str));

                LOG_DBG("Directed advertising to %s", addr_str);
                desired_adv_type = BT_ADV_DIR;
        }

        LOG_INF("Changing adv state from %d to %d", bt_adv_state,
                desired_adv_type);

        if (desired_adv_type == bt_adv_state) {
                return 0;
        }

        if (bt_adv_state != BT_ADV_NONE) {
                int err = bt_le_adv_stop();
                bt_adv_state = BT_ADV_NONE;
                if (err) {
                        LOG_ERR("Failed to stop advertising (err %d)", err);
                        return err;
                }
        }

        if (desired_adv_type == BT_ADV_DIR) {
                // Doesn't work with my smartphone - using open advertisement
                if (is_paired_peer_connected()) {
                        LOG_DBG("Skipping advertising, profile host is already "
                                "connected");
                        return 0;
                }

		// struct bt_le_adv_param adv_param = *BT_LE_ADV_CONN_DIR_LOW_DUTY(&peer_address);
		// adv_param.options |= BT_LE_ADV_OPT_DIR_ADDR_RPA;
                // int err =
                //     bt_le_adv_start(&adv_param,
                //                     NULL, 0, NULL, 0);
                int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad,
                                          ARRAY_SIZE(ad), NULL, 0);
                if (err) {
                        LOG_ERR("Advertising failed to start (err %d)", err);
                        return err;
                }
                bt_adv_state = BT_ADV_DIR;
        } else if (desired_adv_type == BT_ADV_OPEN) {
                int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad,
                                          ARRAY_SIZE(ad), NULL, 0);
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

// static void security_changed(struct bt_conn *conn, bt_security_t level,
//                              enum bt_security_err err) {
//         char addr[BT_ADDR_LE_STR_LEN];

//         bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

//         if (!err) {
//                 LOG_DBG("Security changed: %s level %u", addr, level);
//         } else {
//                 LOG_ERR("Security failed: %s level %u err %d", addr, level,
//                         err);
//         }
// }

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
                             uint16_t latency, uint16_t timeout) {
        char addr[BT_ADDR_LE_STR_LEN];

        bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

        LOG_DBG("%s: interval %d latency %d timeout %d", addr, interval,
                latency, timeout);
}

static void connected(struct bt_conn *conn, uint8_t err) {
        char addr[BT_ADDR_LE_STR_LEN];
        struct bt_conn_info info;

        (void)bt_conn_get_info(conn, &info);

        (void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

        // Bt advertisement has been stopped
        bt_adv_state = BT_ADV_NONE;
        if (err) {
                LOG_WRN("Failed to connect to %s (%u)", addr, err);
                (void)update_advertisement();
                return;
        }

        LOG_INF("Connected %s", addr);
        (void)update_advertisement();
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
        char addr[BT_ADDR_LE_STR_LEN];

        (void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

        LOG_INF("Disconnected from %s, reson BT_HCI_ERR_ %d", addr, reason);

        // Update ad as work because bluetooth connection state isn't updated
        // here
        (void)k_work_submit(&update_advertisement_work);
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .le_param_updated = le_param_updated,
};

bool pairing_allowed_for_current_profile(const struct bt_conn *conn) {
        return !is_paired() ||
               bt_addr_le_cmp(&peer_address, bt_conn_get_dst(conn)) == 0;
}

static enum bt_security_err
auth_pairing_accept(struct bt_conn *conn,
                    const struct bt_conn_pairing_feat *const feat) {
        (void)feat;
        struct bt_conn_info info;
        bt_conn_get_info(conn, &info);

        if (!pairing_allowed_for_current_profile(conn)) {
                LOG_WRN("Rejecting pairing request to taken profile");
                return BT_SECURITY_ERR_PAIR_NOT_ALLOWED;
        }
        return BT_SECURITY_ERR_SUCCESS;
}

static void auth_cancel(struct bt_conn *conn) {
        char addr[BT_ADDR_LE_STR_LEN];

        bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

        LOG_DBG("Pairing cancelled: %s", addr);
}

static struct bt_conn_auth_cb auth_cbs = {
    .pairing_accept = &auth_pairing_accept,
    .cancel = &auth_cancel,
};

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

        if (!pairing_allowed_for_current_profile(conn)) {
                LOG_ERR("Pairing completed but current profile is not open: %s",
                        addr);
                bt_unpair(BT_ID_DEFAULT, dst);
                return;
        }
        LOG_INF("Pairing completed");

        bt_addr_le_copy(&peer_address, dst);
        (void)update_advertisement();
};

static struct bt_conn_auth_info_cb ble_auth_info_cb_display = {
    .pairing_complete = &auth_pairing_complete,
};

static int bluetooth_init(void) {
        int err;
        err = bt_enable(NULL);
        if (err != 0) {
                LOG_ERR("Error enabling bluetooth %d", err);
                return err;
        }
        bt_conn_cb_register(&conn_callbacks);
        bt_conn_auth_cb_register(&auth_cbs);
        bt_conn_auth_info_cb_register(&ble_auth_info_cb_display);

        k_work_submit(&update_advertisement_work);
        return 0;
}

SYS_INIT(bluetooth_init, APPLICATION, 1);