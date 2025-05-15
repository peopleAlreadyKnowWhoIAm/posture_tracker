#include "app/telemetry_storage.h"

#include <zephyr/logging/log.h>

#include "zephyr/fs/fcb.h"
#include "zephyr/kernel.h"

LOG_MODULE_REGISTER(telemetry_storage, LOG_LEVEL_DBG);

struct telemetry_work {
	struct k_work work;
	struct telemetry telemetry;
};

static struct telemetry_work work;

#define SECTOR_SIZE 0x1000 /* 4K */

static struct flash_sector fcb_sector[] = {
    [0] =
	{
	    .fs_off = 0,
	    .fs_size = SECTOR_SIZE
	},
    [1] = {.fs_off = SECTOR_SIZE, .fs_size = SECTOR_SIZE},
};

static struct fcb telemetry_storage = {
    .f_sector_cnt = ARRAY_SIZE(fcb_sector),
    .f_sectors = fcb_sector,
    .f_scratch_cnt = 0,
    .f_magic = 0xFBCB,
    .f_version = 1,
};

static void telemetry_handle(struct k_work *work) {
	struct telemetry_work *telemetry_work = CONTAINER_OF(work, struct telemetry_work, work);
	struct telemetry *telemetry = &telemetry_work->telemetry;
	// TODO: Fix telemetry timestamp
	telemetry->timestamp = k_uptime_get();
	LOG_INF("Telemetry data: %d", telemetry->timestamp);

        struct fcb_entry entry;
	int rc = fcb_append(&telemetry_storage, sizeof *telemetry, &entry);
        if (rc == -ENOSPC) {
                LOG_INF("Rotating sectors");
                rc = fcb_rotate(&telemetry_storage);
                if (rc != 0) {
                        LOG_ERR("FCB rotate failed: %d", rc);
                        return;
                }
                rc = fcb_append(&telemetry_storage, sizeof *telemetry, &entry);
                if (rc != 0) {
                        LOG_ERR("FCB append failed after rotation: %d", rc);
                        return;
                }
        } else if (rc < 0) {
                LOG_ERR("FCB append failed: %d", rc);
                return;
        }
        rc = flash_area_write(telemetry_storage.fap, FCB_ENTRY_FA_DATA_OFF(entry),
                              telemetry, sizeof *telemetry);
        if (rc != 0) {
                LOG_ERR("FCB write failed: %d", rc);
        }
        rc = fcb_append_finish(&telemetry_storage, &entry);
        if (rc != 0) {
                LOG_ERR("FCB append finish failed: %d", rc);
        
        }
}

void telemetry_storage_submit(struct telemetry *telemetry) {
	if (k_work_is_pending(&work.work)) {
		return;
	}
	k_work_init(&work.work, telemetry_handle);
	work.telemetry = *telemetry;
	k_work_submit(&work.work);
}

int telemetry_get_portion(uint8_t *buf, size_t *len) {
        static struct fcb_entry loc_ctx = {0};
        if (len == NULL) {
                loc_ctx = (struct fcb_entry){0};
                return 0;
        }
        size_t write_off = 0;
        while (write_off + sizeof(struct telemetry) <= *len) {
                int rc = fcb_getnext(&telemetry_storage, &loc_ctx);
                if (rc == -ENOTSUP) {
                        LOG_INF("FCB telem end");
                        *len = write_off;
                        return 1;
                } else if (rc < 0) {
                        LOG_ERR("FCB getnext failed: %d", rc);
                        return rc;
                }
                struct telemetry telemetry;
                rc = flash_area_read(telemetry_storage.fap, FCB_ENTRY_FA_DATA_OFF(loc_ctx),
                                     &telemetry, sizeof(telemetry));
                if (rc != 0) {
                        LOG_ERR("FCB read failed: %d", rc);
                        return rc;
                }
                memcpy(buf + write_off, &telemetry, sizeof(telemetry));
                write_off += sizeof(telemetry);
        }
        *len = write_off;
        return 0;
}
        
// TODO count all telemetry pieces
// static int telemetry_walk_cb(struct fcb_entry_ctx *loc_ctx, void *arg) {
//         struct telemetry telemetry;
//         int rc = flash_area_read(loc_ctx->fap, FCB_ENTRY_FA_DATA_OFF(loc_ctx->loc),
//                                  &telemetry, sizeof(telemetry));
//         if (rc != 0) {
//                 LOG_ERR("FCB read failed: %d", rc);
//                 return rc;
//         }
//         LOG_INF("Telemetry data: %d", telemetry.timestamp);
//         return 0;
// }

static int telemetry_storage_init(void) {
	int rc = fcb_init(FIXED_PARTITION_ID(telemetry_partition), &telemetry_storage);
	if (rc != 0) {
		LOG_ERR("FCB init failed: %d", rc);
                return rc;
	}
        // fcb_walk(&telemetry_storage, NULL, telemetry_walk_cb, NULL);
	LOG_INF("FCB init success, Empyt: %d", fcb_is_empty(&telemetry_storage));
	return rc;
}

SYS_INIT(telemetry_storage_init, APPLICATION, 2);
