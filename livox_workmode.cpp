/*
 * livox_workmode — Set Livox MID-360 work mode (sleep / normal / standby).
 *
 * Usage:  livox_workmode <config.json> <sleep|standby|normal>
 * Build:  g++ -o livox_workmode livox_workmode.cpp \
 *             -llivox_lidar_sdk_shared -lpthread
 *
 * Exit codes:
 *   0  success
 *   1  bad arguments / SDK init failure
 *   2  timeout waiting for lidar
 *   3  SDK reported failure after all retries
 */

#include <cstdio>
#include <cstring>
#include <atomic>
#include <chrono>
#include <thread>

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

static const int MAX_RETRIES = 5;
static const int RETRY_DELAY_MS = 1500;

static std::atomic<bool>           g_done{false};
static std::atomic<bool>           g_success{false};
static std::atomic<int>            g_retries{0};
static LivoxLidarWorkMode          g_target_mode;

/* ---- Query current state (diagnostic) -------------------------------- */

static void QueryInfoCallback(livox_status status, uint32_t handle,
                               LivoxLidarDiagInternalInfoResponse *response,
                               void * /*client_data*/) {
    if (status != kLivoxLidarStatusSuccess || !response) {
        printf("[livox_workmode] QueryInternalInfo failed (status=%d)\n", status);
        return;
    }
    uint16_t off = 0;
    for (uint8_t i = 0; i < response->param_num; ++i) {
        LivoxLidarKeyValueParam *kv = (LivoxLidarKeyValueParam *)&response->data[off];
        if (kv->key == kKeyCurWorkState) {
            printf("[livox_workmode] Current work state: %u\n", kv->value[0]);
        }
        if (kv->key == kKeyWorkMode) {
            printf("[livox_workmode] Target work mode: %u\n", kv->value[0]);
        }
        off += sizeof(uint16_t) * 2 + kv->length;
    }
}

/* ---- Work mode callback with retry ----------------------------------- */

static void WorkModeCallback(livox_status status, uint32_t handle,
                              LivoxLidarAsyncControlResponse *response,
                              void * /*client_data*/) {
    int ret = response ? response->ret_code : -1;
    int err_key = response ? response->error_key : -1;

    if (status == kLivoxLidarStatusSuccess && ret == 0) {
        printf("[livox_workmode] Work mode set OK (handle=%u)\n", handle);
        g_success = true;
        g_done = true;
        return;
    }

    int attempt = g_retries.fetch_add(1) + 1;
    printf("[livox_workmode] Attempt %d failed (status=%d ret_code=%d error_key=%d)\n",
           attempt, status, ret, err_key);

    if (attempt >= MAX_RETRIES) {
        printf("[livox_workmode] Giving up after %d attempts\n", MAX_RETRIES);
        g_done = true;
        return;
    }

    std::thread([handle]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(RETRY_DELAY_MS));
        if (!g_done) {
            printf("[livox_workmode] Retrying work mode change...\n");
            SetLivoxLidarWorkMode(handle, g_target_mode, WorkModeCallback, nullptr);
        }
    }).detach();
}

/* ---- Lidar discovered ------------------------------------------------ */

static void InfoChangeCallback(const uint32_t handle,
                                const LivoxLidarInfo *info,
                                void * /*client_data*/) {
    if (!info) return;
    printf("[livox_workmode] Lidar found: SN=%s IP=%s handle=%u\n",
           info->sn, info->lidar_ip, handle);

    /* Query current state for diagnostics. */
    QueryLivoxLidarInternalInfo(handle, QueryInfoCallback, nullptr);

    /* Brief delay so the lidar completes its post-handshake setup. */
    std::thread([handle]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        if (!g_done) {
            printf("[livox_workmode] Sending work mode command (mode=0x%02x)...\n",
                   (unsigned)g_target_mode);
            SetLivoxLidarWorkMode(handle, g_target_mode, WorkModeCallback, nullptr);
        }
    }).detach();
}

/* ---- main ------------------------------------------------------------ */

int main(int argc, char *argv[]) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <config.json> <sleep|standby|normal>\n", argv[0]);
        return 1;
    }

    const char *config_path = argv[1];
    const char *mode_str    = argv[2];

    if (strcmp(mode_str, "sleep") == 0) {
        g_target_mode = kLivoxLidarSleep;       /* 0x03 */
    } else if (strcmp(mode_str, "standby") == 0) {
        g_target_mode = kLivoxLidarWakeUp;      /* 0x02 — "ready / standby" */
    } else if (strcmp(mode_str, "normal") == 0) {
        g_target_mode = kLivoxLidarNormal;       /* 0x01 */
    } else {
        fprintf(stderr, "Invalid mode '%s' — use 'sleep', 'standby', or 'normal'\n", mode_str);
        return 1;
    }

    printf("[livox_workmode] Setting MID-360 to %s (0x%02x) …\n",
           mode_str, (unsigned)g_target_mode);

    if (!LivoxLidarSdkInit(config_path)) {
        fprintf(stderr, "[livox_workmode] SDK init failed\n");
        return 1;
    }

    SetLivoxLidarInfoChangeCallback(InfoChangeCallback, nullptr);

    for (int i = 0; i < 200 && !g_done; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    LivoxLidarSdkUninit();

    if (!g_done) {
        fprintf(stderr, "[livox_workmode] Timeout — lidar not found\n");
        return 2;
    }

    printf("[livox_workmode] Done (success=%d)\n", g_success.load());
    return g_success ? 0 : 3;
}
