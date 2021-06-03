#include <zephyr.h>
#include <disk/disk_access.h>
#include <fs/fs.h>
#include <ff.h>
#include <logging/log_backend.h>
#include <logging/log_output.h>
#include <logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "thread_runner.hpp"

namespace {

LOG_MODULE_REGISTER(logger, LOG_LEVEL_INF);

class directory_util {
public:
    enum class ORDER {ASCENT, DESCENT};
    directory_util() {
        reset();
    }
    uint32_t get_entries(ORDER order, const char* path) {
        uint32_t count = 0;
        reset();
        fs_dir_t dir;
        fs_dir_t_init(&dir);
        if (fs_opendir(&dir, path) == 0) {
            while (fs_readdir(&dir, &dirent) == 0 && dirent.name[0] != 0) {
                if (dirent.type == FS_DIR_ENTRY_FILE) {
                    push(order, dirent.name);
                    ++count;
                }
            }
            fs_closedir(&dir);
        }
        return count;
    }
    static constexpr uint32_t SIZE = 10;
    char array[SIZE][MAX_FILE_NAME + 1];
private:
    void push(ORDER order, const char* str) {
        for (uint32_t i = 0; i < SIZE; ++i) {
            if (array[i][0] == 0) {
                stor(array[i], str);
                break;
            } else {
                int comp = strcmp(str, array[i]);
                if ((order == ORDER::ASCENT && comp < 0) ||
                    (order == ORDER::DESCENT && comp > 0)) {
                    for (uint32_t j = SIZE - 1; j > i; --j) {
                        if (array[j - 1][0] != 0)
                            stor(array[j], array[j - 1]);
                    }
                    stor(array[i], str);
                    break;
                }
            }
        }
    }
    void reset() {
        for (uint32_t i = 0; i < SIZE; ++i)
            array[i][0] = 0;
    }
    void stor(char* to, const char* from) const {
        snprintf(to, MAX_FILE_NAME + 1, "%s", from);
    }
    fs_dirent dirent;
};

class log_manager {
public:
    void set_rootpath(const char* rootpath) {
        snprintf(this->rootpath, sizeof this->rootpath, "%s", rootpath);
        snprintf(workpath, sizeof workpath, "%s/log", rootpath);
        if (fs_mkdir(workpath) == 0)
            LOG_INF("make log directory. (%s)", log_strdup(workpath));
        logpath[0] = 0;
        write_error_count = 0;
    }
    void maintain() {
        snprintf(workpath, sizeof workpath, "%s/log", rootpath);
        uint32_t count = entries.get_entries(directory_util::ORDER::ASCENT, workpath);
        LOG_INF("SD CARD log count %d", count);
        if (count >= MAX_FILE_COUNT) {
            for (uint32_t i = 0; i < entries.SIZE; ++i) {
                if (entries.array[i][0] != 0)
                    fs_unlink(entries.array[i]);
            }
        }
        int32_t freeMB = get_freeMB(rootpath);
        LOG_INF("SD CARD free %dMB", freeMB);
        if (freeMB > 0 && freeMB < MIN_FREE_MB) {
            entries.get_entries(directory_util::ORDER::ASCENT, workpath);
            for (uint32_t i = 0; i < entries.SIZE; ++i) {
                if (entries.array[i][0] != 0) {
                    fs_unlink(entries.array[i]);
                    if (get_freeMB(rootpath) >= MIN_FREE_MB)
                        break;
                }
            }
        }
    }
    void setup_newlog() {
        snprintf(workpath, sizeof workpath, "%s/log", rootpath);
        entries.get_entries(directory_util::ORDER::DESCENT, workpath);
        if (entries.array[0][0] == 0) {
            snprintf(logpath, sizeof logpath, "%s/log/log000000.log", rootpath);
        } else {
            int n = atoi(&entries.array[0][3]);
            snprintf(logpath, sizeof logpath, "%s/log/log%06u.log", rootpath, n + 1);
        }
        output_enabled = false; //@@
        LOG_INF("new log path %s", log_strdup(logpath));
        fs_file_t_init(&logfp);
    }
    static int log_output(uint8_t *buf, size_t size, void *ctx) {
        log_manager* self = static_cast<log_manager*>(ctx);
        self->output(buf, size);
        return size;
    }
private:
    void output(const uint8_t* data, uint32_t length) {
        if (output_enabled && logpath[0] != 0 && data != nullptr && length != 0) {
            if (logfp.mp == NULL)
                fs_open(&logfp, logpath, FS_O_WRITE | FS_O_CREATE | FS_O_APPEND);
            if (logfp.mp != NULL) {
                if (fs_write(&logfp, data, length) == static_cast<ssize_t>(length)) {
                    fs_sync(&logfp);
                } else {
                    if (++write_error_count > 10) {
                        fs_close(&logfp);
                        output_enabled = false;
                        LOG_INF("Too many write errors, disable output to MicroSD.");
                    }
                }
            }
        }
    }
    int32_t get_freeMB(const char* path) const {
        int32_t freeMB = -1;
        struct fs_statvfs statvfs{0};
        if (fs_statvfs(path, &statvfs) == 0)
            freeMB = static_cast<int64_t>(statvfs.f_frsize) * static_cast<int64_t>(statvfs.f_bfree) / 1000000LL;
        return freeMB;
    }
    directory_util entries;
    fs_file_t logfp;
    char rootpath[PATH_MAX], workpath[PATH_MAX], logpath[PATH_MAX];
    uint32_t write_error_count = 0;
    bool output_enabled = false;
    static constexpr int32_t MAX_FILE_COUNT = 500;
    static constexpr int32_t MIN_FREE_MB = 512;
};

uint8_t logbuffer[256];

log_manager logman;

LOG_OUTPUT_DEFINE(log_output_microsd, &log_manager::log_output, logbuffer, sizeof logbuffer);

void microsd_put(const struct log_backend *const backend,
                        struct log_msg *msg)
{
    log_msg_get(msg);
    uint32_t flags = LOG_OUTPUT_FLAG_LEVEL |
                     LOG_OUTPUT_FLAG_TIMESTAMP |
                     LOG_OUTPUT_FLAG_FORMAT_TIMESTAMP;
    log_output_ctx_set(&log_output_microsd, &logman);
    log_output_msg_process(&log_output_microsd, msg, flags);
    log_msg_put(msg);
}

void microsd_dropped(const struct log_backend *const backend, uint32_t cnt)
{
    log_output_dropped_process(&log_output_microsd, cnt);
}

void microsd_panic(const struct log_backend *const backend)
{
    log_output_flush(&log_output_microsd);
}

const struct log_backend_api log_backend_microsd_api = {
    .put = microsd_put,
    .dropped = microsd_dropped,
    .panic = microsd_panic,
};

LOG_BACKEND_DEFINE(log_backend_microsd, log_backend_microsd_api, true);

class logger {
public:
    int setup() {
        if (disk_access_init("SD") != 0)
            return -1;
        LOG_INF("SD CARD OK");
        mount.type = FS_FATFS;
        mount.fs_data = &fatfs;
        mount.mnt_point = mnt_point;
        if (fs_mount(&mount) != FR_OK)
            return -1;
        LOG_INF("SD CARD mounted at %s", log_strdup(mnt_point));
        logman.set_rootpath(mnt_point);
        logman.maintain();
        logman.setup_newlog();
        return 0;
    }
    void loop() const {
        k_msleep(1000);
    }
private:
    FATFS fatfs;
    fs_mount_t mount;
    static const char* mnt_point;
};
const char* logger::mnt_point = "/SD:";

LEXX_THREAD_RUNNER_DELAYED(logger, 5000);

}

/* vim: set expandtab shiftwidth=4: */
