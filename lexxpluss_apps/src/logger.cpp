#include <zephyr.h>
#include <disk/disk_access.h>
#include <fs/fs.h>
#include <ff.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "thread_runner.hpp"

namespace {

class sorted_string_array {
public:
    enum class ORDER {ASCENT, DESCENT};
    sorted_string_array() {
        reset(ORDER::ASCENT);
    }
    void reset(ORDER order) {
        this->order = order;
        for (uint32_t i = 0; i < SIZE; ++i)
            array[i][0] = 0;
    }
    void push(const char* str) {
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
    static constexpr uint32_t SIZE = 10;
    char array[SIZE][MAX_FILE_NAME + 1];
private:
    void stor(char* to, const char* from) {
        snprintf(to, MAX_FILE_NAME + 1, "%s", from);
    }
    ORDER order = ORDER::ASCENT;
};

class log_manager {
public:
    void set_rootpath(const char* rootpath) {
        snprintf(this->rootpath, sizeof this->rootpath, "%s", rootpath);
        snprintf(workpath, sizeof workpath, "%s/log", rootpath);
        fs_mkdir(workpath);
        logpath[0] = 0;
    }
    void maintain() {
        uint32_t freebytes = get_freebytes(rootpath);
        if (freebytes > 0 && freebytes < MIN_FREE_BYTES) {
            entries.reset(sorted_string_array::ORDER::ASCENT);
            fs_dir_t dir;
            fs_dir_t_init(&dir);
            snprintf(workpath, sizeof workpath, "%s/log", rootpath);
            if (fs_opendir(&dir, workpath) == 0) {
                while (fs_readdir(&dir, &dirent) == 0 && dirent.name[0] != 0) {
                    if (dirent.type == FS_DIR_ENTRY_FILE)
                        entries.push(dirent.name);
                }
                fs_closedir(&dir);
            }
            for (uint32_t i = 0; i < entries.SIZE; ++i) {
                if (entries.array[i][0] != 0) {
                    fs_unlink(entries.array[i]);
                    if (get_freebytes(rootpath) >= MIN_FREE_BYTES)
                        break;
                }
            }
        }
    }
    void setup_newlog() {
        entries.reset(sorted_string_array::ORDER::DESCENT);
        fs_dir_t dir;
        fs_dir_t_init(&dir);
        snprintf(workpath, sizeof workpath, "%s/log", rootpath);
        if (fs_opendir(&dir, workpath) == 0) {
            while (fs_readdir(&dir, &dirent) == 0 && dirent.name[0] != 0) {
                if (dirent.type == FS_DIR_ENTRY_FILE)
                    entries.push(dirent.name);
            }
            fs_closedir(&dir);
        }
        if (entries.array[0][0] == 0) {
            snprintf(logpath, sizeof workpath, "%s/log/log000.log", rootpath);
        } else {
            char num[4]{
                entries.array[0][3],
                entries.array[0][4],
                entries.array[0][5],
                0
            };
            int n = atoi(num);
            snprintf(logpath, sizeof workpath, "%s/log/log%03u.log", rootpath, n + 1);
        }
    }
    void output(const char* message) {
        if (logpath[0] != 0 && message != nullptr && message[0] != 0) {
            fs_file_t fp;
            fs_file_t_init(&fp);
            if (fs_open(&fp, logpath, FS_O_WRITE | FS_O_CREATE | FS_O_APPEND) == 0) {
                fs_write(&fp, message, strlen(message));
                fs_close(&fp);
            }
        }
    }
private:
    int64_t get_freebytes(const char* path) const {
        int64_t freebytes = -1;
        struct fs_statvfs statvfs;
        if (fs_statvfs(path, &statvfs) == 0)
            freebytes = static_cast<int64_t>(statvfs.f_frsize) * static_cast<int64_t>(statvfs.f_bfree);
        return freebytes;
    }
    sorted_string_array entries;
    fs_dirent dirent;
    char rootpath[PATH_MAX], workpath[PATH_MAX], logpath[PATH_MAX];
    static constexpr uint32_t MIN_FREE_BYTES = 512 * 1024 * 1024;
};

class logger {
public:
    int setup() {
        if (disk_access_init("SD") != 0)
            return -1;
        mount.type = FS_FATFS;
        mount.fs_data = &fatfs;
        mount.mnt_point = mnt_point;
        if (fs_mount(&mount) != FR_OK)
            return -1;
        logman.set_rootpath(mnt_point);
        logman.maintain();
        logman.setup_newlog();
        return 0;
    }
    void loop() const {
        k_msleep(1000);
    }
private:
    log_manager logman;
    FATFS fatfs;
    fs_mount_t mount;
    static const char* mnt_point;
};
const char* logger::mnt_point = "/SD:";

LEXX_THREAD_RUNNER(logger);

}

/* vim: set expandtab shiftwidth=4: */
