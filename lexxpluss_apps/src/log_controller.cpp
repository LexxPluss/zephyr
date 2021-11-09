#include <zephyr.h>
#include <disk/disk_access.h>
#include <fs/fs.h>
#include <ff.h>
#include <cstdio>
#include <cstring>
#include "log_controller.hpp"

k_msgq msgq_log;

namespace {

char __aligned(4) msgq_log_buffer[8 * sizeof (msg_log)];

class directory_list {
public:
    enum class ORDER {ASCENT, DESCENT};
    directory_list() {
        reset();
    }
    uint32_t list(const char *path, ORDER order = ORDER::ASCENT) {
        uint32_t count{0};
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
    const char *operator[](int index) const {
        return entries[index];
    }
    static constexpr uint32_t MAX_ENTRIES{30};
private:
    void reset() {
        for (auto &i : entries)
            i[0] = '\0';
    }
    void push(ORDER order, const char* str) {
        for (uint32_t i{0}; i < MAX_ENTRIES; ++i) {
            if (entries[i][0] == 0) {
                stor(entries[i], str);
                break;
            } else {
                int comp{strcmp(str, entries[i])};
                if ((order == ORDER::ASCENT && comp < 0) ||
                    (order == ORDER::DESCENT && comp > 0)) {
                    for (uint32_t j{MAX_ENTRIES - 1}; j > i; --j) {
                        if (entries[j - 1][0] != 0)
                            stor(entries[j], entries[j - 1]);
                    }
                    stor(entries[i], str);
                    break;
                }
            }
        }
    }
    void stor(char *to, const char *from) const {
        snprintf(to, MAX_FNAME, "%s", from);
    }
    fs_dirent dirent;
    static constexpr uint32_t MAX_FNAME{MAX_FILE_NAME + 1};
    char entries[MAX_ENTRIES][MAX_FNAME];
};

class log_util {
public:
    void maintain_log_area(const char *root) {
        reduce_file_count(root);
        reduce_disk_volume(root);
    }
private:
    void reduce_file_count(const char *root) {
        snprintf(workpath, sizeof workpath, "%s/log", root);
        uint32_t count{list.list(workpath)};
        if (count > MAX_FILE_COUNT) {
            uint32_t n{count - MAX_FILE_COUNT};
            if (n > list.MAX_ENTRIES)
                n = list.MAX_ENTRIES;
            for (uint32_t i{0}; i < n; ++i) {
                const char *name{list[i]};
                if (name[0] != '\0') {
                    snprintf(workpath, sizeof workpath, "%s/log/%s", root, name);
                    fs_unlink(workpath);
                }
            }
        }
    }
    void reduce_disk_volume(const char *root) {
        int32_t freeMB{get_freeMB(root)};
        if (freeMB > 0 && freeMB < MIN_FREE_MB) {
            list.list(workpath);
            for (uint32_t i{0}; i < list.MAX_ENTRIES; ++i) {
                const char *name{list[i]};
                if (name[0] != '\0') {
                    snprintf(workpath, sizeof workpath, "%s/log/%s", root, name);
                    fs_unlink(workpath);
                    if (get_freeMB(root) >= MIN_FREE_MB)
                        break;
                }
            }
        }
    }
    int32_t get_freeMB(const char *root) const {
        int32_t freeMB{-1};
        struct fs_statvfs statvfs;
        if (fs_statvfs(root, &statvfs) == 0)
            freeMB = static_cast<int64_t>(statvfs.f_frsize) * static_cast<int64_t>(statvfs.f_bfree) / 1000000LL;
        return freeMB;
    }
    directory_list list;
    char workpath[PATH_MAX];
    static constexpr int32_t MIN_FREE_MB{1024};
    static constexpr uint32_t MAX_FILE_COUNT{500};
};

class log_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq_log, msgq_log_buffer, sizeof (msg_log), 8);
        return 0;
    }
    void run() {
        if (disk_access_init("SD") == 0) {
            printk("SD OK\n");
            mount.type = FS_FATFS;
            mount.fs_data = &fatfs;
            mount.mnt_point = sdroot;
            if (fs_mount(&mount) == 0) {
                util.maintain_log_area(sdroot);
                fs_ok = true;
            }
        }
        if (!fs_ok)
            return;
        while (true) {
            k_msleep(1000);
        }
    }
private:
    FATFS fatfs;
    fs_mount_t mount;
    log_util util;
    bool fs_ok{false};
    static const char *sdroot;
} impl;
const char *log_controller_impl::sdroot{"/SD:"};

}

void log_controller::init()
{
    impl.init();
}

void log_controller::run(void *p1, void *p2, void *p3)
{
    impl.run();
}

k_thread log_controller::thread;

// vim: set expandtab shiftwidth=4:
