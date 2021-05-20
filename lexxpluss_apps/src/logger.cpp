#include <zephyr.h>
#include <disk/disk_access.h>
#include <fs/fs.h>
#include <ff.h>
#include "thread_runner.hpp"

namespace {

class storage_manager {
public:
    void maintain_free_space(const char* path) {
        uint32_t freebytes = get_freebytes(path);
        if (freebytes > 0 && freebytes < MIN_FREE_BYTES) {
        }
    }
private:
    int64_t get_freebytes(const char* path) {
        int64_t freebytes = -1;
        struct fs_statvfs statvfs;
        if (fs_statvfs(path, &statvfs) == 0)
            freebytes = static_cast<int64_t>(statvfs.f_frsize) * static_cast<int64_t>(statvfs.f_bfree);
        return freebytes;
    }
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
        man.maintain_free_space(mnt_point);
        return 0;
    }
    void loop() const {
        k_msleep(1000);
    }
private:
    storage_manager man;
    FATFS fatfs;
    fs_mount_t mount;
    static const char* mnt_point;
};
const char* logger::mnt_point = "/SD:";

LEXX_THREAD_RUNNER(logger);

}

/* vim: set expandtab shiftwidth=4: */
