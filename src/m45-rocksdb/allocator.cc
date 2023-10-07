#include "allocator.hpp"
#include <cstdint>
#include <cstdio>
#include <pthread.h>
#include <unistd.h>

BlockManager::BlockManager(user_zns_device *disk) {
    this->lba_size = disk->lba_size_bytes;
    // TODO(Zhiyang): Leave some spaces for imap metadata.
    this->imap_size = 0;
    this->disk = disk;
    this->capacity = disk->capacity_bytes;
    this->wp = WritePointer {
        .wp_lock = PTHREAD_RWLOCK_INITIALIZER, 
        .position = imap_size, // leave space for the imap.
    };
    printf("zns size is %d\n", this->disk->lba_size_bytes);
}

int BlockManager::append(void* buffer, uint32_t size, uint64_t *start_addr) {
    // update operation should be atomic.

    pthread_rwlock_wrlock(&this->wp.wp_lock);
    uint64_t wp = this->get_current_position();
    *start_addr = wp;
    if (wp + size >= disk->capacity_bytes) {
        pthread_rwlock_unlock(&this->wp.wp_lock);
        return -1;
    }
    
    this->update_current_position(wp + size);
    pthread_rwlock_unlock(&this->wp.wp_lock);

    int ret = zns_udevice_write(this->disk, wp, buffer, size);
    if (ret != 0) {
        return ret;
    }

    return ret;
}

int BlockManager::read(uint64_t lba, void *buffer, uint32_t size) {
    return zns_udevice_read(this->disk, lba, buffer, size);
}

uint64_t BlockManager::get_current_position() {
    uint64_t ret = this->wp.position;
    return ret;
}

int BlockManager::update_current_position(uint64_t addr) {
    this->wp.position = addr;
}