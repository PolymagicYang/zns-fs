#include "allocator.hpp"
#include "structures.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
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
    
    // zns nvme write one block at a time.
    uint64_t padding_size;
    bool needs_pad;
    if (size % TEST_LBA_SIZE != 0) {
        padding_size = (size / TEST_LBA_SIZE) * (TEST_LBA_SIZE) + TEST_LBA_SIZE;
        needs_pad = true;
    } else {
        padding_size = size;
        needs_pad = false;
    }
    this->update_current_position(wp + padding_size);
    pthread_rwlock_unlock(&this->wp.wp_lock);

    if (needs_pad) {
        char* buf = static_cast<char *>(calloc(1, padding_size));
        memcpy(buf, buffer, size);
        return zns_udevice_write(this->disk, wp, buf, padding_size);
    } else {
        return zns_udevice_write(this->disk, wp, buffer, padding_size);
    }
}

int BlockManager::read(uint64_t lba, void *buffer, uint32_t size) {
    uint64_t padding_size;
    if (size % TEST_LBA_SIZE != 0) {
        padding_size = (size / TEST_LBA_SIZE) * (TEST_LBA_SIZE) + TEST_LBA_SIZE;
    } else {
        padding_size = size;
    }
    char* buf = static_cast<char *>(calloc(1, padding_size));
    int ret = zns_udevice_read(this->disk, lba, buf, padding_size);

    if (ret != 0) {
        printf("error!\n");
    }
    
    memcpy(buffer, buf, size);

    return ret;
}

int BlockManager::write(uint64_t lba, void *buffer, uint32_t size) {
    uint64_t padding_size;
    if (size % TEST_LBA_SIZE != 0) {
        padding_size = (size / TEST_LBA_SIZE) * (TEST_LBA_SIZE) + TEST_LBA_SIZE;
        char* buf = static_cast<char *>(calloc(1, padding_size));
        
        memcpy(buf, buffer, size);

        return zns_udevice_write(this->disk, lba, buf, padding_size);
    } else {
        padding_size = size;
        return zns_udevice_write(this->disk, lba, buffer, padding_size);
    }
}

uint64_t BlockManager::get_current_position() {
    uint64_t ret = this->wp.position;
    return ret;
}

int BlockManager::update_current_position(uint64_t addr) {
    this->wp.position = addr;
}