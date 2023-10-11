#include "allocator.hpp"

#include <pthread.h>
#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>

#include "structures.h"

BlockManager::BlockManager(user_zns_device *disk) {
  this->lba_size = disk->lba_size_bytes;
  // TODO(Zhiyang): Leave some spaces for imap metadata.
  this->imap_size = 0;
  this->disk = disk;
  this->capacity = disk->capacity_bytes;
  this->wp = WritePointer{
      .wp_lock = PTHREAD_RWLOCK_INITIALIZER,
      .position = imap_size,  // leave space for the imap.
  };
}

int BlockManager::append(void *buffer, uint32_t size, uint64_t *start_addr,
                         bool update) {
  // update operation should be atomic.
  /*
  check wheter the wp is on the block boundary.
  if on the boundary, append directly else
  read the block out then append.

  wp should be the multiple of block size, e.g. 4097 => 4096.
  needs to hold the lock when writing.
  */
  int ret = 0;
  pthread_rwlock_wrlock(&this->wp.wp_lock);
  uint64_t wp = this->get_current_position();
  // printf("append to %ld, size is %d!\n", wp, size);
  uint32_t lba_size = this->disk->lba_size_bytes;
  *start_addr = wp;
  if (wp + size >= disk->capacity_bytes) {
    pthread_rwlock_unlock(&this->wp.wp_lock);
    return -1;
  }

  // zns nvme write one block at a time.
  uint64_t padding_size;

  if (wp % lba_size == 0) {
    if (size % lba_size != 0) {
      padding_size = (size / lba_size) * (lba_size) + lba_size;
      char buf[padding_size];
      printf("padding size is %d, buf size is %d\n", padding_size, size);
      memcpy(buf, buffer, size);
      ret = zns_udevice_write(this->disk, wp, buf, padding_size);
    } else {
      ret = zns_udevice_write(this->disk, wp, buffer, size);
    }
  } else {
    // printf("needs padding\n");
    uint64_t wp_base = (wp / lba_size) * lba_size;
    uint64_t curr_data_size_in_block = wp - wp_base;

    if ((size + curr_data_size_in_block) % lba_size != 0) {
      padding_size =
          ((size + curr_data_size_in_block) / lba_size) * (lba_size) + lba_size;
    } else {
      padding_size = size;
    }
    char curr_block[lba_size];
    char new_blocks[padding_size];
    ret = zns_udevice_read(this->disk, wp_base, curr_block, lba_size);

    memcpy(new_blocks, curr_block, curr_data_size_in_block);
    memcpy(new_blocks + curr_data_size_in_block, buffer, size);
    ret = zns_udevice_write(this->disk, wp_base, new_blocks, padding_size);
    // printf("lba is %lx\n", wp);
    // for (int i = 0; i < size; i++) {
    //     printf("%x", ((char*)buffer)[i]);
    // }
    // printf("end for\n");
  }

  if (update) {
    this->update_current_position(wp + size);
  }
  pthread_rwlock_unlock(&this->wp.wp_lock);
  return ret;
}

int BlockManager::read(uint64_t lba, void *buffer, uint32_t size) {
  // printf("read\n");
  uint32_t lba_size = this->disk->lba_size_bytes;
  uint64_t padding_size;
  uint64_t wp_base = (lba / lba_size) * lba_size;
  uint64_t curr_data_size_in_block = lba - wp_base;
  uint64_t read_size = size + curr_data_size_in_block;
  if (read_size % lba_size != 0) {
    padding_size = (read_size / lba_size) * (lba_size) + lba_size;
  } else {
    padding_size = read_size;
  }
  char *buf = static_cast<char *>(calloc(1, padding_size));
  int ret = zns_udevice_read(this->disk, wp_base, buf, padding_size);
  free(buf);

  if (ret != 0) {
    printf("error!\n");
  }

  // printf("read end\n");
  memcpy(buffer, buf + curr_data_size_in_block, size);

  // printf("lba is %x, read from wp_base %x\n", lba, wp_base);
  // for (int i = 0; i < size; i++) {
  //     printf("%x", ((char*)buffer)[i]);
  // }
  // printf("end for\n");

  return ret;
}

int BlockManager::write(uint64_t lba, void *buffer, uint32_t size) {
  // if no padding needed, only needs before part + buffer part.
  // if address is on the block boundary, no need before part.
  // if no padding and no boundary cross, write directly.
  printf("write\n");
  int ret = 0;
  uint32_t lba_size = this->disk->lba_size_bytes;
  uint64_t padding_size;
  uint64_t wp_base = (lba / lba_size) * lba_size;
  uint64_t curr_data_size_in_block = lba - wp_base;

  bool padding;
  bool cross_bd;
  if (lba % lba_size == 0) {
    cross_bd = false;
  } else {
    cross_bd = true;
  }

  if ((size + curr_data_size_in_block) % lba_size != 0) {
    padding_size =
        ((size + curr_data_size_in_block) / lba_size) * (lba_size) + lba_size;
    padding = true;
  } else {
    padding_size = size;
    padding = false;
  }

  if (!padding & !cross_bd) {
    ret = zns_udevice_write(this->disk, lba, buffer, size);
  }
  if (padding & !cross_bd) {
    char after[lba_size];
    char new_blocks[padding_size];

    uint64_t after_base = ((size + lba) / lba_size) * lba_size;
    uint64_t after_index = size + lba - after_base;
    int ret1 = zns_udevice_read(this->disk, after_base, after, lba_size);

    printf("block size %d, buffer size %d\n", padding_size, size);
    memcpy(new_blocks, buffer, size);
    memcpy(new_blocks + size, after + after_index, padding_size - size);
    int ret2 = zns_udevice_write(this->disk, lba, new_blocks, padding_size);
    ret = ret1 & ret2;
  }
  if (!padding & cross_bd) {
    char before[lba_size];
    char new_blocks[padding_size];

    int ret1 = zns_udevice_read(this->disk, wp_base, before, lba_size);

    memcpy(new_blocks, before, curr_data_size_in_block);
    memcpy(new_blocks + curr_data_size_in_block, buffer, size);
    int ret2 = zns_udevice_write(this->disk, wp_base, new_blocks, padding_size);
    ret = ret1 & ret2;
  }
  if (padding & cross_bd) {
    char before[lba_size];
    char after[lba_size];
    char new_blocks[padding_size];

    uint64_t after_base = ((size + lba) / lba_size) * lba_size;
    uint64_t after_index = size + lba - after_base;
    int ret1 = zns_udevice_read(this->disk, wp_base, before, lba_size);
    int ret2 = zns_udevice_read(this->disk, after_base, after, lba_size);
    ret = ret1 & ret2;

    memcpy(new_blocks, before, curr_data_size_in_block);
    memcpy(new_blocks + curr_data_size_in_block, buffer, size);
    memcpy(new_blocks + curr_data_size_in_block + size, after + after_index,
           padding_size - size - curr_data_size_in_block);
    int ret3 = zns_udevice_write(this->disk, wp_base, new_blocks, padding_size);
    ret = ret & ret3;
  }
  // printf("lba is %x\n", lba);
  // for (int i = 0; i < size; i++) {
  //     printf("%x", ((char*)buffer)[i]);
  // }
  // printf("end for\n");

  return ret;
}

uint64_t BlockManager::get_current_position() {
  uint64_t ret = this->wp.position;
  return ret;
}

int BlockManager::update_current_position(uint64_t addr) {
  this->wp.position = addr;
  return 1;
}
