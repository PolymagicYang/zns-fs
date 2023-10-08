#ifndef STOSYS_BLOCK_H
#define STOSYS_BLOCK_H
#include <cstdint>
#pragma once

#include <pthread.h>
#include <zns_device.h>

struct WritePointer {
  pthread_rwlock_t wp_lock;
  uint64_t position;
};

class BlockManager {
 public:
  BlockManager(user_zns_device *disk);

  int append(void *buffer, uint32_t size, uint64_t *start_addr);

  int read(uint64_t lba, void *buffer, uint32_t size);

  int write(uint64_t lba, void *buffer, uint32_t size);

  // return the write pointer.
  uint64_t get_current_position();

  int store_block_to_disk(uint64_t lba, void *buffer, uint32_t size);

  int read_block_from_disk(uint64_t lba, void *buffer, uint32_t size);

  int update_current_position(uint64_t addr);

 private:
  // Maintain the last block address.
  WritePointer wp;
  uint32_t lba_size;
  uint32_t imap_size;
  user_zns_device *disk;
  uint64_t capacity;
};

#endif
