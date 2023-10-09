// #include "storage_layer.hpp"
#include <sys/ioctl.h>

#include <cassert>
#include <iostream>

#include "allocator.hpp"
#include "structures.h"
#include <pthread.h>

uint64_t store_segment_on_disk(const size_t size, void *data,
                               BlockManager *allocator, bool overwrite) {
  uint64_t lba;
  // printf("segment size is %d\n", size);
  if (overwrite) {
    pthread_rwlock_wrlock(&allocator->wp.wp_lock);
    uint64_t inode_addr = allocator->get_current_position();
    allocator->update_current_position(inode_addr - sizeof(struct ss_inode));
    // printf("curr addr is %ld.\n", inode_addr - sizeof(struct ss_inode));
    pthread_rwlock_unlock(&allocator->wp.wp_lock);
  }
  int ret = allocator->append((void *)data, size, &lba, true);

  // ret == 0 => No space for writing.
  assert(ret == 0);

  return lba;
}

struct ss_data get_from_disk(const uint64_t lba, BlockManager *allocator) {
  struct ss_data buffer;
  int ret = allocator->read(lba, &buffer, sizeof(struct ss_data));

  assert(ret == 0);
  return buffer;
}
