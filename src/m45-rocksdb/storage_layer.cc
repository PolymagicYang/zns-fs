// #include "storage_layer.hpp"
#include "allocator.hpp"
#include "structures.h"

#include <cassert>
#include <iostream>
#include <sys/ioctl.h>

uint64_t store_segment_on_disk(const size_t size, void *data, BlockManager *allocator) {
  uint64_t lba;
  int ret = allocator->append((void *) data,  size, &lba);

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
