// #include "storage_layer.hpp"
#include <sys/ioctl.h>

#include <cassert>
#include <iostream>

#include "allocator.hpp"
#include "structures.h"

uint64_t store_segment_on_disk(const size_t size, void *data,
                               BlockManager *allocator) {
  uint64_t lba;
  int ret = allocator->append((void *)data, size, &lba);

  // ret == 0 => No space for writing.
  assert(ret == 0);

  return lba;
}

void get_from_disk(const uint64_t lba, const size_t size, void *data,
                   BlockManager *allocator) {
  int ret = allocator->read(lba, data, size);
  assert(ret == 0);
}
