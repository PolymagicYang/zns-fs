// #include "storage_layer.hpp"
#include "allocator.hpp"
#include "structures.h"

#include <cassert>
#include <iostream>
#include <sys/ioctl.h>

uint64_t store_segment_on_disk(const uint8_t nblocks,
                               const struct ss_data *data, BlockManager *allocator) {
  uint64_t lba;
  int ret = allocator->append((void *) data,  nblocks * sizeof(struct ss_data), &lba);

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
