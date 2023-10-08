#ifndef STOSYS_PROJECT_STORAGE_H
#define STOSYS_PROJECT_STORAGE_H

#include "inode.hpp"
#include "structures.h"
#include "allocator.hpp"

struct ss_data get_from_disk(const uint64_t lba, BlockManager *allocator);
uint64_t store_segment_on_disk(const size_t size, void *data, BlockManager *allocator);

#endif
