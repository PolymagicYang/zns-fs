#ifndef STOSYS_PROJECT_STORAGE_H
#define STOSYS_PROJECT_STORAGE_H

#include "allocator.hpp"
#include "inode.hpp"
#include "structures.h"

void get_from_disk(const uint64_t lba, const size_t size, void *data,
                   BlockManager *allocator);

uint64_t store_segment_on_disk(const size_t size, void *data,
                               BlockManager *allocator, bool overwrite);

#endif
