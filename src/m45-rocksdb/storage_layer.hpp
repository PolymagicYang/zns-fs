#ifndef STOSYS_PROJECT_STORAGE_H
#define STOSYS_PROJECT_STORAGE_H

#include "inode.hpp"
#include "structures.h"

struct ss_data *get_from_disk(const uint64_t lba);
void store_block_on_disk(const uint64_t lba, const struct ss_data *data);
uint64_t store_segment_on_disk(const uint8_t nblocks,
                               const struct ss_data *data);

// Maps an SLBA to a block of data of size LBA
extern std::map<uint64_t, struct ss_data> fake_data_storage;
#endif
