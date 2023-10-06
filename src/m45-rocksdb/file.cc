#include "file.hpp"

#include <cmath>
#include <cstring>

#include "storage_layer.hpp"
#include "structures.h"

#define Min(x, y) ((x) > (y) ? (y) : (x))

StoFile::StoFile(const ss_inode *inode) {
  // TODO(valentijn): memory leak or something
  this->inode = new StoInode(inode);
}

StoFile::StoFile(const StoInode *inode) { this->inode = (StoInode *)inode; }

StoFile::~StoFile() {}

void StoFile::write(size_t size, void *data) {
  uint8_t total_blocks = std::ceil(size / (float)g_lba_size);
  struct ss_data *barray =
      (struct ss_data *)malloc(sizeof(struct ss_data) * total_blocks);

  for (uint8_t i = 0; i < total_blocks; i++) {
    struct ss_data *sdata = &barray[i];
    memcpy(sdata->data, data, Min(size, TEST_LBA_SIZE));

    // Move our data pointer one block ahead and adjust our size
    data += TEST_LBA_SIZE;
    size -= TEST_LBA_SIZE;
  }

  // Writes the range of blocks to the disk
  uint64_t slba = store_segment_on_disk(total_blocks, barray);
  this->inode->add_segment(slba, total_blocks);
  free(barray);
}

void StoFile::read(const size_t size, void *result) {
  size_t index = 0;
  for (auto &segment : inode->segments) {
    for (uint8_t i = 0; i < segment.nblocks; i++) {
      struct ss_data *data =
          (struct ss_data *)get_from_disk(segment.start_lba + i);
      memcpy(result, data, std::min(size, g_lba_size));
      result += g_lba_size;
    }
  }
}

void StoFile::write_to_disk() { this->inode->write_to_disk(); }
