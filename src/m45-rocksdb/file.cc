#include "file.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>

#include "allocator.hpp"
#include "structures.h"

#define Min(x, y) ((x) > (y) ? (y) : (x))

StoFile::StoFile(const ss_inode *inode, BlockManager *allocator) {
  // TODO(valentijn): memory leak or something
  this->inode = new StoInode(inode, allocator);
  this->name = inode->name;
  this->allocator = allocator;
}

StoFile::StoFile(StoInode *inode, BlockManager *allocator) {
  this->inode = inode;
  this->name;
  this->allocator = allocator;
}

StoFile::~StoFile() {}

void StoFile::write(size_t size, void *data) {
  std::cout << "Data write " << size << " " << data << std::endl;
  // Move the size of our inode up by the number of bytes in our write
  this->inode->size += size;

  uint8_t total_blocks = std::ceil(size / (float)g_lba_size);

  // Writes the range of blocks to the disk
  uint64_t slba = store_segment_on_disk(size, data, this->allocator);
  this->inode->add_segment(slba, total_blocks);
}

void StoFile::read(const size_t size, void *result) {
  std::fflush(stdout);
  size_t index = 0;
  void *copy = result;

  for (auto &segment : inode->segments) {
    for (uint8_t i = 0; i < segment.nblocks; i++) {
      struct ss_data data =
          (struct ss_data)get_from_disk(segment.start_lba + i, this->allocator);

      // End our data on
      size_t border = std::min(size, g_lba_size);
      data.data[border] = '\0';
      memcpy(result, data.data, border + 1);
      result += g_lba_size;
    }
  }

  std::cout << "Data read " << size << " " << copy << std::endl;
}

void StoFile::write_to_disk() { this->inode->write_to_disk(); }
