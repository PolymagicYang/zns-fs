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
  // std::cout << "Data write " << size << " " << data << std::endl;
  // Move the size of our inode up by the number of bytes in our write
  this->inode->size += size;

  uint8_t total_blocks = std::ceil(size / (float)g_lba_size);

  // Writes the range of blocks to the disk
  bool overwrite = this->inode->inserted;
  uint64_t slba = store_segment_on_disk(size, data, this->allocator, overwrite);
  this->inode->add_segment(slba, total_blocks);
}

void StoFile::read(const size_t size, void *result) {
  std::fflush(stdout);
  size_t current_size = size;
  void *copy = (char *)result;

  for (auto &segment : inode->segments) {
    for (uint8_t i = 0; i < segment.nblocks; i++) {
      size_t segment_size =
          std::min(g_lba_size * segment.nblocks, current_size);
      get_from_disk(segment.start_lba, segment_size, result, this->allocator);

      ((char *)result)[segment_size] = '\0';
      // Move up by the amount we have read
      current_size += segment_size;
      result += segment_size;
    }
  }

  std::cout << "Data read " << size << " " << copy << std::endl;
}

void StoFile::write_to_disk(bool update) { this->inode->write_to_disk(update); }
