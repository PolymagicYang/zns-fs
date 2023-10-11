#include "file.hpp"

#include <pthread.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>

#include "allocator.hpp"
#include "structures.h"

#define Min(x, y) ((x) > (y) ? (y) : (x))

StoFile::StoFile(const ss_inode *inode, BlockManager *allocator) {
  // TODO(valentijn): memory leak or something
  this->inode.lock = PTHREAD_MUTEX_INITIALIZER;
  this->inode.node = get_stoinode_by_id(inode->id, allocator);
  this->name = inode->name;
  this->allocator = allocator;
}

StoFile::StoFile(StoInode *inode, BlockManager *allocator) {
  this->inode.lock = PTHREAD_MUTEX_INITIALIZER;
  this->inode.node = inode;
  this->name;
  this->allocator = allocator;
}

StoFile::~StoFile() {}

void StoFile::write(size_t size, void *data) {
  // Move the size of our inode up by the number of bytes in our write
  pthread_mutex_lock(&this->inode.lock);
  this->inode.node->size += size;

  uint8_t total_blocks = std::ceil(size / (float)g_lba_size);
  bool overwrite = this->inode.node->inserted;
  uint64_t slba = store_segment_on_disk(size, data, this->allocator, overwrite);

  printf("add segment\n");
  this->inode.node->add_segment(slba, total_blocks);
  printf("add segment end\n");
  pthread_mutex_unlock(&this->inode.lock);
}

void StoFile::read(const size_t size, void *result) {
  pthread_mutex_lock(&this->inode.lock);
  std::cout << g_magic_offset << std::endl;
  size_t current_size = Min(size, this->inode.node->size - g_magic_offset);
  void *copy = (char *)result;
  for (auto &segment : inode.node->segments) {
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

  pthread_mutex_unlock(&this->inode.lock);
  std::cout << "Data read " << size << " " << copy << std::endl;
}

void StoFile::write_to_disk(bool update) {
  pthread_mutex_lock(&this->inode.lock);
  this->inode.node->write_to_disk(update);
  pthread_mutex_unlock(&this->inode.lock);
}
