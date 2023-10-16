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
  std::string str(inode->name, inode->strlen);
  this->name = str;
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
  printf("inode size is %d before adding\n", this->inode.node->size);
  this->inode.node->size += size;
  printf("inode size is %d after adding\n", this->inode.node->size);
  uint8_t total_blocks = std::ceil(size / (float)g_lba_size);
  bool overwrite = this->inode.node->inserted;
  uint64_t slba = store_segment_on_disk(size, data, this->allocator, overwrite);

  printf("write file, size is %d, inode size is %d: \n", size, this->inode.node->size);
  for (uint32_t i = 0; i < size; i++) {
    printf("%x", ((char*) data)[i]);
  }
  printf("\n");

  // printf("write segment to %lx\n", slba);
  this->inode.node->add_segment(slba, total_blocks);
  this->inode.node->write_to_disk(false);
  pthread_mutex_unlock(&this->inode.lock);
}

void StoFile::read(const size_t size, void *result) {
  pthread_mutex_lock(&this->inode.lock);
  size_t current_size = Min(size, this->inode.node->size);
  void *copy = (char *)result;
  for (auto &segment : inode.node->inode.segments) {
    for (uint8_t i = 0; i < segment.nblocks; i++) {
      size_t segment_size =
          std::min(g_lba_size * segment.nblocks, current_size);

      printf("get segment from %lx\n", segment.start_lba);
      get_from_disk(segment.start_lba, segment_size, result, this->allocator);

      // TODO(everyone): fix; this buffer overflow.
      // ((char *)result)[segment_size] = '\0';
      // Move up by the amount we have read
      current_size += segment_size;
      result = (void *)((uint64_t)result + segment_size);
    }
  }

  pthread_mutex_unlock(&this->inode.lock);
  // std::cout << "Data read " << size << " " << (char*) copy << std::endl;
}

void StoFile::write_to_disk(bool update) {
  pthread_mutex_lock(&this->inode.lock);
  this->inode.node->write_to_disk(update);
  pthread_mutex_unlock(&this->inode.lock);
}
