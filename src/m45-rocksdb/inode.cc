#include "inode.hpp"

#include <pthread.h>

#include <cassert>
#include <chrono>
#include <cstring>
#include <ctime>
#include <iostream>
#include <random>

#include "allocator.hpp"
#include "structures.h"

// This number is dependent on SEGMENT_SIZE, please recalculate if you
// change that.
#define LBA_SIZE_DIFF 8192

// Inode map that keeps track of where the inodes are in disk (43.5 OSTEP)
// It maps the inode with the physical logical block address
std::unordered_map<uint64_t, uint64_t> inode_map =
    std::unordered_map<uint64_t, uint64_t>();
pthread_mutex_t inode_map_lock = PTHREAD_MUTEX_INITIALIZER;

// Vector containing the physical address of each of the inode maps
std::vector<uint64_t> checkpoint_region = std::vector<uint64_t>();

std::unordered_map<uint64_t, StoInode *> inode_cache =
    std::unordered_map<uint64_t, StoInode *>();
std::unordered_map<uint64_t, StoDir *> dir_cache =
    std::unordered_map<uint64_t, StoDir *>();

StoInode::StoInode(const uint32_t size, std::string name,
                   BlockManager *allocator) {
  this->inode_number = this->inode.id = g_inode_num++;
  this->mode = this->inode.mode = 0;
  this->user_id = this->inode.uuid = 0;
  this->inserted = false;
  this->dirty = true;
  this->size = this->inode.size = size;

  const auto p0 = std::chrono::time_point<std::chrono::system_clock>{};
  this->time = this->inode.time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          p0.time_since_epoch())
          .count();
  this->flags = this->inode.flags = 0;
  this->namelen = this->inode.strlen = name.size();
  this->name = name;
  this->allocator = allocator;
  strncpy(this->inode.name, name.c_str(), this->namelen);
}

StoInode::StoInode(const struct ss_inode *inode, BlockManager *allocator) {
  this->inode_number = inode->id;
  this->mode = inode->mode;
  this->user_id = inode->uuid;
  this->size = inode->size;
  this->time = inode->time;
  this->deleted = inode->deleted;
  this->flags = inode->flags;
  this->namelen = inode->strlen;
  this->inserted = inode->inserted;
  this->allocator = allocator;
  strncpy((char *)this->name.c_str(), inode->name, inode->strlen);
  this->name[this->namelen] = '\0';
  this->dirty = false;
  this->inode = *inode;
}

struct ss_inode *StoInode::get_inode_struct() {
  return &this->inode;
}

void StoInode::write_to_disk(bool update) {
  // If we never change the inode then we don't actually need to write
  // it to disk
  if (!this->dirty) {
    return;
  }

  // Enter the data into the system maps so we know where to find it.
  struct ss_inode *inode = this->get_inode_struct();
  uint64_t lba;

  inode->inserted = true;
  this->allocator->append((void *)inode, sizeof(struct ss_inode), &lba, true);
  pthread_mutex_lock(&inode_map_lock);
  inode_map[this->inode_number] = lba;
  pthread_mutex_unlock(&inode_map_lock);
  this->dirty = false;
}

static uint64_t counter = 0;
#define Round_down(n, m) (n - (n % m))

void StoInode::add_segment(const uint64_t lba, const size_t nblocks) {
  this->dirty = true;

  if (this->segment_index != 0) {
    struct ss_segment *old = &this->inode.segments[this->segment_index - 1];
    uint64_t slba = Round_down(lba, g_lba_size);
    // Calculate the last written LBA by adding the difference between
    // LBA numbers together with the number of blocks. We can save two
    // subtractions if we are so inclined.
    uint64_t last_lba = Round_down(old->start_lba, g_lba_size) +
                        (g_lba_size * (old->nblocks - 1));
    if (last_lba == slba) {
      this->dirty = false;
      return;
    } else if ((last_lba + g_lba_size) == slba) {
      this->inode.segments[this->segment_index - 1] = {
          .start_lba = old->start_lba, .nblocks = old->nblocks + 1};
      return;
    }
  }

  this->inode.segments[this->segment_index++] = {.start_lba = lba,
                                                 .nblocks = nblocks};
}
StoInode::~StoInode() {
  // free(this->name);
}

void setup_test_system() {
  // This value is ignored
  checkpoint_region.push_back(0xDEADBEEF);
}

std::unordered_map<uint64_t, uint64_t> get_imap(const uint64_t lba) {
  return inode_map;
}

struct ss_inode *get_inode_from_disk(const uint64_t lba,
                                     BlockManager *allocator) {
  struct ss_inode *buffer = (struct ss_inode *)malloc(sizeof(struct ss_inode));
  int ret = allocator->read(lba, buffer, sizeof(struct ss_inode));

  assert(ret == 0);

  return buffer;
}

void update_dnode_in_storage(const uint64_t inum, const struct ss_dnode *dnode,
                             BlockManager *allocator) {
  struct ss_inode *inode = get_inode_by_id(inum, allocator);
  if (inode == NULL) {
    std::cerr << "Cannot find inode " << inum << std::endl;
    return;
  }

  struct ss_segment *segment = &inode->segments[0];
  const uint64_t lba = segment->start_lba;

  int ret = allocator->write(lba, (void *)dnode, sizeof(struct ss_dnode));

  assert(ret == 0);
}

uint64_t add_dnode_to_storage(const uint64_t inum,
                              const struct ss_dnode *drecord,
                              BlockManager *allocator) {
  uint64_t lba;
  allocator->append((void *)drecord, sizeof(struct ss_dnode), &lba, true);
  pthread_mutex_lock(&inode_map_lock);
  inode_map[inum] = lba;
  pthread_mutex_unlock(&inode_map_lock);
  return lba;
}

StoDir *get_directory_by_id(const uint64_t inum, BlockManager *allocator) {
  // If our cache is totally empty, we must be in a new file system
  // and we should recreate the root directory
  if (dir_cache.size() == 0) {
    StoDir *root = new StoDir((char *)"/", 2, allocator);
    root->write_to_disk();
    dir_cache[root->inode_number] = root;
  }

  if (dir_cache.count(inum) == 1) {
    return dir_cache[inum];
  }
  std::cout << "new"
            << " " << inum << std::endl;
  struct ss_inode *inode = get_inode_by_id(inum, allocator);

  // Return NULL if the inode does not contain a directory
  if (!(inode->flags & FLAG_DIRECTORY)) {
    return NULL;
  }

  // A directory can only contain one block
  struct ss_segment *segment = &inode->segments[0];

  // Fetch the start ID from memory
  void *buffer = malloc(sizeof(struct ss_dnode));

  allocator->read(segment->start_lba, buffer, sizeof(struct ss_dnode));
  struct ss_dnode *dnode = (struct ss_dnode *)buffer;

  dir_cache[inum] = new StoDir(inum, dnode, allocator);
  dir_cache[inum]->dnode = *dnode;
  return dir_cache[inum];
}

struct ss_dnode *get_dnode_by_id(const uint64_t inum, BlockManager *allocator) {
  return &get_directory_by_id(inum, allocator)->dnode;
}

struct ss_inode *get_inode_by_id(const uint64_t inum, BlockManager *allocator) {
  return &get_stoinode_by_id(inum, allocator)->inode;
}

static uint16_t count_inodes = 0;
StoInode *get_stoinode_by_id(const uint64_t inum, BlockManager *allocator) {
  if (inode_cache.count(inum) == 1) {
    return inode_cache[inum];
  }

  for (auto &lba : checkpoint_region) {
    pthread_mutex_lock(&inode_map_lock);
    std::unordered_map<uint64_t, uint64_t> map = get_imap(lba);

    auto found = map.find(inum);
    if (found == map.end()) {
      pthread_mutex_unlock(&inode_map_lock);
      continue;
    }

    pthread_mutex_unlock(&inode_map_lock);
    struct ss_inode *ret = get_inode_from_disk(found->second, allocator);
    inode_cache[inum] = new StoInode(ret, allocator);
    ret->segments;
    return inode_cache[inum];
  }

  return NULL;
}
