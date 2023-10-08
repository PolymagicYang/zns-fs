#include "inode.hpp"

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
std::map<uint64_t, uint64_t> inode_map = std::map<uint64_t, uint64_t>();

// Vector containing the physical address of each of the inode maps
std::vector<uint64_t> checkpoint_region = std::vector<uint64_t>();

StoInode::StoInode(const uint32_t size, std::string name,
                   BlockManager *allocator) {
  this->inode_number = g_inode_num++;
  this->mode = 0;
  this->user_id = 0;
  this->size = size;

  const auto p0 = std::chrono::time_point<std::chrono::system_clock>{};
  this->time = std::chrono::duration_cast<std::chrono::milliseconds>(
                   p0.time_since_epoch())
                   .count();
  this->flags = 0;
  this->namelen = name.size();
  this->name = name;
  this->allocator = allocator;
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
  this->allocator = allocator;
  strncpy((char *)this->name.c_str(), inode->name, inode->strlen);
  this->name[this->namelen] = '\0';
  std::copy(std::begin(inode->segments), std::end(inode->segments),
            std::begin(this->segments));
}
struct ss_inode StoInode::get_inode_struct() {
  struct ss_inode inode;
  inode.id = this->inode_number;
  inode.mode = this->mode;
  inode.uuid = this->user_id;
  inode.size = this->size;
  inode.time = this->time;
  inode.deleted = false;
  inode.flags = this->flags;
  inode.strlen = this->namelen;
  this->name = std::string(inode.name);
  std::copy(std::begin(this->segments), std::end(this->segments),
            std::begin(inode.segments));
  return inode;
}

void StoInode::write_to_disk() {
  // Enter the data into the system maps so we know where to find it.
  struct ss_inode inode = this->get_inode_struct();
  uint64_t lba;

  this->allocator->append((void *)&inode, sizeof(struct ss_inode), &lba);
  inode_map[this->inode_number] = lba;
}

static uint64_t counter = 0;
void StoInode::add_segment(const uint64_t lba, const size_t nblocks) {
  struct ss_segment *old = &this->segments[this->segment_index - 1];

  // Calculate the last written LBA by adding the difference between
  // LBA numbers together with the number of blocks. We can save two
  // subtractions if we are so inclined.
  uint64_t last_lba = old->start_lba + (LBA_SIZE_DIFF * (old->nblocks - 1));
  if ((lba - last_lba) == LBA_SIZE_DIFF) {
    std::cout << "Append data " << counter++ << std::endl;
    this->segments[this->segment_index - 1] = {.start_lba = old->start_lba,
                                               .nblocks = old->nblocks + 1};
    return;
  }
  this->segments[this->segment_index++] = {.start_lba = lba,
                                           .nblocks = nblocks};
}
StoInode::~StoInode() {
  // free(this->name);
}

void setup_test_system() {
  // This value is ignored
  checkpoint_region.push_back(0xDEADBEEF);
}

std::map<uint64_t, uint64_t> get_imap(const uint64_t lba) { return inode_map; }

struct ss_inode *get_inode_from_disk(const uint64_t lba,
                                     BlockManager *allocator) {
  struct ss_inode *buffer = (struct ss_inode *)malloc(sizeof(struct ss_inode));
  int ret = allocator->read(lba, buffer, sizeof(struct ss_inode));

  assert(ret == 0);

  return buffer;
}

void update_dnode_in_storage(const uint64_t inum, const struct ss_dnode dnode,
                             BlockManager *allocator) {
  struct ss_inode *inode = get_inode_by_id(inum, allocator);
  if (inode == NULL) {
    std::cerr << "Cannot find inode " << inum << std::endl;
    return;
  }

  struct ss_segment *segment = &inode->segments[0];
  const uint64_t lba = segment->start_lba;

  int ret = allocator->write(lba, (void *)&dnode, sizeof(struct ss_dnode));

  assert(ret == 0);
}

uint64_t add_dnode_to_storage(const uint64_t inum,
                              const struct ss_dnode drecord,
                              BlockManager *allocator) {
  uint64_t lba;
  allocator->append((void *)&drecord, sizeof(struct ss_dnode), &lba);
  inode_map[inum] = lba;
  return lba;
}

struct ss_dnode *get_dnode_by_id(const uint64_t inum, BlockManager *allocator) {
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
  return (struct ss_dnode *)buffer;
}

struct ss_inode *get_inode_by_id(const uint64_t inum, BlockManager *allocator) {
  for (auto &lba : checkpoint_region) {
    std::map<uint64_t, uint64_t> map = get_imap(lba);
    auto found = map.find(inum);
    if (found == map.end()) continue;
    struct ss_inode *ret = get_inode_from_disk(found->second, allocator);

    ret->segments;
    return ret;
  }

  return NULL;
}
