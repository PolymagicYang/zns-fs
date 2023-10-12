#pragma once
#include <pthread.h>

#include "allocator.hpp"
#include "directory.hpp"
#ifndef STOSYS_PROJECT_INODE_H
#define STOSYS_PROJECT_INODE_H
#include <unordered_map>

#include "structures.h"

static uint32_t g_inode_num = 2;

// Our reads need to be offset by this number whenever the
// deallocation is called.  Don't ask.
extern uint32_t g_magic_offset;
extern "C" {
void update_dnode_in_storage(const uint64_t inum, const struct ss_dnode *dnode,
                             BlockManager *);
struct ss_inode *get_inode_by_id(const uint64_t inum, BlockManager *);
struct ss_dnode *get_dnode_by_id(const uint64_t inum, BlockManager *);
uint64_t add_dnode_to_storage(const uint64_t inum,
                              const struct ss_dnode *drecord, BlockManager *);
void setup_test_system();
}

class StoInode {
 public:
  StoInode(const uint32_t size, std::string name, BlockManager *allocator);
  ~StoInode();
  StoInode(const struct ss_inode *inode, BlockManager *allocator);

  uint64_t inode_number;
  uint16_t mode;
  uint16_t user_id;
  uint32_t size;
  uint64_t time;
  bool deleted;
  bool inserted;
  bool dirty;

  uint32_t flags;
  uint16_t namelen;
  std::string name;

  void add_segment(const uint64_t lba, const size_t nblocks);
  struct ss_inode *get_inode_struct();
  void write_to_disk(bool update);
  struct ss_inode inode;

  uint8_t segment_index = 0;
  BlockManager *allocator;
};

StoInode *get_stoinode_by_id(const uint64_t inum, BlockManager *allocator);
StoDir *get_directory_by_id(const uint64_t inum, BlockManager *allocator);

extern std::unordered_map<uint64_t, uint64_t> inode_map;
extern std::unordered_map<uint64_t, StoInode *> inode_cache;
extern std::unordered_map<uint64_t, StoDir *> dir_cache;
extern std::vector<uint64_t> checkpoint_region;

#endif
