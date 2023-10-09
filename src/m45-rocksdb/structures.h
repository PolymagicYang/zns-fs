#ifndef STOSYS_PROJECT_STRUCTURES_H
#define STOSYS_PROJECT_STRUCTURES_H
#include <cstdint>
#pragma once

#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <chrono>
#include <ctime>
#include <map>
#include <vector>

#define SEGMENT_SIZE 500 
#define NAMELEN 256 
#define DIRSIZE 32
#define TEST_LBA_SIZE 4096

// 4th bit
#define FLAG_DIRECTORY (1 << 4)
#define FLAG_LOCK (1 << 5)
extern uint64_t g_lba_size;

struct ss_segment {
  uint64_t start_lba = 0;
  size_t nblocks = 0;
};

struct ss_inode {
  uint32_t id;
  uint16_t mode;
  uint16_t uuid;
  uint32_t size;
  uint64_t time;
  uint64_t inserted;
  bool deleted;
  struct ss_segment segments[SEGMENT_SIZE];
  uint32_t flags;
  uint16_t strlen;
  char name[NAMELEN];  
};

struct ss_dnode_record {
  uint16_t inum;
  uint16_t reclen;
  uint16_t namelen;
  char name[NAMELEN];
};

struct ss_dnode {
  struct ss_dnode_record entries[DIRSIZE];
  uint16_t strlen;
  char dirname[NAMELEN];
};

using __inode_map = std::map<uint64_t, struct ss_inode>;
using __dir_map = std::map<uint64_t, struct ss_dnode>;

struct InodeMap {
  pthread_rwlock_t lock;
  __inode_map inodes;
};

struct DirMap {
  pthread_rwlock_t lock;
  __dir_map dnodes;
};

struct ss_data {
  char data[TEST_LBA_SIZE];
};

#endif
