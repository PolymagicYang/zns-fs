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

#define Round_up(num, mul) ((num + mul - 1) / mul) * mul

#define SEGMENT_SIZE 16
#define NAMELEN 128
#define DIRSIZE 16
#define IMAP_OCCUPY 1 // meta info occupies IMAP_OCCUPY * lba_size space to restore.
#define META_ADDR 0
#define INIT_CODE "123456789123456"
#define INIT_CODE_SIZE 16
/*
meta data layout:

0-15 bytes: verify if the device is initialized. 
16-23 bytes: pointer to the imap (this pointer should as same as the wp, because we'll overwrite this imap).
*/

#define Round_up(num, round) (((num) + (round)-1) / (round)) * (round)
// 4th bit
#define FLAG_DIRECTORY (1 << 4)
#define FLAG_LOCK (1 << 5)
extern uint64_t g_lba_size;
extern char init_code[16];

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
  bool deleted = false;
  struct ss_segment segments[SEGMENT_SIZE];
  uint32_t flags;
  uint16_t strlen;
  char name[NAMELEN];
};

struct ss_dnode_record {
  uint16_t inum = 0;
  uint16_t reclen = 0;
  uint16_t namelen = 0;
  char name[NAMELEN];
};

struct ss_dnode {
  struct ss_dnode_record entries[DIRSIZE];
  uint64_t strlen;
  char dirname[NAMELEN];
};
#endif
