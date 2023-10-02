#ifndef STOSYS_PROJECT_STRUCTURES_H
#define STOSYS_PROJECT_STRUCTURES_H
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <map>
#include <vector>
#include <pthread.h>
#include <ctime>
#include <chrono>


#define SEGMENT_SIZE 16
#define NAMELEN 256
#define DIRSIZE 32

struct ss_segment {
    uint64_t start_lba;
    size_t nblocks;
};

    
struct ss_inode {
	uint32_t id;
    uint16_t mode;
    uint16_t uuid;
    uint32_t size;
	uint64_t time;
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
	char dirname[DIRSIZE];
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
#endif

