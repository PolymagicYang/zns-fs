#pragma once
#ifndef STOSYS_PROJECT_INODE_H
#define STOSYS_PROJECT_INODE_H

#include "structures.h"

// Some hardcoded constant, it will still take 1 block of space
// inside of the storage system itself though.
#define MAP_SIZE 1024

class StoInode {
		public:
	StoInode(const uint64_t inode_id, const uint32_t size, char *name);
	~StoInode();

	uint64_t inode_id;
	uint16_t mode;
	uint16_t user_id;
	uint32_t size;
	uint64_t time;
	bool deleted;

	std::vector<struct ss_segment> segments;
	uint32_t flags;
	uint16_t namelen;
	char name[NAMELEN];
};


void setup_test_system();

extern std::map<uint64_t, struct ss_inode> fake_storage;
extern std::map<uint64_t, uint64_t> inode_map;
extern std::vector<uint64_t> checkpoint_region;

struct ss_inode *get_inode_by_id(const uint64_t inum);
struct ss_dnode_record *get_dnode_record(const uint64_t inum);

#endif

