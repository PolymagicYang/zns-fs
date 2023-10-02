#ifndef STOSYS_DIR_H
#define STOSYS_DIR_H
#pragma once
#include "structures.h"
#include <vector>
#include <iostream>

// total size of a directory in memory
#define DIRSIZE 1024

class StoDir {
	public:
	// Records 
	std::vector<struct ss_dnode_record> records;

	// Elements stored on disk
	uint32_t inode_number;
	uint16_t namelen;
	char name[NAMELEN];
			   
	StoDir(char *name, uint32_t inode);
	~StoDir();

	int add_entry(const uint16_t inode_number, const uint16_t reclen,
				  const char *name);
	int remove_entry(const char *name);
	struct ss_dnode_record *find_entry(const char *name);
	
};

struct ss_inode *find_file(StoDir &directory, std::string name);

#endif 
