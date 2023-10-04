#ifndef STOSYS_PROJECT_FILE_H_
#define STOSYS_PROJECT_FILE_H_
#pragma once

#include "structures.h"
#include "inode.hpp"
#include "storage_layer.hpp"

#include <mutex>

class StoFile {
public:
	char *name;
	StoFile(const ss_inode *inode);
	StoFile(const StoInode *inode);
	~StoFile();
	void write_to_disk();
	void write(size_t size, void *data);
	void read(size_t size, void *result);

private:
	StoInode *inode;
	
};

#endif
