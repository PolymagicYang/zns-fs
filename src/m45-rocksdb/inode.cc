#include "inode.hpp"
#include <chrono>
#include <ctime>
#include <cstring>
#include <random>

std::random_device rd;
std::mt19937 rng(rd());
std::uniform_int_distribution<uint64_t> uni(1 << 4, 1 << 12);

// Fake in-memory map of the inodes
std::map<uint64_t, struct ss_inode> fake_storage = std::map<uint64_t, struct ss_inode>();

// Inode map that keeps track of where the inodes are in disk (43.5 OSTEP)
// It maps the inode with the physical logical block address
std::map<uint64_t, uint64_t> inode_map = std::map< uint64_t, uint64_t>();

// Vector containing the physical address of each of the inode maps
std::vector<uint64_t> checkpoint_region = std::vector<uint64_t>();

StoInode::StoInode(const uint64_t inode_id, const uint32_t size, char *name) {
	this->inode_id = inode_id;
	this->mode = 0;
	this->user_id = 0;
	this->size = size;
	
	const auto p0 = std::chrono::time_point<std::chrono::system_clock>{};
	this->time = std::chrono::duration_cast<std::chrono::milliseconds>(p0.time_since_epoch()).count();
	this->flags = 0;
	this->namelen = strlen(name);
	strncpy(this->name, name, this->namelen);
	this->name[this->namelen] = '\0';

	struct ss_inode inode;
	
	inode.id = this->inode_id;
	inode.mode = this->mode;
	inode.uuid = this->user_id;
	inode.size = this->size;
	inode.time = this->time;
	inode.deleted = false;
	inode.flags = this->flags;
	inode.strlen = this->namelen;
	strncpy(inode.name, this->name, this->namelen);
	inode.name[this->namelen] = '\0';
	uint64_t lba = static_cast<uint64_t>(uni(rng));
	fake_storage[lba] = inode;
	inode_map[this->inode_id] = lba;
}

StoInode::~StoInode() {
	free(this->name);
}

void setup_test_system() {
	
}

std::map<uint64_t, uint64_t> get_imap(const uint64_t lba) {
	return inode_map;
}

struct ss_inode *get_inode_from_disk(const uint64_t lba) {
	return &fake_storage[lba];
}

struct ss_inode *get_inode_by_id(const uint64_t inum) {
	for (auto &lba : checkpoint_region) {
		std::map<uint64_t, uint64_t> map = get_imap(lba);
		auto found = map.find(inum);
		if (found == map.end())
			continue;
		return get_inode_from_disk(found->second);
	}

	return NULL;
}

struct ss_dnode_record *get_dnode_record(const uint64_t inum) {
	//struct ss_inode *dinode = get_inode_by_id(inum);
	//return get_from_disk(dinode->inum, c);
}
