#include "directory.hpp"
#include "inode.hpp"
#include <cstring>


StoDir::StoDir(char *name, uint32_t inode) {
	this->inode_number = inode;
	size_t length = strlen(name);
	this->namelen = length;
	strncpy(this->name, name, length);
	this->name[this->namelen] = '\0';
	this->records = std::vector<struct ss_dnode_record>();

	StoInode sinode = StoInode(inode, DIRSIZE, name);
	
}

int StoDir::add_entry(const uint16_t inode_number, const uint16_t reclen, const char *name) {
	struct ss_dnode_record drec;
	drec.inum = inode_number;
	drec.reclen = reclen;
	drec.namelen = strlen(name);
	strncpy(drec.name, name, drec.namelen);
	drec.name[drec.namelen] = '\0';
	return 0;
}

struct ss_dnode_record *StoDir::find_entry(const char *name) {
	for (auto &entry : this->records) {
		if (strncmp(entry.name, name, entry.namelen)) {
			return &entry;
		}		
	}
	
	return NULL;
}

int StoDir::remove_entry(const char *name) {
	struct ss_dnode_record *dnode = this->find_entry(name);
	// Set them to skippable values
	// TODO(valentijn): reuse or garbage collect these values
	dnode->inum = 0;
	dnode->namelen = 0;
}


struct ss_inode *find_file(StoDir &directory, std::string name) {
	std::string delimiter = "/";

	auto location = name.find(delimiter);
	auto current = name.substr(0, location);

	struct ss_dnode_record *entry = directory.find_entry(current.c_str());
	
	// Iterate to the next level in our directory hierarchy
	if (location != std::string::npos) { 
		auto next = name.substr(location+1, name.size());
		struct ss_inode *next_dir_inode = get_inode_by_id(entry->inum);
		// StoDir next_dir = load_directory(next_dir_inode);
		std::cout << current << " " << next << std::endl;
	}
	
	return get_inode_by_id(entry->inum);
}

StoDir::~StoDir() {
	
}



