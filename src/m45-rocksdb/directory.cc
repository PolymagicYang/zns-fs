#include "directory.hpp"

#include <cstring>
#include <regex>

#include "allocator.hpp"
#include "inode.hpp"
#include "structures.h"

StoDir::StoDir(char *name, const uint64_t parent_inode,
               BlockManager *allocator) {
  this->inode_number = 0;
  this->parent_inode = parent_inode;
  size_t length = strlen(name) + 1;
  this->namelen = this->dnode.strlen = length;
  strncpy(this->name, name, length);
  this->name[this->namelen] = '\0';
  this->records = std::array<struct ss_dnode_record, DIRSIZE>();
  this->allocator = allocator;
  strncpy(this->dnode.dirname, this->name, length);
}

StoDir::StoDir(const uint64_t inum, const struct ss_dnode *node,
               BlockManager *allocator) {
  this->inode_number = inum;
  this->namelen = node->strlen;
  strncpy(this->name, node->dirname, node->strlen);
  std::copy(std::begin(node->entries), std::end(node->entries),
            std::begin(this->records));
  this->allocator = allocator;
}

struct ss_dnode *StoDir::create_dnode() {
  return &this->dnode;
}

void StoDir::write_to_disk() {
  // If the inode is zero we haven't actually written to disk yet
  // and we need to generate the inode
  if (this->inode_number == 0) {
    // Add our dnode to the system with a fresh inode
    StoInode *sinode = new StoInode(DIRSIZE, name, this->allocator);
    sinode->flags |= FLAG_DIRECTORY;
    sinode->inode.flags |= FLAG_DIRECTORY;

    sinode->dirty = true;

    this->inode_number = sinode->inode_number;
    // Add the parent and self referential files to our system
    this->add_entry(this->inode_number, 12, ".");
    this->add_entry(this->parent_inode, 12, "..");

    const uint64_t lba =
        add_dnode_to_storage(this->inode_number, &this->dnode, this->allocator);

    sinode->add_segment(lba, 1);
    sinode->write_to_disk(true);
	inode_cache_lock.lock();
    inode_cache[sinode->inode_number] = sinode;
	inode_cache_lock.unlock();
    return;
  }

  // Use our stored inode number
  update_dnode_in_storage(this->inode_number, &this->dnode, this->allocator);
}

int StoDir::add_entry(const uint16_t inode_number, const uint16_t reclen,
                      const char *name) {
  // Find an empty spot in our directory structure
  uint16_t index = 0;
  for (auto &entry : this->records) {
    if (entry.inum == 0) break;
    index++;
  }

  struct ss_dnode_record drec;
  if (inode_number == 0) {
    std::cerr << "Invalid inode number '" << inode_number << "' for " << name
              << std::endl;
  }
  drec.inum = inode_number;
  drec.reclen = reclen;
  drec.namelen = strlen(name);
  strncpy(drec.name, name, drec.namelen);
  drec.name[drec.namelen] = '\0';
  this->records[index] = drec;
  this->dnode.entries[index] = drec;
  return 0;
}

struct ss_dnode_record *StoDir::find_entry(const char *name) {
  if (name == NULL || this == NULL) return NULL;

  size_t needle_size = strlen(name);
  printf("needld_size for %s is %d\n", name, needle_size);

  for (auto &entry : this->records) {
    // Weird C++ behaviour, we force it to be boolean.
    // We want to check whether they have the same length and if the entry
    // is still valid.
    // TODO(Zhiyang): Delete Comments.
    bool condition = needle_size == entry.namelen && entry.reclen != 0 &&
                     (strncmp(entry.name, name, entry.namelen) == 0);

    if (condition) {
      printf("found entry in find_entry: %s\n", entry.name);
      return &entry;
    }
  }

  return NULL;
}

int StoDir::remove_entry(const char *name) {
  struct ss_dnode_record *dnode = this->find_entry(name);
  // Set them to skippable values
  // TODO(valentijn): reuse or garbage collect these values
  printf("remove file %s from inum %d\n", name, dnode->inum);
  dnode->inum = 0;
  dnode->namelen = 0;
  dnode->reclen = 0;
  return 1;
}

enum DirectoryError find_inode(StoDir *directory, std::string name,
                               struct ss_inode *found,
                               struct find_inode_callbacks *cbs,
                               BlockManager *allocator) {
  // Find and replace double // with a singular /
  name = std::regex_replace(name, std::regex("//"), "/");

  // Split our string until the next delimiter and find it in the
  // current directory
  std::string delimiter = "/";
  struct ss_dnode_record *entry;

  auto location = name.find(delimiter);
  auto current = name.substr(0, location);
  auto prev = location;

  do {
    prev = location;
    location = name.find(delimiter);
    current = name.substr(0, location);
    printf("find current file name %s\n", current.c_str());
    entry = directory->find_entry(current.c_str());

    // We don't need to iterate if we found the inode
    if (name == current) break;

    // Iterate to the next level in our directory hierarchy
    auto next = name.substr(location + 1, name.size());
    if (entry == NULL && cbs && cbs->missing_directory_cb) {
      entry = cbs->missing_directory_cb(current.c_str(), directory,
                                        cbs->user_data, allocator);
    } else if (entry == NULL) {
      return DirectoryError::Directory_not_found;
    }

    // If there is no dnode found then our storage system fucked up
    // and we can't really fix it anymore
    struct ss_dnode *next_dir_inode = get_dnode_by_id(entry->inum, allocator);
    if (next_dir_inode == NULL) {
      return DirectoryError::Dnode_not_found;
    }

    directory = get_directory_by_id(entry->inum, allocator);
    name = next;
  } while (prev != std::string::npos);

  // If we reach the end of the hierarchy and we found something,
  // then we can just return our inode directly, no harm done.
  if (entry != NULL) {
    if (cbs && cbs->found_file_cb) {
      cbs->found_file_cb(current.c_str(), directory,
                         get_inode_by_id(entry->inum, allocator), entry,
                         cbs->user_data, allocator);
    }

    if (entry->inum != 0) {
      // not deleted.
      *found = *get_inode_by_id(entry->inum, allocator);
    }
    return DirectoryError::Found_inode;
  }

  if (cbs && cbs->missing_file_cb) {
    *found = *cbs->missing_file_cb(current.c_str(), directory, cbs->user_data,
                                   allocator);
    return DirectoryError::Created_inode;
  }

  return DirectoryError::Inode_not_found;
}

StoDir::~StoDir() {}
