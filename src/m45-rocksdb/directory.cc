#include "directory.hpp"

#include <cstring>

#include "inode.hpp"

StoDir::StoDir(char *name, const uint64_t parent_inode) {
  this->inode_number = 0;
  this->parent_inode = parent_inode;
  size_t length = strlen(name) + 1;
  this->namelen = length;
  strncpy(this->name, name, length);
  this->name[this->namelen] = '\0';
  this->records = std::array<struct ss_dnode_record, DIRSIZE>();
}

StoDir::StoDir(const uint64_t inum, const struct ss_dnode *node) {
  this->inode_number = inum;
  this->namelen = node->strlen;
  strncpy(this->name, node->dirname, node->strlen);
  std::copy(std::begin(node->entries), std::end(node->entries),
            std::begin(this->records));
}

struct ss_dnode StoDir::create_dnode() {
  struct ss_dnode node;
  strncpy(node.dirname, this->name, this->namelen);
  node.strlen = this->namelen;
  // Unsafe casting to the 32 entriesb
  std::copy(std::begin(this->records), std::end(this->records),
            std::begin(node.entries));
  return node;
}

void StoDir::write_to_disk() {
  // If the inode is zero we haven't actually written to disk yet
  // and we need to generate the inode
  if (this->inode_number == 0) {
    // Add our dnode to the system with a fresh inode
    StoInode sinode = StoInode(DIRSIZE, name);
    sinode.flags |= FLAG_DIRECTORY;

    this->inode_number = sinode.inode_number;
    // Add the parent and self referential files to our system
    this->add_entry(this->inode_number, 12, ".");
    this->add_entry(this->parent_inode, 12, "..");

    const uint64_t lba =
        add_dnode_to_storage(this->inode_number, this->create_dnode());

    sinode.add_segment(lba, 1);
    sinode.write_to_disk();
    return;
  }

  // Use our stored
  update_dnode_in_storage(this->inode_number, this->create_dnode());
}

int StoDir::add_entry(const uint16_t inode_number, const uint16_t reclen,
                      const char *name) {
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
  this->records[this->entry_index++] = drec;
  return 0;
}

struct ss_dnode_record *StoDir::find_entry(const char *name) {
  size_t needle_size = strlen(name);
  for (auto &entry : this->records) {
    // Weird C++ behaviour, we force it to be boolean.
    // We want to check whether they have the same length and if the entry
    // is still valid.
    bool condition = needle_size == entry.namelen && entry.reclen != 0 &&
                     (strncmp(entry.name, name, entry.namelen) == 0);

    if (condition) {
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

enum DirectoryError find_inode(StoDir &directory, std::string name,
                               struct ss_inode *found,
                               struct find_inode_callbacks *cbs) {
  std::string delimiter = "/";

  // Split our string until the next delimiter and find it in the
  // current directory
  auto location = name.find(delimiter);
  auto current = name.substr(0, location);
  struct ss_dnode_record *entry = directory.find_entry(current.c_str());

  // Iterate to the next level in our directory hierarchy
  if (location != std::string::npos) {
    auto next = name.substr(location + 1, name.size());
    if (entry == NULL && cbs && cbs->missing_directory_cb) {
      entry = cbs->missing_directory_cb(current.c_str(), directory);
    } else if (entry == NULL) {
      return DirectoryError::Directory_not_found;
    }

    // If there is no dnode found then our storage system fucked up
    // and we can't really fix it anymore
    struct ss_dnode *next_dir_inode = get_dnode_by_id(entry->inum);
    if (next_dir_inode == NULL) {
      return DirectoryError::Dnode_not_found;
    }

    StoDir next_dir = StoDir(entry->inum, next_dir_inode);
    return find_inode(next_dir, next, found, cbs);
  }

  // If we reach the end of the hierarchy and we found something,
  // then we can just return our inode directly, no harm done.
  if (entry != NULL) {
    if (cbs && cbs->found_file_cb) {
      cbs->found_file_cb(current.c_str(), directory,
                         get_inode_by_id(entry->inum));
    }

    *found = *get_inode_by_id(entry->inum);
    return DirectoryError::Found_inode;
  }

  if (cbs && cbs->missing_file_cb) {
    *found = *cbs->missing_file_cb(current.c_str(), directory);
    return DirectoryError::Created_inode;
  }

  return DirectoryError::Inode_not_found;
}

StoDir::~StoDir() {}
