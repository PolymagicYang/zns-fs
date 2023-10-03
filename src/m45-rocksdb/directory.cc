#include "directory.hpp"

#include <cstring>

#include "inode.hpp"

StoDir::StoDir(char *name) {
  this->inode_number = 0;
  size_t length = strlen(name);
  this->namelen = length;
  strncpy(this->name, name, length);
  this->name[this->namelen] = '\0';
  this->records = std::array<struct ss_dnode_record, DIRSIZE>();
}

struct ss_dnode StoDir::create_dnode() {
  struct ss_dnode node;
  strncpy(node.dirname, this->name, this->namelen);
  node.strlen = this->namelen;
  // Unsafe casting to the 32 entries
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
    const uint64_t lba =
        add_dnode_to_storage(sinode.inode_number, this->create_dnode());
    sinode.add_segment(lba, 1);
    this->inode_number = sinode.inode_number;
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
  for (auto &entry : this->records) {
    // Weird C++ behaviour, we force it to be boolean.
    bool condition = strncmp(entry.name, name, entry.namelen) == 0;

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

struct ss_inode *find_file(StoDir &directory, std::string name) {
  std::string delimiter = "/";

  auto location = name.find(delimiter);
  auto current = name.substr(0, location);

  struct ss_dnode_record *entry = directory.find_entry(current.c_str());

  // Iterate to the next level in our directory hierarchy
  if (location != std::string::npos) {
    auto next = name.substr(location + 1, name.size());
    struct ss_dnode *next_dir_inode = get_dnode_by_id(entry->inum);
    // StoDir next_dir = load_directory(next_dir_inode);
    std::cout << current << " " << next << std::endl;
  }

  return get_inode_by_id(entry->inum);
}

StoDir::~StoDir() {}
