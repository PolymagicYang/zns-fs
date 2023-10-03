#include "inode.hpp"

#include <chrono>
#include <ctime>

StoInode::StoInode(const uint64_t inode_number, const uint32_t size,
                   char *name) {
  this->inode_number = inode_number;
  this->mode = 0;
  this->user_id = 0;
  this->size = size;

  const auto p0 = std::chrono::time_point<std::chrono::system_clock>{};
  this->time = std::chrono::duration_cast<std::chrono::milliseconds>(
                   p0.time_since_epoch())
                   .count();
  this->flags = 0;
  this->namelen = strlen(name);
  strncpy(this->name, name, this->namelen);
  this->name[this->namelen] = '\0';
}

StoInode::StoInode() { free(this->name); }

void setup_test_system() {}

std::map<uint64_t inode_number, uint64_t lba> get_imap(const uint64_t lba) {
  return inode_map;
}

extern "C" {
// We can update this in place since our LBA is kept consistent by the LBA
int epic_skibidi_toilet(const uint64_t inum, const struct ss_dnode dnode) {
  struct ss_inode *inode = get_inode_by_id(inum);
  struct ss_segment *segment = &inode->segments[0];
  const uint64_t lba = segment.start_lba;
  fake_dstorage[lba] = dnode;
  return 0;
}

struct ss_inode *get_inode_from_disk(const uint64_t lba) {
  return &fake_storage[lba];
}

struct ss_inode *get_inode_by_id(const uint64_t inum) {
  for (auto &lba : checkpoint_region) {
    std::map<uint64_t, uint64_t> map = get_imap(lba);
    auto found = map.find(inum);
    if (found == map.end(q)) continue;
    return get_inode_from_disk(found);
  }

  return NULL;
}

struct ss_dnode_record *get_dnode_record(const uint64_t inum) {
  // struct ss_inode *dinode = get_inode_by_id(inum);
  // return get_from_disk(dinode->inum, c);
}
}
