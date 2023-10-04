#include "inode.hpp"

#include <chrono>
#include <cstring>
#include <ctime>
#include <iostream>
#include <random>

inline std::random_device rd;
inline std::mt19937 rng(rd());
inline std::uniform_int_distribution<uint64_t> uni(1 << 4, 1 << 12);

// Fake in-memory map of the inodes
std::map<uint64_t, struct ss_inode> fake_storage =
    std::map<uint64_t, struct ss_inode>();
std::map<uint64_t, struct ss_dnode> fake_dstorage =
    std::map<uint64_t, struct ss_dnode>();

// Inode map that keeps track of where the inodes are in disk (43.5 OSTEP)
// It maps the inode with the physical logical block address
std::map<uint64_t, uint64_t> inode_map = std::map<uint64_t, uint64_t>();

// Vector containing the physical address of each of the inode maps
std::vector<uint64_t> checkpoint_region = std::vector<uint64_t>();

StoInode::StoInode(const uint32_t size, char *name) {
  this->inode_number = g_inode_num++;
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

struct ss_inode StoInode::get_inode_struct() {
  struct ss_inode inode;
  inode.id = this->inode_number;
  inode.mode = this->mode;
  inode.uuid = this->user_id;
  inode.size = this->size;
  inode.time = this->time;
  inode.deleted = false;
  inode.flags = this->flags;
  inode.strlen = this->namelen;
  strncpy(inode.name, this->name, this->namelen);
  inode.name[this->namelen] = '\0';
  std::copy(std::begin(this->segments), std::end(this->segments),
            std::begin(inode.segments));
  return inode;
}

void StoInode::write_to_disk() {
  // Enter the data into the system maps so we know where to find it.
  struct ss_inode inode = this->get_inode_struct();
  uint64_t lba = static_cast<uint64_t>(uni(rng));
  fake_storage[lba] = inode;
  inode_map[this->inode_number] = lba;
}

void StoInode::add_segment(const uint64_t lba, const size_t nblocks) {
  this->segments[this->segment_index++] = {.start_lba = lba,
                                           .nblocks = nblocks};
}
StoInode::~StoInode() {
  // free(this->name);
}

void setup_test_system() {
  // This value is ignored
  checkpoint_region.push_back(0xDEADBEEF);
}

std::map<uint64_t, uint64_t> get_imap(const uint64_t lba) { return inode_map; }

struct ss_inode *get_inode_from_disk(const uint64_t lba) {
  return &fake_storage[lba];
}

void update_dnode_in_storage(const uint64_t inum, const struct ss_dnode dnode) {
  struct ss_inode *inode = get_inode_by_id(inum);
  if (inode == NULL) {
    std::cerr << "Cannot find inode " << inum << std::endl;
    return;
  }

  struct ss_segment *segment = &inode->segments[0];
  const uint64_t lba = segment->start_lba;
  fake_dstorage[lba] = dnode;
}

uint64_t add_dnode_to_storage(const uint64_t inum,
                              const struct ss_dnode drecord) {
  uint64_t lba = static_cast<uint64_t>(uni(rng));
  fake_dstorage[lba] = drecord;
  inode_map[inum] = lba;
  return lba;
}

struct ss_dnode *get_dnode_by_id(const uint64_t inum) {
  struct ss_inode *inode = get_inode_by_id(inum);

  // Return NULL if the inode does not contain a directory
  if (!(inode->flags & FLAG_DIRECTORY)) {
    return NULL;
  }

  // A directory can only contain one block
  struct ss_segment *segment = &inode->segments[0];

  // Fetch the start ID from memory
  return &fake_dstorage[segment->start_lba];
}

struct ss_inode *get_inode_by_id(const uint64_t inum) {
  for (auto &lba : checkpoint_region) {
    std::map<uint64_t, uint64_t> map = get_imap(lba);
    auto found = map.find(inum);
    if (found == map.end()) continue;
    return get_inode_from_disk(found->second);
  }

  return NULL;
}
