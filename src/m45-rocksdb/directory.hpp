#ifndef STOSYS_DIR_H
#define STOSYS_DIR_H
#pragma once
#include <iostream>
#include <vector>

#include "rocksdb/file_system.h"
#include "rocksdb/io_status.h"
#include "structures.h"

class StoDir {
 public:
  // Records
  std::array<struct ss_dnode_record, DIRSIZE> records;

  // Elements stored on disk
  uint64_t inode_number;
  uint64_t parent_inode;
  uint16_t namelen;
  char name[NAMELEN];

  StoDir(char *name, const uint64_t parent_inode);
  StoDir(const uint64_t inum, const struct ss_dnode *node);
  ~StoDir();

  int add_entry(const uint16_t inode_number, const uint16_t reclen,
                const char *name);
  int remove_entry(const char *name);
  struct ss_dnode_record *find_entry(const char *name);
  struct ss_dnode create_dnode();

  void write_to_disk();

 private:
  uint8_t entry_index = 0;
};

struct find_inode_callbacks {
  struct ss_dnode_record *(*missing_directory_cb)(const char *name,
                                                  StoDir &parent,
                                                  void *user_data);
  struct ss_inode *(*missing_file_cb)(const char *name, StoDir &parent,
                                      void *user_data);
  void (*found_file_cb)(const char *name, StoDir &parent, const ss_inode *inode,
                        void *user_data);
  void *user_data;
};

enum DirectoryError {
  Dnode_not_found = -1,
  Found_inode = 0,
  Created_inode = 1,
  Directory_not_found = -2,
  Inode_not_found = -3
};

// We copy data here because it is easier, but you might want to do
// it by reference instead. This does make the logic a lot more
// complex though. Maybe a good use case for smart pointers?
enum DirectoryError find_inode(StoDir &directory, std::string name,
                               struct ss_inode *found,
                               struct find_inode_callbacks *cbs);

#endif
