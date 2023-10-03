#ifndef STOSYS_DIR_H
#define STOSYS_DIR_H
#pragma once
#include <iostream>
#include <vector>

#include "structures.h"

class StoDir {
 public:
  // Records
  std::array<struct ss_dnode_record, DIRSIZE> records;

  // Elements stored on disk
  uint32_t inode_number;
  uint16_t namelen;
  char name[NAMELEN];

  StoDir(char *name);
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

struct ss_inode *find_file(StoDir &directory, std::string name);

#endif
