#include "fswrapper.hpp"
#include <cassert>

namespace ROCKSDB_NAMESPACE {

StoDirFS::StoDirFS(char *name, const uint64_t parent_inode) {
  this->directory = new StoDir(name, parent_inode);
}

StoDirFS::StoDirFS(const uint64_t inum, const struct ss_dnode *dnode) {
  this->directory = new StoDir(inum, dnode);
}
  
StoDirFS::~StoDirFS() {
  free(this->directory);
  
}  

IOStatus StoDirFS::Fsync(const IOOptions &opts, IODebugContext *db) {
  std::cerr << "Fsync the directory" << std::endl;
}

StoFileLock::StoFileLock(const uint64_t inode_num, std::string name) {
  this->inode_num = inode_num;
  this->name = name;
}

void StoFileLock::Clear() {
  this->name.clear();
  this->inode_num = 0;  
}

StoFileLock::~StoFileLock() {
  assert(this->inode_num != 0);
}
  
  
}


