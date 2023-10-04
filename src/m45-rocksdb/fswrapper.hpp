#include "directory.hpp"
#include "rocksdb/file_system.h"
#include "rocksdb/io_status.h"

namespace ROCKSDB_NAMESPACE {
class StoDirFS : public FSDirectory {
public:
  StoDirFS(char *name, const uint64_t parent_inode);
	StoDirFS(const uint64_t inum, const struct ss_dnode *dnode);
  ~StoDirFS();

  virtual IOStatus Fsync(const IOOptions &opts, IODebugContext *db);

private:
  StoDir *directory;	
};

class StoFileLock : public FileLock {
public:
  int64_t inode_num;
  std::string name;

  void Clear();
  ~StoFileLock();
  StoFileLock(const uint64_t inode_num, std::string name);
};
 
}
