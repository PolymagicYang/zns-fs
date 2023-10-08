#include "allocator.hpp"
#include "directory.hpp"
#include "file.hpp"
#include "rocksdb/file_system.h"
#include "rocksdb/io_status.h"

namespace ROCKSDB_NAMESPACE {
class StoDirFS : public FSDirectory {
 public:
  StoDirFS(char *name, const uint64_t parent_inode, BlockManager *);
  StoDirFS(const uint64_t inum, const struct ss_dnode *dnode, BlockManager *);
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

class StoRAFile : public FSRandomAccessFile {
 public:
  StoRAFile(struct ss_inode *inode, BlockManager *);
  ~StoRAFile();

  virtual IOStatus Read(uint64_t offset, size_t size, const IOOptions &options,
                        Slice *result, char *scratch,
                        IODebugContext *dbg) const;

 private:
  StoFile *file;
};

class StoSeqFile : public FSSequentialFile {
 public:
  StoSeqFile(struct ss_inode *inode, BlockManager *);
  ~StoSeqFile();
  virtual IOStatus Read(size_t size, const IOOptions &options, Slice *result,
                        char *scratch, IODebugContext *dbg);
  virtual IOStatus Skip(uint64_t size);

 private:
  StoFile *file;
  uint64_t offset;
  // cheap hack to deal with the end of file being reached
  bool eof;
};

class StoWriteFile : public FSWritableFile {
 public:
  StoWriteFile(struct ss_inode *inode, BlockManager *);
  ~StoWriteFile();

  virtual IOStatus Append(const Slice &data, const IOOptions &options,
                          IODebugContext *dbg);
  virtual IOStatus Flush(const IOOptions &options, IODebugContext *dbg);
  virtual IOStatus Sync(const IOOptions &options, IODebugContext *dbg);
  virtual IOStatus Close(const IOOptions &options, IODebugContext *dbg);

 private:
  StoFile *file;
  uint64_t offset;
};

}  // namespace ROCKSDB_NAMESPACE
