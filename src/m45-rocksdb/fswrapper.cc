#include "fswrapper.hpp"

#include <cassert>

namespace ROCKSDB_NAMESPACE {

StoDirFS::StoDirFS(char *name, const uint64_t parent_inode) {
  this->directory = new StoDir(name, parent_inode);
}

StoDirFS::StoDirFS(const uint64_t inum, const struct ss_dnode *dnode) {
  this->directory = new StoDir(inum, dnode);
}

StoDirFS::~StoDirFS() { free(this->directory); }

IOStatus StoDirFS::Fsync(const IOOptions &opts, IODebugContext *db) {
  std::cerr << "Fsync the directory" << std::endl;
  return IOStatus::OK();
}

StoFileLock::StoFileLock(const uint64_t inode_num, std::string name) {
  this->inode_num = inode_num;
  this->name = name;
}

void StoFileLock::Clear() {
  this->name.clear();
  this->inode_num = 0;
}

StoFileLock::~StoFileLock() { assert(this->inode_num != 0); }

StoRAFile::StoRAFile(struct ss_inode *inode) {
  this->file = new StoFile(inode);
}

StoRAFile::~StoRAFile() {
  // something
}

IOStatus StoRAFile::Read(uint64_t offset, size_t size, const IOOptions &options,
                         Slice *result, char *scratch,
                         IODebugContext *dbg) const {
  // Read a total offset + size bytes from the underlying file
  // TODO(valentijn): memory leak?
  char *buffer = (char *)malloc(offset + size);
  file->read(size + offset, (void *)buffer);

  // Skip the offset
  buffer += offset;

  // Copy the buffer over to the result slice
  *result = Slice(buffer);
  return IOStatus::OK();
}

StoSeqFile::StoSeqFile(struct ss_inode *inode) {
  this->file = new StoFile(inode);
  this->offset = 0;
  this->eof = false;
}

StoSeqFile::~StoSeqFile() {
  // something
}

IOStatus StoSeqFile::Read(size_t size, const IOOptions &options, Slice *result,
                          char *scratch, IODebugContext *dbg) {
  if (eof) {
    *result = Slice();
    return IOStatus::OK();
  }
  // We read throw away our offset, this is an inefficient way of
  // doing things
  size_t adjusted = offset + size;
  char *buffer = (char *)malloc(adjusted);
  file->read(adjusted, (void *)buffer);
  *buffer += offset;
  *result = Slice(buffer);
  free(buffer);
  this->offset = adjusted;

  if (strlen(buffer) < size) this->eof = true;

  return IOStatus::OK();
}

IOStatus StoSeqFile::Skip(uint64_t size) {
  // Change our offset to account for the additional skip
  size_t adjusted = offset + size;
  this->offset = adjusted;
}

StoWriteFile::StoWriteFile(struct ss_inode *inode) {
  this->file = new StoFile(inode);
  this->offset = 0;
}

StoWriteFile::~StoWriteFile() {
  // Something
}

// Append data to the end of the file
IOStatus StoWriteFile::Append(const Slice &data, const IOOptions &options,
                              IODebugContext *dbg) {
  if (!this->file) return IOStatus::IOError("File closed");

  this->file->write(data.size(), (void *)data.data());
  return IOStatus::OK();
}

// Flush writes the application data to the filesystem
IOStatus StoWriteFile::Flush(const IOOptions &options, IODebugContext *dbg) {
  if (!this->file) return IOStatus::IOError("File closed");

  this->file->write_to_disk();
  return IOStatus::OK();
}

// Sync writes the filesystem data to the FTL
IOStatus StoWriteFile::Sync(const IOOptions &options, IODebugContext *dbg) {
  if (!this->file) return IOStatus::IOError("File closed");

  this->file->write_to_disk();
  return IOStatus::OK();
}

// Close our file
IOStatus StoWriteFile::Close(const IOOptions &options, IODebugContext *dbg) {
  if (!this->file) return IOStatus::IOError("File closed");

  this->file = nullptr;
  return IOStatus::OK();
}

}  // namespace ROCKSDB_NAMESPACE
