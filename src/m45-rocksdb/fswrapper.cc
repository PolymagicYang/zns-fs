#include "fswrapper.hpp"

#include <cstdint>
#include <pthread.h>

#include <cassert>

#include "../common/unused.h"
#include "allocator.hpp"

#define Min(x, y) ((x) > (y) ? (y) : (x))

namespace ROCKSDB_NAMESPACE {

StoDirFS::StoDirFS(char *name, const uint64_t parent_inode,
                   BlockManager *allocator) {
  this->directory = new StoDir(name, parent_inode, allocator);
}

StoDirFS::StoDirFS(const uint64_t inum, const struct ss_dnode *dnode,
                   BlockManager *allocator) {
  this->directory = new StoDir(inum, dnode, allocator);
}

StoDirFS::~StoDirFS() { delete this->directory; }

IOStatus StoDirFS::Fsync(const IOOptions &opts, IODebugContext *db) {
  UNUSED(opts);
  UNUSED(db);
  std::cerr << "Fsync the directory" << std::endl;
  this->directory->write_to_disk();
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

StoFileLock::~StoFileLock() { assert(this->inode_num == 0); }

StoRAFile::StoRAFile(struct ss_inode *inode, BlockManager *allocator) {
  this->file = new StoFile(inode, allocator);
}

StoRAFile::~StoRAFile() {
  // something
  delete this->file;
  if (this->clean_slice) {
    free(this->buffer);
  }
}

IOStatus StoRAFile::Read(uint64_t offset, size_t size, const IOOptions &options,
                         Slice *result, char *scratch,
                         IODebugContext *dbg) const {
  UNUSED(scratch);
  UNUSED(dbg);

  std::cout << "RA read" << std::endl;
  // Read a total offset + size bytes from the underlying file
  // TODO(valentijn): memory leak?
  pthread_mutex_lock(&this->file->inode.lock);

  char *buffer = (char *)malloc(
      Round_up(Min(offset + size, this->file->inode.node->size), g_lba_size) *
      2);
  this->buffer = buffer;
  this->clean_slice = true;

  pthread_mutex_unlock(&this->file->inode.lock);
  file->read(size + offset, (void *)buffer);

  // Skip the offset
  buffer += offset;

  // Copy the buffer over to the result slice
  *result = Slice(buffer, Min(this->file->inode.node->size - (size_t)1, size));

  std::cout << "RA read " << result->data() << std::endl;
  return IOStatus::OK();
}

StoSeqFile::StoSeqFile(struct ss_inode *inode, BlockManager *allocator) {
  this->file = new StoFile(inode, allocator);
  this->offset = 0;
  this->eof = false;
}

StoSeqFile::~StoSeqFile() {
  // something
  delete this->file;
  free(this->buffer);
}

IOStatus StoSeqFile::Read(size_t size, const IOOptions &options, Slice *result,
                          char *scratch, IODebugContext *dbg) {

  if (eof) {
    printf("read value end.\n");
    *result = Slice();
    return IOStatus::OK();
  }
  // We read throw away our offset, this is an inefficient way of
  // doing things
  pthread_mutex_lock(&this->file->inode.lock);
  size_t adjusted = Min(offset + size, this->file->inode.node->size);

  printf("inode file size is %d\n", this->file->inode.node->size);
  char *buffer = (char *)malloc(Round_up(adjusted, g_lba_size) * 2);
  pthread_mutex_unlock(&this->file->inode.lock);
  this->file->read(adjusted, (void *)buffer);
  ((char *)buffer)[adjusted - 1] = '\0';
  *buffer += offset;

  // Clasp the amount we store based on the current size of our inode
  // TODO(everyone): make this more generic so we don't overallocate memory
  //   as badly as we do atm.
  *result =
      Slice(buffer, std::min(this->file->inode.node->size - offset - 1, size));
  this->buffer = buffer;

  printf("read value size is %d\n", std::min(this->file->inode.node->size - offset - 1, size));
  for (uint32_t i = 0; i < std::min(this->file->inode.node->size - offset - 1, size); i++) {
    printf("%x", buffer[i]);
  }
  printf("\n");

  this->offset = adjusted;

  if (strlen(buffer) < adjusted) this->eof = true;

  return IOStatus::OK();
}

IOStatus StoSeqFile::Skip(uint64_t size) {
  // Change our offset to account for the additional skip
  size_t adjusted = offset + size;
  this->offset = adjusted;
}

StoWriteFile::StoWriteFile(struct ss_inode *inode, BlockManager *allocator) {
  this->file = new StoFile(inode, allocator);

  this->offset = 0;
}

StoWriteFile::~StoWriteFile() {
  // Something
  delete this->file;
}

// Append data to the end of the file
IOStatus StoWriteFile::Append(const Slice &data, const IOOptions &options,
                              IODebugContext *dbg) {
  this->file->write(data.size(), (void *)data.data());
  return IOStatus::OK();
}

// Flush writes the application data to the filesystem
IOStatus StoWriteFile::Flush(const IOOptions &options, IODebugContext *dbg) {
  this->file->write_to_disk(true);
  return IOStatus::OK();
}

// Sync writes the filesystem data to the FTL
IOStatus StoWriteFile::Sync(const IOOptions &options, IODebugContext *dbg) {
  this->file->write_to_disk(true);
  return IOStatus::OK();
}

// Close our file
IOStatus StoWriteFile::Close(const IOOptions &options, IODebugContext *dbg) {
  return IOStatus::OK();
}

}  // namespace ROCKSDB_NAMESPACE
