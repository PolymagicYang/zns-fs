/*
 * MIT License
Copyright (c) 2021 - current
Authors:  Animesh Trivedi
This code is part of the Storage System Course at VU Amsterdam
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

#include "S2FileSystem.h"

#include <stosys_debug.h>
#include <sys/mman.h>
#include <utils.h>

#include <iostream>
#include <regex>
#include <string>

#include "directory.hpp"
#include "file.hpp"
#include "fswrapper.hpp"
#include "inode.hpp"
#include "structures.h"

uint64_t g_lba_size;

namespace ROCKSDB_NAMESPACE {
// Maps inode numbers to locks. They are not automatically populated,
// so a lock must be inserted for first use.
std::map<const std::string, std::mutex> file_locks;

S2FileSystem::S2FileSystem(std::string uri_db_path, bool debug) {
  FileSystem::Default();
  std::string sdelimiter = ":";
  std::string edelimiter = "://";
  this->_uri = uri_db_path;
  struct zdev_init_params params;
  std::string device = uri_db_path.substr(
      uri_db_path.find(sdelimiter) + sdelimiter.size(),
      uri_db_path.find(edelimiter) -
          (uri_db_path.find(sdelimiter) + sdelimiter.size()));
  // make sure to setup these parameters properly and check the forced reset
  // flag for M5
  params.name = strdup(device.c_str());
  params.log_zones = 3;
  params.gc_wmark = 1;
  params.force_reset = false;
  int ret = init_ss_zns_device(&params, &this->_zns_dev);
  if (ret != 0) {
    std::cout << "Error: " << uri_db_path << " failed to open the device "
              << device.c_str() << "\n";
    std::cout << "Error: ret " << ret << "\n";
  }

  // Setup the inode and dnode map
  this->inodes = {.lock = PTHREAD_RWLOCK_INITIALIZER, .inodes = __inode_map()};
  this->dnodes = {.lock = PTHREAD_RWLOCK_INITIALIZER, .dnodes = __dir_map()};

  setup_test_system();
  file_locks = std::map<const std::string, std::mutex>();

  // Create the root directory node

  // Be aware that the behaviour here is a bit subtle, the inode is
  // not generated until the directory is written to disk. So you can
  // only rely on the inode number being there after the write_to_disk
  // is called
  StoDir root = StoDir("/", 2);
  StoDir foo = StoDir("foo", 2);

  root.write_to_disk();
  foo.write_to_disk();
  root.add_entry(foo.inode_number, 1, "foo");
  root.write_to_disk();

  // root.add_entry(4, 2, "foo/bar");
  // root.add_entry(5, 3, "foo/baz");
  root.add_entry(6, 2, "queef");
  std::cout << "Added all the entries" << std::endl;
  struct ss_inode new_inode;
  enum DirectoryError err =
      find_inode(root, "foo/../foo/./baz", &new_inode, NULL);
  if (err == DirectoryError::Dnode_not_found) {
    std::cerr << "Cannot find dnode" << std::endl;
  } else if (err == DirectoryError::Directory_not_found) {
    std::cerr << "Cannot find parent directory" << std::endl;
  } else if (err == DirectoryError::Created_inode) {
    std::cerr << "Created a new inode for the file" << std::endl;
  } else if (err == DirectoryError::Found_inode) {
    std::cerr << "Found the inode" << std::endl;
  } else if (err == DirectoryError::Inode_not_found) {
    std::cerr << "Did not find the inode" << std::endl;
  }

  std::cout << "Find file in root" << std::endl;

  assert(ret == 0);
  assert(this->_zns_dev->lba_size_bytes != 0);
  assert(this->_zns_dev->capacity_bytes != 0);

  // Globally store our LBA size so we can access everywhere
  // without copying everything oer.
  g_lba_size = this->_zns_dev->lba_size_bytes;

  ss_dprintf(DBG_FS_1,
             "device %s is opened and initialized, reported LBA size is %u and "
             "capacity %lu \n",
             device.c_str(), this->_zns_dev->lba_size_bytes,
             this->_zns_dev->capacity_bytes);
}

S2FileSystem::~S2FileSystem() {}

struct ss_inode *callback_missing_file_create(const char *name, StoDir &parent,
                                              void *user_data) {
  std::cerr << "Create " << name << " as sequential file" << std::endl;
  // The system seems to want the file to contain a newline
  // for some reason, so let's do that

  // We don't know how large this will end up being, but
  // we start off with one block
  StoInode inode = StoInode(1, (char *)name);
  inode.write_to_disk();
  parent.add_entry(inode.inode_number, 12, name);
  parent.write_to_disk();
  return get_inode_by_id(inode.inode_number);
}

// Create a brand new sequentially-readable file with the specified name.
// On success, stores a pointer to the new file in *result and returns OK.
// On failure stores nullptr in *result and returns non-OK.  If the file does
// not exist, returns a non-OK status.
//
// The returned file will only be accessed by one thread at a time.
IOStatus S2FileSystem::NewSequentialFile(
    const std::string &fname, const FileOptions &file_opts,
    std::unique_ptr<FSSequentialFile> *result,
    __attribute__((unused)) IODebugContext *dbg) {
  std::cerr << "[SequentialFile] Create " << fname << std::endl;
  *result = nullptr;
  StoDir root = StoDir(2, get_dnode_by_id(2));
  struct ss_inode found_inode;
  std::string cut = fname.substr(1, fname.size());

  struct find_inode_callbacks cbs = {
      // .missing_file_cb = callback_missing_file_create
  };

  enum DirectoryError err = find_inode(root, cut, &found_inode, &cbs);

  if (err == DirectoryError::Dnode_not_found) {
    std::cerr << "Cannot find dnode" << std::endl;
  } else if (err == DirectoryError::Directory_not_found) {
    std::cerr << "Cannot find parent directory" << std::endl;
  } else if (err == DirectoryError::Created_inode) {
    std::cerr << "Created a new inode for the file" << std::endl;
    // result->reset(new StoSeqFile(&found_inode));
    return IOStatus::OK();
  } else if (err == DirectoryError::Found_inode) {
    std::cerr << "Found the inode" << std::endl;
    result->reset(new StoSeqFile(&found_inode));
    return IOStatus::OK();
  } else if (err == DirectoryError::Inode_not_found) {
    std::cerr << "Did not find the inode" << std::endl;
  }

  return IOStatus::IOError(__FUNCTION__);
}

IOStatus S2FileSystem::IsDirectory(const std::string &,
                                   const IOOptions &options, bool *is_dir,
                                   IODebugContext *) {
  return IOStatus::IOError(__FUNCTION__);
}

// Create a brand new random access read-only file with the
// specified name.  On success, stores a pointer to the new file in
// *result and returns OK.  On failure stores nullptr in *result and
// returns non-OK.  If the file does not exist, returns a non-OK
// status.
//
// The returned file may be concurrently accessed by multiple threads.
IOStatus S2FileSystem::NewRandomAccessFile(
    const std::string &fname, const FileOptions &file_opts,
    std::unique_ptr<FSRandomAccessFile> *result,
    __attribute__((unused)) IODebugContext *dbg) {
  std::cerr << "[RandomAccess] create " << fname << std::endl;
  *result = nullptr;
  StoDir root = StoDir(2, get_dnode_by_id(2));
  struct ss_inode found_inode;
  std::string cut = fname.substr(1, fname.size());

  struct find_inode_callbacks cbs = {
      //.missing_file_cb = callback_missing_file_create
  };

  enum DirectoryError err = find_inode(root, cut, &found_inode, &cbs);

  if (err != DirectoryError::Found_inode) {
    return IOStatus::IOError(__FUNCTION__);
  }

  result->reset(new StoRAFile(&found_inode));
}

const char *S2FileSystem::Name() const { return "S2FileSytem"; }

// Create an object that writes to a new file with the specified
// name.  Deletes any existing file with the same name and creates a
// new file.  On success, stores a pointer to the new file in
// *result and returns OK.  On failure stores nullptr in *result and
// returns non-OK.
//
// The returned file will only be accessed by one thread at a time.
IOStatus S2FileSystem::NewWritableFile(const std::string &fname,
                                       const FileOptions &file_opts,
                                       std::unique_ptr<FSWritableFile> *result,
                                       __attribute__((unused))
                                       IODebugContext *dbg) {
  std::cout << "[WriteableFile] Create " << fname << std::endl;
  *result = nullptr;
  StoDir root = StoDir(2, get_dnode_by_id(2));
  struct ss_inode found_inode;
  std::string cut = fname.substr(1, fname.size());

  struct find_inode_callbacks cbs = {.missing_file_cb =
                                         callback_missing_file_create};

  enum DirectoryError err = find_inode(root, cut, &found_inode, &cbs);

  if (err == DirectoryError::Found_inode) {
    // TODO(someone): File deletion
    return IOStatus::IOError(
        "File already exists. Please implement file deletion!");
  } else if (err != DirectoryError::Created_inode) {
    return IOStatus::IOError(__FUNCTION__);
  }

  result->reset(new StoWriteFile(&found_inode));
  return IOStatus::OK();
}

IOStatus S2FileSystem::ReopenWritableFile(const std::string &,
                                          const FileOptions &,
                                          std::unique_ptr<FSWritableFile> *,
                                          IODebugContext *) {
  return IOStatus::IOError(__FUNCTION__);
}

IOStatus S2FileSystem::NewRandomRWFile(const std::string &, const FileOptions &,
                                       std::unique_ptr<FSRandomRWFile> *,
                                       IODebugContext *) {
  return IOStatus::IOError(__FUNCTION__);
}

IOStatus S2FileSystem::NewMemoryMappedFileBuffer(
    const std::string &, std::unique_ptr<MemoryMappedFileBuffer> *) {
  return IOStatus::IOError(__FUNCTION__);
}

// Create an object that represents a directory. Will fail if directory
// doesn't exist. If the directory exists, it will open the directory
// and create a new Directory object.
//
// On success, stores a pointer to the new Directory in
// *result and returns OK. On failure stores nullptr in *result and
// returns non-OK.
IOStatus S2FileSystem::NewDirectory(const std::string &name,
                                    const IOOptions &io_opts,
                                    std::unique_ptr<FSDirectory> *result,
                                    __attribute__((unused))
                                    IODebugContext *dbg) {
  std::string cut = name.substr(1, name.size() - 2);
  std::cout << "[NewDirectory] " << cut << " " << name << std::endl;
  StoDir root = StoDir(2, get_dnode_by_id(2));
  struct ss_inode found_inode;
  enum DirectoryError error = find_inode(root, cut, &found_inode, NULL);

  if (error != DirectoryError::Found_inode) {
    return IOStatus::IOError("New Directory: " + name + "not found");
  }

  if (!(found_inode.flags & FLAG_DIRECTORY)) {
    return IOStatus::InvalidArgument(name + " is not a directory.");
  }
  result->reset(new StoDirFS(found_inode.id, get_dnode_by_id(found_inode.id)));
  return IOStatus::OK();
}

IOStatus S2FileSystem::GetFreeSpace(const std::string &, const IOOptions &,
                                    uint64_t *, IODebugContext *) {
  return IOStatus::IOError(__FUNCTION__);
}

IOStatus S2FileSystem::Truncate(const std::string &, size_t, const IOOptions &,
                                IODebugContext *) {
  return IOStatus::IOError(__FUNCTION__);
}

// Create the specified directory. Returns error if directory exists.
IOStatus S2FileSystem::CreateDir(const std::string &dirname,
                                 const IOOptions &options,
                                 __attribute__((unused)) IODebugContext *dbg) {
  return IOStatus::IOError(__FUNCTION__);
}

// If we cannot find the directory, we will create one and return the entry
// for it
struct ss_dnode_record *callback_missing_directory(const char *name,
                                                   StoDir &parent,
                                                   void *user_data) {
  std::cerr << "Create missing directory " << name << " in " << parent.name
            << std::endl;
  StoDir directory = StoDir((char *)name, parent.inode_number);
  directory.write_to_disk();
  parent.add_entry(directory.inode_number, 12, name);
  parent.write_to_disk();
  return parent.find_entry(name);
}

// Create a new directory if we cannot find it, we could also pass
// the same function as missing directory.
struct ss_inode *callback_missing_file_create_dir(const char *name,
                                                  StoDir &parent,
                                                  void *user_data) {
  std::cerr << "Create the target directory " << name << " in " << parent.name
            << std::endl;
  StoDir directory = StoDir((char *)name, parent.inode_number);
  directory.write_to_disk();
  parent.add_entry(directory.inode_number, 12, name);
  parent.write_to_disk();
  return get_inode_by_id(directory.inode_number);
}

void callback_found_file_print(const char *name, StoDir &parent,
                               struct ss_inode *inode,
                               struct ss_dnode_record *entry, void *user_data) {
  std::cerr << "Found file " << name << " in " << parent.name << std::endl;
}

// Creates directory if missing. Return Ok if it exists, or successful in
// Creating.
IOStatus S2FileSystem::CreateDirIfMissing(const std::string &dirname,
                                          const IOOptions &options,
                                          __attribute__((unused))
                                          IODebugContext *dbg) {
  // Remove the starting and trailing /
  std::string cut = dirname.substr(1, dirname.size() - 1);

  // Setup the callbacks so we create the string of directories
  struct find_inode_callbacks cbs = {
      .missing_directory_cb = callback_missing_directory,
      .missing_file_cb = callback_missing_file_create_dir,
      .found_file_cb = callback_found_file_print,
  };
  StoDir root = StoDir(2, get_dnode_by_id(2));
  struct ss_inode found_inode;
  enum DirectoryError error = find_inode(root, cut, &found_inode, &cbs);

  // Check if our inode is actually a directory, otherwise it is okay.
  if (!(found_inode.flags & FLAG_DIRECTORY)) {
    return IOStatus::InvalidArgument("File not a directory!");
  }

  return IOStatus::OK();
}

IOStatus S2FileSystem::GetFileSize(const std::string &fname,
                                   const IOOptions &options,
                                   uint64_t *file_size,
                                   __attribute__((unused))
                                   IODebugContext *dbg) {
  return IOStatus::IOError(__FUNCTION__);
}

IOStatus S2FileSystem::DeleteDir(const std::string &dirname,
                                 const IOOptions &options,
                                 __attribute__((unused)) IODebugContext *dbg) {
  return IOStatus::IOError(__FUNCTION__);
}

IOStatus S2FileSystem::GetFileModificationTime(const std::string &fname,
                                               const IOOptions &options,
                                               uint64_t *file_mtime,
                                               __attribute__((unused))
                                               IODebugContext *dbg) {
  return IOStatus::IOError(__FUNCTION__);
}

IOStatus S2FileSystem::GetAbsolutePath(const std::string &db_path,
                                       const IOOptions &options,
                                       std::string *output_path,
                                       __attribute__((unused))
                                       IODebugContext *dbg) {
  return IOStatus::IOError(__FUNCTION__);
}

IOStatus S2FileSystem::DeleteFile(const std::string &fname,
                                  const IOOptions &options,
                                  __attribute__((unused)) IODebugContext *dbg) {
  return IOStatus::IOError(__FUNCTION__);
}

IOStatus S2FileSystem::NewLogger(const std::string &fname,
                                 const IOOptions &io_opts,
                                 std::shared_ptr<Logger> *result,
                                 __attribute__((unused)) IODebugContext *dbg) {
  return IOStatus::IOError(__FUNCTION__);
}

IOStatus S2FileSystem::GetTestDirectory(const IOOptions &options,
                                        std::string *path,
                                        __attribute__((unused))
                                        IODebugContext *dbg) {
  return IOStatus::IOError(__FUNCTION__);
}

// Release the lock acquired by a previous successful call to LockFile.
// REQUIRES: lock was returned by a successful LockFile() call
// REQUIRES: lock has not already been unlocked.
IOStatus S2FileSystem::UnlockFile(FileLock *lock, const IOOptions &options,
                                  __attribute__((unused)) IODebugContext *dbg) {
  return IOStatus::IOError(__FUNCTION__);
}

struct ss_inode *callback_missing_file_create_lock(const char *name,
                                                   StoDir &parent,
                                                   void *user_data) {
  // Our lock will only contain one block so we can hardcode it here
  StoInode inode = StoInode(1, (char *)name);

  std::cerr << "Lock created" << std::endl;

  // Open as file and write thee lock to disk. We use a sentinel value
  // in the data block to store that is locked. A more lightweight approach
  // would use the flag system in the inode.
  StoFile file = StoFile(&inode);
  file.write(5, (void *)"lock");

  // Flush the inode. Since this file is new, we also flush the directory.
  file.write_to_disk();
  parent.add_entry(inode.inode_number, 12, name);
  parent.write_to_disk();

  return get_inode_by_id(inode.inode_number);
}

// Lock the specified file.  Used to prevent concurrent access to
// the same db by multiple processes.  On failure, stores nullptr in
// *lock and returns non-OK.
//
// On success, stores a pointer to the object that represents the
// acquired lock in *lock and returns OK.  The caller should call
// UnlockFile(*lock) to release the lock.  If the process exits,
// the lock will be automatically released.
//
// If somebody else already holds the lock, finishes immediately
// with a failure.  I.e., this call does not wait for existing locks
// to go away.
//
// May create the named file if it does not already exist.
IOStatus S2FileSystem::LockFile(const std::string &fname,
                                const IOOptions &options, FileLock **lock,
                                __attribute__((unused)) IODebugContext *dbg) {
  *lock = nullptr;
  std::cerr << "[Lock]" << fname << std::endl;
  struct ss_inode found_inode;

  struct find_inode_callbacks cbs = {.missing_file_cb =
                                         callback_missing_file_create_lock};

  StoDir root = StoDir(2, get_dnode_by_id(2));
  std::string cut = fname.substr(1, fname.size() - 1);
  enum DirectoryError error = find_inode(root, cut, &found_inode, &cbs);

  if (error == DirectoryError::Created_inode) {
    *lock = new StoFileLock(found_inode.id, fname);
    return IOStatus::OK();
  } else if (error != DirectoryError::Found_inode) {
    return IOStatus::IOError("Path to " + fname + " does not exist");
  }

  // We check this in our callback so that we can more easily signal
  // errors to the system
  StoFile file = StoFile(&found_inode);
  char buf[128];

  // Read the first five bytes of our file to find our sentinel and see if it is
  // the same
  // TODO: Check if this works? What does this do whenever it is unlocked.
  file.read(5, &buf);
  if (strncmp(buf, "lock", 5) == 0) {
    return IOStatus::IOError("Lock " + fname + " already locked");
  } else {
    return IOStatus::IOError("File " + fname + " is not a lock");
  }
}

IOStatus S2FileSystem::AreFilesSame(const std::string &, const std::string &,
                                    const IOOptions &, bool *,
                                    IODebugContext *) {
  return IOStatus::IOError(__FUNCTION__);
}

IOStatus S2FileSystem::NumFileLinks(const std::string &, const IOOptions &,
                                    uint64_t *, IODebugContext *) {
  return IOStatus::IOError(__FUNCTION__);
}

IOStatus S2FileSystem::LinkFile(const std::string &, const std::string &,
                                const IOOptions &, IODebugContext *) {
  return IOStatus::IOError(__FUNCTION__);
}

void callback_found_file_rename(const char *name, StoDir &parent,
                                struct ss_inode *ss_inode,
                                struct ss_dnode_record *entry,
                                void *user_data) {
  char *new_name = (char *)user_data;
  size_t length = strlen(new_name);

  // Copy the name to the inode and write it to disk. Somewhat
  // inconvient to wrap it around a class.
  strncpy((char *)ss_inode->name, new_name, length);
  ss_inode->name[length] = '\0';
  ss_inode->strlen = length;
  StoInode inode = StoInode(ss_inode);
  inode.write_to_disk();

  // Update our directory entry and write to disk
  strncpy((char *)entry->name, new_name, length);
  entry->name[length] = '\0';
  entry->namelen = length;
  parent.write_to_disk();
}

IOStatus S2FileSystem::RenameFile(const std::string &src,
                                  const std::string &target,
                                  const IOOptions &options,
                                  __attribute__((unused)) IODebugContext *dbg) {
  StoDir root = StoDir(2, get_dnode_by_id(2));
  std::string cut = src.substr(1, src.size());
  std::cout << "[Rename] " << cut << " to " << target << std::endl;

  auto index_src = src.rfind("/");
  auto index_target = target.rfind("/");

  std::string new_name = target.substr(index_target, target.size());
  std::string path_to_src = src.substr(0, index_src);
  std::string path_to_target = target.substr(0, index_target);

  if (path_to_src.compare(path_to_target) != 0) {
    return IOStatus::IOError("Rename cannot move to a different directory!");
  }

  // Remove a leading / if it exists
  if (new_name[0] == '/') {
    new_name = new_name.substr(1, new_name.size());
  }

  struct find_inode_callbacks cbs = {
      .found_file_cb = callback_found_file_rename,
      .user_data = (void *)new_name.c_str()};
  struct ss_inode found_inode;
  enum DirectoryError err = find_inode(root, cut, &found_inode, &cbs);
  if (err == DirectoryError::Found_inode) {
    return IOStatus::OK();
  }

  return IOStatus::NotFound();
}

IOStatus S2FileSystem::GetChildrenFileAttributes(
    const std::string &dir, const IOOptions &options,
    std::vector<FileAttributes> *result,
    __attribute__((unused)) IODebugContext *dbg) {
  return FileSystem::GetChildrenFileAttributes(dir, options, result, dbg);
}

// Store in *result the names of the children of the specified directory.
// The names are relative to "dir".
// Original contents of *results are dropped.
// Returns OK if "dir" exists and "*result" contains its children.
//         NotFound if "dir" does not exist, the calling process does not have
//                  permission to access "dir", or if "dir" is invalid.
//         IOError if an IO Error was encountered
IOStatus S2FileSystem::GetChildren(const std::string &dir,
                                   const IOOptions &options,
                                   std::vector<std::string> *result,
                                   __attribute__((unused))
                                   IODebugContext *dbg) {
  return IOStatus::IOError(__FUNCTION__);
}

// Returns OK if the named file exists.
//         NotFound if the named file does not exist,
//                  the calling process does not have permission to determine
//                  whether this file exists, or if the path is invalid.
//         IOError if an IO Error was encountered
IOStatus S2FileSystem::FileExists(const std::string &fname,
                                  const IOOptions &options,
                                  __attribute__((unused)) IODebugContext *dbg) {
  StoDir root = StoDir(2, get_dnode_by_id(2));
  std::string cut = fname.substr(1, fname.size());
  std::cout << "[FileExist] " << cut << std::endl;

  struct ss_inode found_inode;
  // TODO(valentijn): permission checking
  // TODO(valentijn): overzealous not found code
  enum DirectoryError err = find_inode(root, cut, &found_inode, NULL);
  if (err == DirectoryError::Found_inode) {
    return IOStatus::OK();
  } else if (err == DirectoryError::Dnode_not_found) {
    return IOStatus::IOError("Bad dnode " + fname);
  }

  return IOStatus::NotFound();
}

IOStatus S2FileSystem::ReuseWritableFile(
    const std::string &fname, const std::string &old_fname,
    const FileOptions &file_opts, std::unique_ptr<FSWritableFile> *result,
    __attribute__((unused)) IODebugContext *dbg) {
  return IOStatus::IOError(__FUNCTION__);
}
}  // namespace ROCKSDB_NAMESPACE
