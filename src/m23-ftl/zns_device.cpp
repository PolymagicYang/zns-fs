/* MIT License
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

#include "zns_device.h"

#include <fcntl.h>
#include <libgen.h>
#include <libnvme.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "../common/nvmewrappers.h"
#include "../common/unused.h"
#include "../common/utils.h"
#include "ftl.hpp"
#include "ftlgc.hpp"
#include "zone.hpp"

extern "C" {
// TODO(valentijn): Implement this functionh
int deinit_ss_zns_device(struct user_zns_device *my_dev) {
  int ret = -ENOSYS;
  // cppcheck-suppress cstyleCast
  FTL *ftl = (FTL *)my_dev->_private;
  Calliope *mori = (Calliope *)ftl->mori;

  death_sensei = true;
  pthread_mutex_lock(&ftl->need_gc_lock);
  pthread_cond_signal(&ftl->need_gc);
  pthread_mutex_unlock(&ftl->need_gc_lock);

  if (mori != NULL && mori->thread.joinable()) mori->thread.join();

  // Store current ftl status.
  
  ftl->backup();
  free(my_dev);
  delete ftl->mori;
  delete ftl;
  return ret;
}

int init_ss_zns_device(struct zdev_init_params *params,
                       struct user_zns_device **my_dev) {
  int fd;
  int ret;
  void *regs;
  struct nvme_id_ns ns {};
  struct nvme_id_ctrl ctrl;

  fd = nvme_open(params->name);

  if (fd < 0) {
    printf("device %s opening failed %d errno %d \n", params->name, fd, errno);
    return -fd;
  }
  uint32_t nsid;
  ret = nvme_get_nsid(fd, &nsid);
  if (ret != 0) {
    printf("ERROR: failed to retrieve the nsid %d \n", ret);
    print_nvme_error("init", ret);
    return ret;
  }

  ret = nvme_identify_ns(fd, nsid, &ns);
  if (ret) {
    printf("ERROR: failed to retrieve the nsid %d \n", ret);
    print_nvme_error("init", ret);
    return ret;
  }
  uint32_t lba_size_in_use = 1 << ns.lbaf[(ns.flbas & 0xf)].ds;

  // Calculate MDTS_size for later use.
  // Copy the name of the target partition and strip the last numbers so
  // we get the device
  char *path;
  char name[6];
  strncpy(name, params->name, 6);
  name[5] = '\0';

  std::cout << params->name << std::endl;
  // Open the system file for reading to read the registers.
  int aret = asprintf(&path, "/sys/class/nvme/%s/device/resource0", name);
  if (aret < 0) return aret;

  int sysfd = open(path, O_RDONLY | O_SYNC);
  if (fd < 0) {
    fprintf(stderr, "failed to open %s\n", path);
    free(path);
    return 1;
  }

  // Map a page from the device so we can read the registers as a
  // pointer
  regs = mmap(NULL, getpagesize(), PROT_READ, MAP_SHARED, sysfd, 0);
  if (regs == MAP_FAILED) {
    fprintf(stderr, "failed to map device BAR\n");
    puts(path);
    free(path);
    close(fd);
    return 1;
  }

  // Load the cap registers and extract the MPSMIN value.
  uint64_t cap = nvme_mmio_read64((void *)((uint64_t)regs + NVME_REG_CAP));
  uint64_t mpsmin_raw =
      NVME_CAP_MPSMIN(cap);  // hard code this since libnvme is being difficult
  uint64_t MPSMIN = 1 << (12 + mpsmin_raw);

  nvme_identify_ctrl(fd, &ctrl);
  uint64_t MDTS = (uint64_t)ctrl.mdts - 1;
  uint64_t MDTS_SIZE = (1 << MDTS) * MPSMIN;

  FTL *ftl = new FTL(fd, MDTS_SIZE, nsid, lba_size_in_use, params->gc_wmark,
                     params->log_zones);
  free(path);
  close(sysfd);

  struct zns_device_testing_params tparams {
    .zns_lba_size = lba_size_in_use,
    .zns_zone_capacity = (uint32_t)ftl->zcap * lba_size_in_use,
    .zns_num_zones = static_cast<uint32_t>(ftl->zones.size()),
  };

  struct user_zns_device *device =
      (user_zns_device *)malloc(sizeof(struct user_zns_device));
  device->lba_size_bytes = lba_size_in_use,
  device->capacity_bytes =
      (ns.ncap - (ftl->log_zones + 1) * ftl->zcap) *
      lba_size_in_use,  // ZNS capacity - log zones (includes metadata).
      device->tparams = tparams;
  device->_private = ftl;
  *my_dev = device;

  munmap(regs, getpagesize());
  return 0;
}

int zns_udevice_read(struct user_zns_device *my_dev, uint64_t address,
                     void *buffer, uint32_t size) {
  // Check log map firstly because of fresh data, then check data map, one page
  // size at a time (to avoid overwrite). e.g. full sequantially writes on
  // address 0x0 to make a data map entry, then 1 page write on 0x2, the data in
  // log map will be newer than data map.
  // cppcheck-suppress cstyleCast
  FTL *flt = (FTL *)my_dev->_private;
  return flt->read(address, buffer, size);
}

int zns_udevice_write(struct user_zns_device *my_dev, uint64_t address,
                      void *buffer, uint32_t size) {
  // uint64_t *ret_wp;
  // cppcheck-suppress cstyleCast
  FTL *flt = (FTL *)my_dev->_private;
  uint32_t ret_size = flt->write(address, buffer, size);
  return ret_size;
}
}
