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

#include "zns_device.h"

#include <fcntl.h>
#include <libgen.h>
#include <libnvme.h>
#include <math.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <map>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "../common/nvmewrappers.h"
#include "../common/unused.h"
#include "../common/utils.h"

using Ftlmap = std::map<uint64_t, uint64_t>;
// TODO(valentijn): use lock to enable map thread-safe (e.g. rw-lock), maybe we
// should design a map with fined-grained parallelism.

int get_zns_zone_info(int fd, int nsid, uint64_t *zcap, uint32_t *nr) {
  // coypied from m1 to fetch zcap.
  char *zone_reports;

  struct nvme_zone_report zns_report {};
  struct nvme_zns_desc *desc = NULL;
  int ret;
  uint64_t num_zones;
  ret = nvme_zns_mgmt_recv(fd, nsid, 0, NVME_ZNS_ZRA_REPORT_ZONES,
                           NVME_ZNS_ZRAS_REPORT_ALL, 0, sizeof(zns_report),
                           (void *)&zns_report);
  if (ret != 0) {
    fprintf(stderr, "failed to report zones, ret %d \n", ret);
    return ret;
  }
  num_zones = le64_to_cpu(zns_report.nr_zones);
  uint64_t total_size =
      sizeof(zns_report) + (num_zones * sizeof(struct nvme_zns_desc));
  zone_reports = (char *)calloc(1, total_size);
  ret = nvme_zns_mgmt_recv(fd, nsid, 0, NVME_ZNS_ZRA_REPORT_ZONES,
                           NVME_ZNS_ZRAS_REPORT_ALL, 1, total_size,
                           (void *)zone_reports);
  if (ret != 0) {
    fprintf(stderr, "failed to report zones, ret %d \n", ret);
    return ret;
  }
  desc = ((struct nvme_zone_report *)zone_reports)->entries;
  num_zones = le64_to_cpu(((struct nvme_zone_report *)zone_reports)->nr_zones);
  *zcap = (uint64_t)le64_to_cpu(desc->zcap);
  *nr = num_zones;

  free(zone_reports);
  return ret;
}

int ss_zns_device_zone_reset(int fd, uint32_t nsid, uint64_t slba) {
  // this is to supress gcc warnings, remove it when you complete this function
  int32_t ret =
      nvme_zns_mgmt_send(fd, nsid, slba, false, NVME_ZNS_ZSA_RESET, 0, NULL);

  if (ret != 0) {
    print_nvme_error(ret);
  }
  return ret;
}
/*
one zone for meta data, others for data-zone.
*/
class FTL {
 public:
  int fd;
  uint32_t zcap;
  uint32_t nsid;
  uint64_t mdts_size;
  uint64_t log_wp;
  uint64_t data_wp;
  uint64_t log_end;
  uint16_t bsize;
  uint64_t next_empty_zone;
  int log_zones;

  FTL(int fd, uint64_t mdts_size, uint32_t nsid, uint32_t zcap, int log_zones,
      uint16_t bsize) {
    this->fd = fd;
    this->mdts_size = mdts_size;
    this->nsid = nsid;
    this->zcap = zcap;
    this->log_zones = log_zones;
    this->log_end =
        log_zones * zcap;  // upper bound of log logical block address.
    this->bsize = bsize;

    this->log_map = std::map<uint64_t, uint64_t>();
    this->data_map = std::map<uint64_t, uint64_t>();
    this->log_wp = zcap;  // first log address.
    this->data_wp = log_zones * zcap;
    this->next_empty_zone = -1;
  };

  ~FTL() {
    this->data_map.erase(this->data_map.begin(), this->data_map.end());
    this->log_map.erase(this->log_map.begin(), this->log_map.end());
  }

  bool get_pa(uint64_t addr, uint64_t *pa) {
    // search log map firstly, if no return then try data log.
    uint64_t ppa;
    uint64_t pba;
    bool has_ppa = this->get_ppa(addr, &ppa);
    if (has_ppa) {
      *pa = ppa;
      return true;
    } else {
      bool has_pba = this->get_pba(addr, &pba);
      if (has_pba) {
        *pa = pba;
        return true;
      } else {
        return false;
      }
    }
  }

  void insert_lba_log(uint64_t lpa, uint64_t ppa) {
    this->log_map.insert({lpa, ppa});
  }

  void insert_lba_data(uint64_t lba, uint64_t pba) {
    this->data_map.insert({lba, pba});
  }

  uint64_t get_wp() { return -1; }

  uint64_t update_wp() { return -1; }

  void merge_zones() {
    // TODO(valentijn): optimize with batch writing.
    // unused
    // std::vector<uint64_t> key, value;

    // tuple: {logicl block address, physical block address}.
    std::unordered_map<uint64_t, std::vector<std::tuple<uint64_t, uint64_t>>>
        base2lba;

    for (Ftlmap::iterator it = this->log_map.begin(); it != this->log_map.end();
         ++it) {
      uint64_t lba = it->first;
      uint64_t pba = it->second;
      // uint32_t offset = lba % this->zcap;
      uint32_t lba_base = (lba / this->zcap) * this->zcap;

      auto base = base2lba.find(lba_base);
      if (base == base2lba.end()) {
        std::vector<std::tuple<uint64_t, uint64_t>> vect{{lba, pba}};
        base2lba.insert({lba_base, vect});
      } else {
        base->second.push_back({lba, pba});
      }
    }

    uint64_t curr_data_wp_lba = this->data_wp;
    for (auto it = base2lba.begin(); it != base2lba.end(); ++it) {
      // because of sorted map, we can get the logical addresses sequentially.
      uint64_t lba_base = it->first;
      std::vector<std::tuple<uint64_t, uint64_t>> lbas = it->second;

      for (size_t i = 0; i < lbas.size(); i++) {
        uint64_t lba = std::get<0>(lbas[i]);
        uint64_t pba = std::get<1>(lbas[i]);
        uint32_t offset = lba % (this->zcap);

        auto exists = this->data_map.find(lba_base);
        if (exists != this->data_map.end()) {
          // GC
        }

        void *rdata = calloc(1, bsize);
        ss_nvme_read(this->fd, this->nsid, pba, 0, 0, 0, 0, 0, 0, this->bsize,
                     rdata, 0, nullptr);

        // Write invalid data until offset.
        for (; curr_data_wp_lba < this->data_wp + offset; curr_data_wp_lba++) {
          ss_nvme_write(this->fd, this->nsid, curr_data_wp_lba, 0, 0, 0, 0, 0,
                        0, 0, 0, nullptr, 0, nullptr);
        }
        ss_nvme_write(this->fd, this->nsid, curr_data_wp_lba++, 0, 0, 0, 0, 0,
                      0, 0, bsize, rdata, 0, nullptr);
        // puts("");
        free(rdata);
      }

      this->data_map[lba_base] = this->data_wp;
      this->data_wp += zcap;
    }

    // reset log zones.
    for (int32_t i = 0; i < this->log_zones - 1; i++) {
      ss_zns_device_zone_reset(this->fd, this->nsid, this->zcap + i * zcap);
    }
    this->log_map.clear();
    this->log_wp = this->zcap;
  }

 private:
  Ftlmap log_map;
  Ftlmap data_map;
  // return physical page address from log map.
  bool get_ppa(uint64_t lba, uint64_t *ppa) {
    auto ret = this->log_map.find(lba);
    if (ret == this->log_map.end()) {
      return false;
    } else {
      *ppa = ret->second;
      return true;
    }
  }

  // return physical block address from data map.
  bool get_pba(uint64_t lba, uint64_t *pba) {
    auto ret = this->data_map.find(lba);
    if (ret == this->data_map.end()) {
      return false;
    } else {
      *pba = ret->second;
      return true;
    }
  }
};

extern "C" {

// TODO(valentijn): Implement this function
int deinit_ss_zns_device(struct user_zns_device *my_dev) {
  int ret = -ENOSYS;
  // cppcheck-suppress cstyleCast
  FTL *ftl = (FTL *)my_dev->_private;
  ftl->~FTL();

  // this is to supress gcc warnings, remove it when you complete this function
  UNUSED(my_dev);
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
    print_nvme_error(ret);
    return ret;
  }

  ret = nvme_identify_ns(fd, nsid, &ns);
  if (ret) {
    printf("ERROR: failed to retrieve the nsid %d \n", ret);
    print_nvme_error(ret);
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

  uint64_t zcap;
  uint32_t nr;
  get_zns_zone_info(fd, nsid, &zcap, &nr);
  FTL ftl = FTL(fd, MDTS_SIZE, nsid, zcap, params->log_zones, lba_size_in_use);
  free(path);
  close(sysfd);

  // reset all zones.
  uint64_t end = nr * zcap;
  for (uint64_t i = 0; i < end; i += zcap) {
    int r = ss_zns_device_zone_reset(fd, nsid, i);
    if (r != 0) {
      printf("failed to reset all zones.");
      return r;
    }
  }

  struct zns_device_testing_params tparams {
    .zns_lba_size = lba_size_in_use,
    .zns_zone_capacity = (uint32_t)zcap * lba_size_in_use,
    // cppcheck-suppress unreadVariable
        .zns_num_zones = nr,
  };
  struct user_zns_device device {
    .lba_size_bytes = lba_size_in_use,
    .capacity_bytes =
        (ns.ncap - ftl.log_zones * ftl.zcap) *
        lba_size_in_use,  // ZNS capacity - log zones (includes metadata).
        .tparams = tparams, ._private = (void *)&ftl,
  };
  *my_dev = &device;

  munmap(regs, getpagesize());
  return 0;
}

int zns_udevice_read(struct user_zns_device *my_dev, uint64_t address,
                     void *buffer, uint32_t size) {
  // Check log map firstly beacuse of fresh data, then check data map, one page
  // size at a time (to avoid overwrite). e.g. full sequantially writes on
  // address 0x0 to make a data map entry, then 1 page write on 0x2, the data in
  // log map will be newer than data map.
  // cppcheck-suppress cstyleCast
  FTL *flt = (FTL *)my_dev->_private;
  uint32_t pages_num = size / my_dev->lba_size_bytes;
  address /= my_dev->lba_size_bytes;
  // maximum reading mdts.
  uint32_t mdts = flt->mdts_size / my_dev->lba_size_bytes;
  // store <start address, nlb>.
  std::pair<uint64_t, uint64_t> adjacent_phas = std::pair<uint64_t, uint64_t>();
  std::vector<std::pair<uint64_t, uint32_t>> phas;

  uint64_t pa;
  uint64_t prev_addr = 0;
  for (uint64_t i = 0; i < pages_num; i++) {
    bool contains = flt->get_pa(address + i, &pa);
    if (contains) {
      if (prev_addr == 0) {
        // init.
        adjacent_phas.first = pa;
        adjacent_phas.second = 1;
      } else if (pa - 1 == prev_addr && adjacent_phas.second < mdts) {
          adjacent_phas.second += 1;
      } else {
          phas.push_back(adjacent_phas);
          adjacent_phas = std::pair<uint64_t, uint64_t>();
          adjacent_phas.first = pa;
          adjacent_phas.second = 1;
      }
      prev_addr = pa;
    } else {
      // Invalid logical page address.
      return -1;
    }
  }
  if (phas.size() < mdts) {
    // Add the last phas.
    phas.push_back(adjacent_phas);
  }

  uint64_t data_ptr = (uint64_t) buffer;
  for (uint32_t i = 0; i < phas.size(); i++) {
    // should be optimized later.
    uint32_t block_size = my_dev->lba_size_bytes;
    uint32_t nlb = phas[i].second;
    uint64_t pa = phas[i].first;
    int ret = ss_nvme_read(flt->fd, flt->nsid, pa, nlb-1, 0, 0, 0, 0, 0, nlb * block_size,
                 (void*) data_ptr, 0, nullptr);
    if (ret != 0) {
      return ret;
    }
    data_ptr = data_ptr + nlb * block_size;
  }
  phas.clear();
  return 0;
}

/*
NVMe ZNS Identify Controller:
zasl    : 5     # reported as pow(2, n)

mtds is reported as pow(2, n) either, but mtds is 6 (7-1).

zasl is smaller than the mdts_size, then nvme_write is better for large chuck
data write.

I propose design a queue in the future to advoid multi-threaded data races.
*/
int zns_udevice_write(struct user_zns_device *my_dev, uint64_t address,
                      void *buffer, uint32_t size) {
  // uint64_t *ret_wp;
  // cppcheck-suppress cstyleCast
  FTL *flt = (FTL *)my_dev->_private;

  // unused
  //  uint32_t pages_num = size / my_dev->lba_size_bytes;
  uint16_t total_nlb =
      size / my_dev->lba_size_bytes;  // size is the multiple of lba_size.
  uint16_t max_nlb_per_round = flt->mdts_size / my_dev->lba_size_bytes;
  address /= my_dev->lba_size_bytes;

  if ((flt->log_wp + total_nlb) > flt->log_end) {
    // flt->merge_zones();
  }

  if (size == flt->zcap) {
    // fit perfectly in one data zone.
    int ret = ss_nvme_write(flt->fd, flt->nsid, flt->data_wp, total_nlb - 1, 0,
                            0, 0, 0, 0, 0, size, buffer, 0, nullptr);
    if (ret != 0) {
      return ret;
    }
    flt->insert_lba_data(address, flt->log_wp);
  } else if (size <= flt->mdts_size) {
    // TODO(valentijn): Try multithreaded append.
    uint64_t wp = flt->log_wp;
    int ret = ss_nvme_write(flt->fd, flt->nsid, wp, total_nlb - 1, 0, 0, 0, 0,
                            0, 0, size, buffer, 0, nullptr);
    if (ret != 0) {
      return ret;
    }
    flt->log_wp = wp + total_nlb;
    for (uint64_t i = 0; i < total_nlb; i++) {
      flt->insert_lba_log(address + i, wp + i);
    }
  } else {
    // divide them into many writes in mdts_size.
    uint64_t wp = flt->log_wp;
    uint64_t i;
    uint64_t data_len = max_nlb_per_round * my_dev->lba_size_bytes;
    void *buffer_ptr;
    for (i = 0; i < total_nlb; i += max_nlb_per_round) {
      buffer_ptr = (void *)((uint64_t)buffer + i * my_dev->lba_size_bytes);
      int ret =
          ss_nvme_write(flt->fd, flt->nsid, flt->log_wp, max_nlb_per_round - 1,
                        0, 0, 0, 0, 0, 0, data_len, buffer_ptr, 0, nullptr);
      if (ret != 0) {
        return ret;
      }
      flt->log_wp = flt->log_wp + max_nlb_per_round;
    }
    if (i > total_nlb) {
      // write remaining blocks.
      uint64_t ream_blocks = (total_nlb - (i - max_nlb_per_round));
      buffer_ptr = (void *)((uint64_t)buffer + i * my_dev->lba_size_bytes);
      data_len = ream_blocks * my_dev->lba_size_bytes;
      int ret =
          ss_nvme_write(flt->fd, flt->nsid, flt->log_wp, ream_blocks - 1, 0, 0,
                        0, 0, 0, 0, data_len, buffer_ptr, 0, nullptr);

      if (ret != 0) {
        return ret;
      }

      flt->log_wp += ream_blocks;
    }

    for (uint64_t nlb = 0; nlb < total_nlb; nlb++) {
      flt->insert_lba_data(address + nlb, wp + nlb);
    }
  }

  return 0;
}
}
