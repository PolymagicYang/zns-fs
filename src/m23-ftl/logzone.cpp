/* MIT License
Copyright (c) 2021 - current
Authors:  Valentijn Dymphnus van de Beek & Zhiyang Wang
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

#include "logzone.hpp"

#include <libnvme.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <cassert>
#include <cstddef>
#include <cstdint>

#include "../common/nvmewrappers.h"
#include "znsblock.hpp"

ZNSLogZone::ZNSLogZone(const int zns_fd, const uint32_t nsid,
                       const uint32_t zone_id, const uint64_t size,
                       const uint64_t capacity, const enum ZoneState state,
                       const enum ZoneZNSType zns_type, const uint64_t slba,
                       const enum ZoneModel model, const uint64_t position,
                       const uint64_t lba_size, const uint64_t mdts_size) {
  this->zns_fd = zns_fd;
  this->nsid = nsid;
  this->zone_id = zone_id;
  this->size = size;
  this->capacity = capacity;
  this->state = state;
  this->model = model;
  this->position = position;
  this->base = slba;
  this->slba = slba;
  this->lba_size = lba_size;
  this->mdts_size = mdts_size;

  this->block_map =
      ZoneMap{.lock = PTHREAD_RWLOCK_INITIALIZER, .map = BlockMap()};
}

// TODO(valentijn) update so it throws exceptions
inline int ZNSLogZone::send_management_command(
    const enum nvme_zns_send_action action, const bool select_all) const {
  int ret = ss_nvme_zns_mgmt_send(this->zns_fd, this->nsid, this->slba,
                                  select_all, action, 0, NULL);
  if (ret != 0) {
    print_nvme_error("send_management_command", ret);
  }
  return ret;
}

inline int ZNSLogZone::send_management_command(
    const enum nvme_zns_send_action action) const {
  return this->send_management_command(action, false);
}

int ZNSLogZone::reset_all_zones() {
  return this->send_management_command(NVME_ZNS_ZSA_RESET, true);
}

int ZNSLogZone::get_index() {
  int ret = this->zone_id;
  return ret;
}

int ZNSLogZone::reset() {
  int ret = ss_device_zone_reset(this->zns_fd, this->nsid, this->base);

  // Remove all blocks from the memory of this zone
  this->block_map.map.clear();
  this->position = this->base;
  return ret;
}

bool ZNSLogZone::is_full() {
  bool ret = this->get_current_capacity() == 0;
  return ret;
}

/*
WARN: unsafe function, read and write will call this function with their own
lock. Use this method with a lock!
*/
uint32_t ZNSLogZone::get_current_capacity() const {
  // printf("base is %d, cap is %d, position is %d\n", this->base, this->capacity, this->position);
  return this->capacity + this->base - this->position;
}

uint64_t ZNSLogZone::get_wp() {
  uint64_t ret = this->position;
  return ret;
}

int ZNSLogZone::close_zone(void) const {
  return send_management_command(NVME_ZNS_ZSA_CLOSE);
}

int ZNSLogZone::open_zone(void) const {
  return send_management_command(NVME_ZNS_ZSA_OPEN);
}

int ZNSLogZone::finish_zone(void) const {
  return send_management_command(NVME_ZNS_ZSA_FINISH);
}

int ZNSLogZone::reset_zone(void) {
  int ret = send_management_command(NVME_ZNS_ZSA_RESET);
  this->position = this->slba;
  return ret;
}

std::ostream &operator<<(std::ostream &os, ZNSLogZone const &tc) {
  const char *state = get_state_text(tc.state);
  const char *model = get_zone_model_text(tc.model);

  return os << "Zone " << std::dec << tc.zone_id << std::endl
            << "====================================" << std::endl
            << "Starting Logical Block Address: "
            << "0x" << std::hex << tc.slba << std::dec << std::endl
            << "Usage: " << tc.get_current_capacity() << " out of " << tc.size
            << std::endl
            << "Write pointer: "
            << "0x" << std::hex << tc.position << std::endl
            << "Metadata: " << state << " | "
            << "Log"
            << " | " << model << std::endl;
}

int ZNSLogZone::ss_sequential_write(const void *buffer,
                                    const uint16_t max_nlb_per_round,
                                    const uint16_t total_nlb) {
  uint64_t i;
  uint64_t data_len = max_nlb_per_round * this->lba_size;
  void *buffer_ptr;
  for (i = 0; i < total_nlb; i += max_nlb_per_round) {
    buffer_ptr = (void *)((uint64_t)buffer + i * this->lba_size);
    int ret = ss_nvme_write(this->zns_fd, this->nsid, this->position,
                            max_nlb_per_round - 1, 0, 0, 0, 0, 0, 0, data_len,
                            buffer_ptr, 0, nullptr);
    if (ret != 0) {
      return ret;
    }
    this->position += max_nlb_per_round;
  }
  if (i > total_nlb) {
    // write remaining blocks.
    uint64_t ream_blocks = (total_nlb - (i - max_nlb_per_round));
    buffer_ptr = (void *)((uint64_t)buffer + i * this->lba_size);
    data_len = ream_blocks * this->lba_size;
    int ret =
        ss_nvme_write(this->zns_fd, this->nsid, this->position, ream_blocks - 1,
                      0, 0, 0, 0, 0, 0, data_len, buffer_ptr, 0, nullptr);
    if (ret != 0) {
      return ret;
    }
    this->position += ream_blocks;
  }

  return 0;
}

// TODO(someone): this is copying the entire block map every time
uint64_t ZNSLogZone::get_alive_capacity() const {
  BlockMap *map = (BlockMap *)&this->block_map.map;

  uint64_t alive_count = 0;
  for (BlockMap::iterator it = map->begin(); it != map->end(); ++it) {
    if (it->second.valid) alive_count++;
  }

  return alive_count;
}

int ZNSLogZone::invalidate_block(const uint64_t pa) {
  // See if the physical adress exists, else print an error and move on.
  // This can happen if the cache at the FTL is invalid or if it has
  // done a multiple region write.
  if (this->block_map.map.count(pa) != 1) {
    std::cerr << "Error: Block " << pa << " does not exist in " << this->zone_id
              << std::endl;
    return -1;
  }

  // Store the invalid zone in the system
  this->block_map.map[pa].valid = false;

  return 0;
}

/*
return the size of the inserted buffer.
*/
uint32_t ZNSLogZone::write(void *buffer, uint32_t size, uint32_t *write_size,
                           uint64_t lba) {
  uint32_t max_writes = this->get_current_capacity() * this->lba_size;
  *write_size = (size > max_writes) ? max_writes : size;

  // size is the multiple of lba_size.
  uint16_t total_nlb = *write_size / this->lba_size;
  if (size < this->lba_size) {
    total_nlb = 1;
  }
  uint16_t max_nlb_per_round = this->mdts_size / this->lba_size;
  uint64_t write_base = this->position;

  if (size <= this->mdts_size) {
    // This values cause bad things to happen
    int ret = ss_nvme_write(this->zns_fd, this->nsid, this->position,
                            total_nlb - 1, 0, 0, 0, 0, 0, 0, *write_size,
                            (void *)buffer, 0, nullptr);
    if (ret != 0) {
      return ret;
    }
    this->position += total_nlb;
  } else {
    std::cout << "sequential" << std::endl;
    int ret = ss_sequential_write(buffer, max_nlb_per_round, total_nlb);
    if (ret != 0) return ret;
  }

  for (uint64_t i = 0, address = write_base; i < total_nlb; i++) {
    uint64_t pa = address + i;
    uint64_t local_lba = lba + i * this->lba_size;
    this->block_map.map[pa] = {.address = pa,
                               .logical_address = local_lba,
                               .valid = true};
  }

  return 0;
}

uint32_t ZNSLogZone::read(const uint64_t pa, const void *buffer, uint32_t size,
                          uint32_t *read_size) {
  if (pa + size / this->lba_size > this->base + this->capacity) {
    // cross boundary read.
    size = (this->base + this->capacity - pa) * this->lba_size;
  }
  uint32_t nlb = size / this->lba_size;
  if (nlb == 0) {
    nlb = 1;
  }
  *read_size = size;

  int read_ret =
      ss_nvme_read(this->zns_fd, this->nsid, pa, nlb - 1, 0, 0, 0, 0, 0,
                   nlb * this->lba_size, (void *)buffer, 0, nullptr);
  if (read_ret != 0) {
    return read_ret;
  }

  return 0;
}

std::vector<ZNSLogZone> create_logzones(const int zns_fd, const uint32_t nsid,
                                        const uint64_t lba_size,
                                        const uint64_t mdts_size) {
  // TODO(valentijn): don't hard code this please
  struct nvme_zone_report *zns_report =
      (struct nvme_zone_report *)calloc(1, 0x1000);
  uint64_t zcap;
  uint64_t nr;
  get_zns_zone_info(zns_fd, nsid, &nr, &zcap, zns_report);

  // Go through all the reprots and turn them into zones
  std::vector<ZNSLogZone> zones = std::vector<ZNSLogZone>();
  for (uint32_t i = 0; i < zns_report->nr_zones; i++) {
    struct nvme_zns_desc current = zns_report->entries[i];

    const enum ZoneZNSType ztype = static_cast<ZoneZNSType>(current.zt);
    const enum ZoneState zstate = static_cast<ZoneState>(current.zs);
    // TODO(valentijn): zone attributes (p.28 ZNS Command specification)
    const uint64_t capacity = le64_to_cpu(current.zcap);
    const uint64_t zone_slba = le64_to_cpu(current.zslba);
    uint64_t write_pointer = le64_to_cpu(current.wp);

    if (RESET_ZONE) {
      write_pointer = zone_slba;
    }

    zones.push_back(ZNSLogZone(zns_fd, nsid, i, capacity, capacity, zstate,
                               ztype, zone_slba, HostManaged, write_pointer,
                               lba_size, mdts_size));
  }

  // Reset all the zones in one go so that we are in a valid initial state
  zones.at(0).reset_all_zones();
  free(zns_report);
  return zones;
}

std::vector<ZNSBlock *> ZNSLogZone::get_nonfree_blocks() {
  BlockMap *map = (BlockMap *)&this->block_map.map;
  std::vector<ZNSBlock *> nonfree_blocks;

  for (BlockMap::iterator it = map->begin(); it != map->end(); ++it) {
    if (it->second.valid) nonfree_blocks.push_back(&it->second);
  }
  return nonfree_blocks;
}
