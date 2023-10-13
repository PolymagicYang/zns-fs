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

#include "datazone.hpp"

#include <cstdint>

#include "libnvme.h"

ZNSDataZone::ZNSDataZone(const int zns_fd, const uint32_t nsid,
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
  this->zns_type = zns_type;
  this->model = model;
  this->position = position;
  this->base = position;
  this->slba = slba;
  this->lba_size = lba_size;
  this->mdts_size = mdts_size;

  this->block_map = std::vector<int>();
  for (uint16_t i = 0; i < this->capacity; i++) {
    this->block_map.push_back(0);
  }
}

// TODO(valentijn) update so it throws exceptions
inline int ZNSDataZone::send_management_command(
    const enum nvme_zns_send_action action, const bool select_all) const {
  int ret = ss_nvme_zns_mgmt_send(this->zns_fd, this->nsid, this->slba,
                                  select_all, action, 0, NULL);
  if (ret != 0) {
    print_nvme_error("send_management_command", ret);
  }
  return ret;
}

inline int ZNSDataZone::send_management_command(
    const enum nvme_zns_send_action action) const {
  return this->send_management_command(action, false);
}

inline int ZNSDataZone::reset_all_zones() const {
  return this->send_management_command(NVME_ZNS_ZSA_RESET, true);
}

int ZNSDataZone::get_index() {
  int ret = this->zone_id;
  return ret;
}

int ZNSDataZone::reset() {
  int ret = this->reset_zone();
  this->position = this->base;

  // Remove all blocks from the memory of this zone
  for (uint16_t i = 0; i < this->capacity; i++) {
    this->block_map[i] = 0;
  }
  return ret;
}

bool ZNSDataZone::is_full() {
  bool ret = this->get_current_capacity() == 0;
  return ret;
}

/*
WARN: unsafe function, read and write will call this function with their own
lock. Use this method with a lock!
*/
uint32_t ZNSDataZone::get_current_capacity() const {
  return this->capacity + this->base - this->position;
}

uint64_t ZNSDataZone::get_wp() {
  uint64_t ret = this->position;
  return ret;
}

int ZNSDataZone::close_zone(void) const {
  return send_management_command(NVME_ZNS_ZSA_CLOSE);
}

int ZNSDataZone::open_zone(void) const {
  return send_management_command(NVME_ZNS_ZSA_OPEN);
}

int ZNSDataZone::finish_zone(void) const {
  return send_management_command(NVME_ZNS_ZSA_FINISH);
}

int ZNSDataZone::reset_zone(void) {
  this->position = this->slba;
  return send_management_command(NVME_ZNS_ZSA_RESET);
}

std::ostream &operator<<(std::ostream &os, ZNSDataZone const &tc) {
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
            << "Metadata: " << state << " | Data Zone | " << model << std::endl;
}

int ZNSDataZone::ss_sequential_write(const void *buffer,
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

int ZNSDataZone::invalidate_block(uint16_t index) {
  // See if the physical adress exists, else print an error and move on.
  // This can happen if the cache at the FTL is invalid or if it has
  // done a multiple region write.
  if (this->block_map[index]) {
    std::cerr << "Error: Block " << index << " does not exist in "
              << this->zone_id << std::endl;
    return -1;
  }

  // Store the invalid zone in the system
  this->block_map[index] = 0;

  return 0;
}

// used for data zone, should be merged in the future.
// return true if there's no data conflicts else false.
bool ZNSDataZone::write_until(void *buffer, uint32_t size, uint32_t index) {
  // index must ok, because these data are mod by zcap.
  // size is the multiple of lba_size.
  uint64_t curr_wp = this->position - this->base;
  if (curr_wp > index) {
    // already write, should invalidate this one.
    return false;
  }

  uint16_t num_blocks_until = (index - curr_wp);
  // write invalid blocks until index.
  if (num_blocks_until != 0) {
    int ret = ss_nvme_write_zeros(this->zns_fd, this->nsid, this->position,
                                  num_blocks_until - 1, 0, 0, 0, 0);
    if (ret != 0) {
      return false;
    }
    this->position += num_blocks_until;
  }

  int write_t = ss_nvme_write(this->zns_fd, this->nsid, this->position, 0, 0, 0,
                              0, 0, 0, 0, size, buffer, 0, nullptr);
  this->block_map[this->position - this->base] = 1;
  if (write_t != 0) {
    return false;
  }
  this->position += 1;
  return true;
}

bool ZNSDataZone::can_write(uint32_t index) {
  bool ret;
  uint64_t curr_index = this->position - this->base;
  if (curr_index > index) {
    ret = false;
  } else {
    ret = true;
  }
  return ret;
}

uint32_t ZNSDataZone::read(const uint64_t pa, const void *buffer, uint32_t size,
                           uint32_t *read_size) {
  if (pa + size / this->lba_size > this->base + this->capacity) {
    // cross boundary read.
    size = (this->base + this->capacity - pa) * this->lba_size;
  }
  uint32_t nlb = size / this->lba_size;
  *read_size = size;

  int read_ret =
      ss_nvme_read(this->zns_fd, this->nsid, pa, nlb - 1, 0, 0, 0, 0, 0,
                   nlb * this->lba_size, (void *)buffer, 0, nullptr);
  if (read_ret != 0) {
    return read_ret;
  }

  return 0;
}

/*
return the size of the inserted buffer.
*/
uint32_t ZNSDataZone::write_nounce(const void *buffer, uint32_t size,
                                   uint32_t *write_size) {
  uint32_t max_writes = this->get_current_capacity() * this->lba_size;
  *write_size = (size > max_writes) ? max_writes : size;

  // size is the multiple of lba_size.
  uint16_t total_nlb = size / this->lba_size;
  uint16_t max_nlb_per_round = this->mdts_size / this->lba_size;

  if (size <= this->mdts_size) {
    int ret =
        ss_nvme_write(this->zns_fd, this->nsid, this->position, total_nlb - 1,
                      0, 0, 0, 0, 0, 0, size, (void *)buffer, 0, nullptr);
    if (ret != 0) {
      return ret;
    }
    this->position += total_nlb;
  } else {
    int ret = ss_sequential_write(buffer, max_nlb_per_round, total_nlb);
    if (ret != 0) return ret;
  }

  return 0;
}

/*
return the size of the inserted buffer.
*/
uint32_t ZNSDataZone::write(const void *buffer, uint32_t size,
                            uint32_t *write_size) {
  uint32_t max_writes = this->get_current_capacity() * this->lba_size;
  *write_size = (size > max_writes) ? max_writes : size;

  // size is the multiple of lba_size.
  uint16_t total_nlb = *write_size / this->lba_size;
  uint16_t max_nlb_per_round = this->mdts_size / this->lba_size;
  uint16_t init_index = this->position - this->base;

  if (size <= this->mdts_size) {
    int ret =
        ss_nvme_write(this->zns_fd, this->nsid, this->position, total_nlb - 1,
                      0, 0, 0, 0, 0, 0, size, (void *)buffer, 0, nullptr);
    if (ret != 0) {
      return ret;
    }
    this->position += total_nlb;
  } else {
    int ret = ss_sequential_write(buffer, max_nlb_per_round, total_nlb);
    if (ret != 0) return ret;
  }

  // mark valid until the current index.
  for (uint16_t i = init_index; i < this->position - this->base; i++) {
    this->block_map[i] = 1;
  }

  return 0;
}

// WARN: Only for GC, don't use it for other purposes.
void ZNSDataZone::copy_range(ZNSDataZone *other, uint16_t start, uint16_t end) {
  if (end <= start) {
    return;
  }
  uint32_t size = (end - start) * this->lba_size;
  char buffer[size];
  uint32_t read_size;
  // TODO(Zhiyang): Implement ss_nvme_copy instead of copying & pasting.
  // TODO(Zhiyang): error handling.
  this->read(this->base + start, &buffer, size, &read_size);
  other->write(&buffer, size, &read_size);

  return;
}

bool ZNSDataZone::exists(uint64_t lba) {
  uint64_t index = lba % this->capacity;
  bool ret = this->block_map[index];
  return ret;
}

std::vector<ZNSDataZone> create_datazones(const int zns_fd, const uint32_t nsid,
                                          const uint64_t lba_size,
                                          const uint64_t mdts_size) {
  // TODO(valentijn): don't hard code this please
  struct nvme_zone_report *zns_report =
      (struct nvme_zone_report *)calloc(1, 0x1000);
  uint64_t zcap;
  uint64_t nr;
  get_zns_zone_info(zns_fd, nsid, &nr, &zcap, zns_report);

  // Go through all the reprots and turn them into zones
  std::vector<ZNSDataZone> zones = std::vector<ZNSDataZone>();
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

    zones.push_back(ZNSDataZone(zns_fd, nsid, i, capacity, capacity, zstate,
                                ztype, zone_slba, HostManaged, write_pointer,
                                lba_size, mdts_size));
  }

  // Reset all the zones in one go so that we are in a valid initial state
  zones.at(0).reset_all_zones();
  free(zns_report);
  return zones;
}
