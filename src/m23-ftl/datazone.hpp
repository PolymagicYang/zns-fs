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

#ifndef STOSYS_PROJECT_DATA_ZONE_H
#define STOSYS_PROJECT_DATA_ZONE_H
#pragma once
#include <libnvme.h>
#include <nvme/ioctl.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "../common/nvmewrappers.h"
#include "znsblock.hpp"
#include "zone.hpp"

class ZNSDataZone {
 public:
  uint32_t zone_id;
  uint32_t nsid;

  ZNSDataZone(const int zns_fd, const uint32_t nsisd, const uint32_t zone_id,
              const uint64_t size, const uint64_t capacity,
              const enum ZoneState state, const enum ZoneZNSType zns_type,
              const uint64_t slba, const enum ZoneModel model,
              const uint64_t position, const uint64_t lba_size,
              const uint64_t mdts_size);

  /** Zone capacity is the total optimized number of blocks in the
          region. This is always less than the zone size. */
  uint64_t capacity;

  /** Zone size is the maximum amount of blocks that can be stored
          in the zone. Typically not useful for our purposes. */
  uint64_t size;

  /** Starting logical block address of the zone. Typically used for zone
   * commands. */
  uint64_t slba;

  enum ZoneState state;
  enum ZoneModel model;
  enum ZoneZNSType zns_type;

  /** Calculates the current capacity of the block. */
  uint32_t get_current_capacity() const;

  /** Gets a block from the zone based on the block id. */
  uint32_t read(const uint64_t lba, const void *buffer, uint32_t size,
                uint32_t *read_size);

  uint32_t write(const void *buffer, uint32_t size, uint32_t *write_size);

  uint32_t write_nounce(const void *buffer, uint32_t size,
                        uint32_t *write_size);
  /** Reset the write pointer to the start. */
  int reset_zone(void);

  /** Open the zone explicitely so more resources are allocated. */
  int open_zone(void) const;

  /** Explicitely close a zone and free the resources. */
  int close_zone(void) const;

  /** Close the zone so that writes cannot be performed until it is reset. */
  int finish_zone(void) const;

  friend std::ostream &operator<<(std::ostream &os, ZNSDataZone const &tc);

  friend std::vector<ZNSDataZone> create_datazones(const int zns_fd,
                                                   const uint32_t nsid,
                                                   const uint64_t lba_size,
                                                   const uint64_t mdts_size);

  bool write_until(void *buffer, uint32_t size, uint32_t index);

  /** Returns the number of blocks which are still valid */
  uint64_t get_alive_capacity() const;

  /** Gets the index or id of the zone*/
  int get_index();

  /** Resets the zone and performs additional checking compared to
   * ZNSZone::reset_zone */
  int reset();

  /** Checks if the zone is full*/
  bool is_full();

  /** Gets the write pointer of the zone */
  uint64_t get_wp();

  /** Write pointer */
  uint64_t position;

  /** Zone Logical Block Address or the lowest addressable point */
  uint64_t base;

  /** Size of a block */
  uint64_t lba_size;

  /** Maximum transfer size */
  uint64_t mdts_size;

  /** Map of the physical addresses to the buffer and state */
  std::vector<int> block_map;

  /** Set the block to being free based on the physical address */
  int invalidate_block(uint16_t index);

  /** copy from index [start: end) to target. */
  void copy_range(ZNSDataZone *target, uint16_t start, uint16_t end);

  /** Gets the blocks that are still valid */
  // TODO(someone): change name to get_valid_blocks
  std::vector<physaddr_t> get_nonfree_blocks() const;

  /** Zone mutex for the FTL::write and Calliope::reap methods */
  pthread_mutex_t zone_mutex = PTHREAD_MUTEX_INITIALIZER;

  /** Resets all the zones in the device. */
  inline int reset_all_zones() const;

  /** Check the existence of the device */
  bool exists(uint64_t lba);

  /** Return true if this index can be written directly. */
  bool can_write(uint32_t index);

 private:
  int zns_fd;

  /** Write to the device in a sequential manner */
  int ss_sequential_write(const void *buffer, const uint16_t max_nlb_per_round,
                          const uint16_t total_nlb);

  inline int send_management_command(
      const enum nvme_zns_send_action action) const;
  /** Convenience function to send a zone management command. */
  inline int send_management_command(const enum nvme_zns_send_action action,
                                     const bool select_all) const;
};

std::ostream &operator<<(std::iostream &os, ZNSDataZone const &tc);

#endif
