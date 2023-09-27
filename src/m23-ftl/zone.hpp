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

#ifndef STOSYS_PROJECT_ZNS_ZONE_H
#define STOSYS_PROJECT_ZNS_ZONE_H
#pragma once

#include <libnvme.h>
#include <pthread.h>

#include <cstdint>
#include <iostream>
#include <map>
#include <vector>
#include <mutex>

#include "znsblock.hpp"

using BlockMap = std::map<uint64_t, ZNSBlock>;

struct ZoneMap {
  pthread_rwlock_t lock;
  BlockMap map;
};

#define RESET_ZONE true

enum ZoneState {
  Empty = 0x0,
  Full = 0x1,
  ImplicitOpen = 0x2,
  ExplicitOpen = 0x3,
  Closed = 0x4,
  ReadOnly = 0xD,
  Offline = 0xF,
  Unknown = 16,
  Bad = 224,
};

/** Type that this zone is allocated in our system. */
enum ZoneFTLType { Log, Data };

/** Interface used for this zone.*/
enum ZoneModel {
  /** Sequential work loads and full control for the host. */
  HostManaged,
  /** Backwards compatible with block devices so that random writes
          can be issued. */
  HostAware
};

/** Zone type as defined by the ZNS standard. */
enum ZoneZNSType {
  /** Similar to block devices. Accepts random writes and does not
          have write pointers. */
  Convential,
  /** Random writes are allowed, but has a write pointer.*/
  SequentialWritePreferred,
  /** Can only be used sequentially with a write pointer. Write
          commands must be aligned with the write pointer. */
  SequentialWriteRequired
};

class ZNSZone {
 public:
  uint32_t zone_id;
  uint32_t nsid;

  ZNSZone(const int zns_fd, const uint32_t nsisd, const uint32_t zone_id,
          const uint64_t size, const uint64_t capacity,
          const enum ZoneState state, const enum ZoneZNSType zns_type,
          const uint64_t slba, const enum ZoneFTLType ftl_type,
          const enum ZoneModel model, const uint64_t position,
          const uint64_t lba_size, const uint64_t mdts_size);

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
  enum ZoneFTLType ftl_type;
  enum ZoneModel model;

  /** Calculates the current capacity of the block. */
  uint32_t get_current_capacity() const;

  /** Gets a block from the zone based on the block id. */
  uint32_t read(const uint64_t lba, const void *buffer, uint32_t size,
                uint32_t *read_size);
  uint32_t write(void *buffer, uint32_t size, uint32_t *write_size);

  /** Reset the write pointer to the start. */
  int reset_zone(void);

  /** Open the zone explicitely so more resources are allocated. */
  int open_zone(void) const;

  /** Explicitely close a zone and free the resources. */
  int close_zone(void) const;

  /** Close the zone so that writes cannot be performed until it is reset. */
  int finish_zone(void) const;

  friend std::ostream &operator<<(std::ostream &os, ZNSZone const &tc);

  friend std::vector<ZNSZone> create_zones(const int zns_fd,
                                           const uint32_t nsid,
                                           const uint64_t lba_size,
                                           const uint64_t mdts_size);

  /** Returns the number of blocks which are still valid */
  uint64_t get_alive_capacity() const;

  /** Gets the index or id of the zone*/
  int get_index();
  
  /** Resets the zone and performs additional checking compared to ZNSZone::reset_zone */
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

  /** Lock of the region */
  pthread_rwlock_t lock = PTHREAD_RWLOCK_INITIALIZER;

  /** Map of the physical addresses to the buffer and state */
  ZoneMap block_map;

  /** Set the block to being free based on the physical address */  
  int invalidate_block(const uint64_t pa);

  /** Gets the blocks that are still valid */
  // TODO(someone): change name to get_valid_blocks
  std::vector<physaddr_t> get_nonfree_blocks() const;

  /** Zone mutex for the FTL::write and Calliope::reap methods */
  pthread_mutex_t zone_mutex = PTHREAD_MUTEX_INITIALIZER;
  
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
  /** Resets all the zones in the device. */
  inline int reset_all_zones() const;
};

std::ostream &operator<<(std::iostream &os, ZNSZone const &tc);

/** Get the information of the zone and return them as reports */
int get_zns_zone_info(const int fd, const int nsid, uint64_t *zcap,
                      uint64_t *nr, struct nvme_zone_report *zns_report);

/** Create a list of zones in the device */
std::vector<ZNSZone> create_zones(const int zns_fd, const uint32_t nsid,
                                  const uint64_t lba_size,
                                  const uint64_t mdts_size);

#endif
