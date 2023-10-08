#ifndef STOSYS_PROJECT_LOG_ZNS_ZONE_H
#define STOSYS_PROJECT_LOG_ZNS_ZONE_H
#include <cstdint>
#include <pthread.h>
#pragma once

#include "zone.hpp"

class ZNSLogZone {
 public:
  uint32_t zone_id;
  uint32_t nsid;

  ZNSLogZone(const int zns_fd, const uint32_t nsisd, const uint32_t zone_id,
          const uint64_t size, const uint64_t capacity,
          const enum ZoneState state, const enum ZoneZNSType zns_type,
          const uint64_t slba, 
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
  enum ZoneModel model;

  /** Calculates the current capacity of the block. */
  uint32_t get_current_capacity() const;

  /** Gets a block from the zone based on the block id. */
  uint32_t read(const uint64_t lba, const void *buffer, uint32_t size,
                uint32_t *read_size);
  uint32_t write(void *buffer, uint32_t size, uint32_t *write_size, uint64_t lba);

  /** Reset the write pointer to the start. */
  int reset_zone(void);

  /** Open the zone explicitely so more resources are allocated. */
  int open_zone(void) const;

  /** Explicitely close a zone and free the resources. */
  int close_zone(void) const;

  /** Close the zone so that writes cannot be performed until it is reset. */
  int finish_zone(void) const;

  friend std::ostream &operator<<(std::ostream &os, ZNSLogZone const &tc);

  friend std::vector<ZNSLogZone> create_logzones(const int zns_fd,
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

  /** Map of the physical addresses to the buffer and state */
  ZoneMap block_map;

  /** Set the block to being free based on the physical address */  
  int invalidate_block(const uint64_t pa);

  /** Gets the blocks that are still valid */
  // TODO(someone): change name to get_valid_blocks
  std::vector<ZNSBlock *> get_nonfree_blocks();

  /** Zone mutex for the FTL::write and Calliope::reap methods */
  pthread_mutex_t zone_mutex = PTHREAD_MUTEX_INITIALIZER;

  /** Resets all the zones in the device. */
  int reset_all_zones();
  
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

#endif
