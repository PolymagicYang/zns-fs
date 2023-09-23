#ifndef STOSYS_PROJECT_ZNS_ZONE_H
#define STOSYS_PROJECT_ZNS_ZONE_H
#pragma once

#include <libnvme.h>
#include <pthread.h>

#include <cstdint>
#include <iostream>
#include <vector>

#include "znsblock.hpp"

enum ZoneState {
  Empty = 0x0,
  Full = 0x1,
  ImplicitOpen = 0x2,
  ExplicitOpen = 0x3,
  Closed = 0x4,
  ReadOnly = 0xD,
  Offline = 0xF
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

  /** Writes a block to the zone and returns the updated write pointer. */
  uint64_t write_block(const uint16_t total_nlb,
                       const uint16_t max_nlb_per_round, const uint64_t address,
                       const void *buffer, const uint32_t size);

  /** Removes a block from the zone and returns the updated write pointer. */
  uint64_t remove_block(const ZNSBlock &block);

  /** Calculates the current capacity of the block. */
  uint32_t get_current_capacity() const;

  /** Gets a block from the zone based on the block id. */
  ZNSBlock &get_block(const uint64_t block_id) const;
  uint32_t read(const uint64_t lba, const void *buffer, uint32_t size);
  uint32_t write(void *buffer, uint32_t size);

  /** Get victim block */
  ZNSBlock &get_victim(void);

  /** Reset the write pointer to the start. */
  int reset_zone(void) const;

  /** Open the zone explicitely so more resources are allocated. */
  int open_zone(void) const;

  /** Explicitely close a zone and free the resources. */
  int close_zone(void) const;

  /** Close the zone so that writes cannot be performed until it is reset. */
  int finish_zone(void) const;

  friend std::ostream &operator<<(std::ostream &os, ZNSZone const &tc);

  // Fuck
  int get_index();
  int reset();
  bool is_full();
  uint32_t curr_capacity();
  uint64_t get_wp();

  uint64_t position;
  uint64_t base;
  uint64_t lba_size;
  uint64_t mdts_size;
  pthread_rwlock_t lock;

 private:
  int zns_fd;

  /** Number of blocks in the zone reserved for GC in percentages. */
  static const uint64_t overcapacity = 5;

  int ss_sequential_write(const void *buffer, const uint16_t max_nlb_per_round,
                          const uint16_t total_nlb);

  std::vector<ZNSBlock> blocks;
  inline int send_management_command(
      const enum nvme_zns_send_action action) const;
};

std::ostream &operator<<(std::iostream &os, ZNSZone const &tc);
int get_zns_zone_info(const int fd, const int nsid, uint64_t *zcap,
                      uint32_t *nr, struct nvme_zns_desc *desc[]);

extern "C" {
std::vector<ZNSZone> create_zones(const int zns_fd, const uint32_t nsid,
                                  const uint64_t lba_size,
                                  const uint64_t mdts_size);
}
#endif
