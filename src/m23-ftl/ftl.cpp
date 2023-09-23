#ifndef FTL_H
#define FTL_H

#include "FTL.h"
#include <cstdint>
#include <sys/types.h>
#include <vector>

FTL::FTL(int fd, uint64_t mdts, uint32_t nsid, uint32_t zcap, uint16_t lba_size, int gc_wmark, int zones_num, int log_zones) {
  this->fd = fd;
  this->mdts_size = mdts;
  this->nsid = nsid;
  this->zcap = zcap;
  this->lba_size = lba_size;
  this->gc_wmark = gc_wmark;
  this->zones_num = zones_num;
  this->log_zones = log_zones;

  std::vector<Zone> zones;
  for (int i = 0; i < zones_num; i++) {
    u_int64_t base_addr = zcap * i;
    Zone zone = Zone(fd, nsid, lba_size, base_addr, zcap, mdts, i);
    zone.reset();
    zones.push_back(zone);
  }
  this->zones = zones;
};

Zone* FTL::get_random_datazone() {
    size_t validRange = this->zones_num - log_zones;
    size_t randomIndex = this->log_zones + (std::rand() % validRange);

    return &this->zones[randomIndex];
};

Zone* FTL::get_random_logzone() {
    size_t randomIndex = std::rand() % this->log_zones;
    return &this->zones[randomIndex];
};

Zone* FTL::get_zone(int i) {
    return &this->zones[i];
};

bool FTL::get_ppa(uint64_t lba, Addr *addr) {
    // TODO: add lock.
    auto ret = this->log_map.map.find(lba);
    if (ret == this->log_map.map.end()) {
      return false;
    } else {
      *addr = ret->second;
      return true;
    }
};

Addr FTL::get_pba(u_int64_t lba) {
    // TODO: add lock.
    return this->data_map.map.find(lba)->second;
};

Addr FTL::get_pa(uint64_t addr) {
    Addr ppa;
    bool has_ppa = this->get_ppa(addr, &ppa);
    if (has_ppa) {
      return ppa;
    } else {
      return this->get_pba(addr);
    }
};

int FTL::read(uint64_t lba, void *buffer, uint32_t size) {
  uint64_t pages_num = size / this->lba_size;

  Addr pa;
  for (uint64_t i = 0; i < pages_num; i++) {
    pa = this->get_pa(lba + i);
    Zone *zone = this->get_zone(pa.zone_num);
    zone->read(pa.addr, buffer, this->lba_size);
    buffer = (void *) ((uint64_t) buffer + this->lba_size);
  } 
  return 0;
};

int FTL::write(uint64_t lba, void *buffer, uint32_t size) {
    // write to the log zone. 
    for (uint16_t i = 1; i < this->zones_num; i++) {
        Zone *zone = this->get_zone(i);
        if (zone->is_full()) { 
            continue; 
        } else {
            uint64_t wp_starts = zone->get_wp();
            uint32_t write_size = zone->write(buffer, size);
            uint64_t wp_ends = zone->get_wp();
            for (; wp_starts < wp_ends; wp_starts++) {
                // TODO: lock the lock.
                this->log_map.lock;
                this->log_map.map[lba] = Addr {
                    .addr = wp_starts,
                    .zone_num = i,
                };
                lba++;
                // TODO: unlock the lock.
            }
            size -= write_size;
            buffer = (void *) ((uint64_t) buffer + write_size);
            if (size == 0) {
                return 0;
            }
        }
    }
};

#endif
