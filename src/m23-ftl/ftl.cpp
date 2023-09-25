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

#ifndef FTL_H
#define FTL_H

#include "ftl.hpp"

#include <pthread.h>
#include <sys/types.h>

#include <cstdint>
#include <vector>

#include "ftlgc.hpp"
#include "zone.hpp"

FTL::FTL(int fd, uint64_t mdts, uint32_t nsid, uint16_t lba_size, int gc_wmark,
         int log_zones) {
  this->fd = fd;
  this->mdts_size = mdts;
  this->nsid = nsid;
  this->lba_size = lba_size;
  this->gc_wmark = gc_wmark;
  this->log_zones = log_zones;
  this->log_map = Ftlmap{.lock = PTHREAD_RWLOCK_INITIALIZER, .map = raw_map()};
  this->data_map = Ftlmap{.lock = PTHREAD_RWLOCK_INITIALIZER, .map = raw_map()};
  this->zone_lock = PTHREAD_RWLOCK_INITIALIZER;
  this->zones = create_zones(fd, nsid, lba_size, mdts);
  this->zcap = zones.at(0).capacity;
  this->zones_num = zones.size();
  this->free_zones = zones.size();

  Calliope *mori = new Calliope(this, 2);
  mori->initialize();
  this->mori = &mori;
}

ZNSZone *FTL::get_random_datazone() {
  size_t validRange = this->zones_num - log_zones;
  size_t randomIndex = this->log_zones + (std::rand() % validRange);

  return &this->zones[randomIndex];
}

ZNSZone *FTL::get_random_logzone() {
  size_t randomIndex = std::rand() % this->log_zones;
  return &this->zones[randomIndex];
}

ZNSZone *FTL::get_free_zone(const uint32_t needed) {
  for (ZNSZone &zone : this->zones) {
    if (zone.get_current_capacity() >= needed) return &zone;
  }
}

// unsafe.
ZNSZone *FTL::get_zone(int i) { return &this->zones[i]; }

bool FTL::get_ppa(uint64_t lba, Addr *addr) {
  pthread_rwlock_rdlock(&this->log_map.lock);
  auto ret = this->log_map.map.find(lba);
  pthread_rwlock_unlock(&this->log_map.lock);
  if (ret == this->log_map.map.end()) {
    return false;
  } else {
    *addr = ret->second;
    return true;
  }
}

Addr FTL::get_pba(u_int64_t lba) {
  pthread_rwlock_rdlock(&this->data_map.lock);
  Addr ret = this->data_map.map.find(lba)->second;
  pthread_rwlock_unlock(&this->data_map.lock);
  return ret;
}

Addr FTL::get_pa(uint64_t addr) {
  Addr ppa;
  bool has_ppa = this->get_ppa(addr, &ppa);
  if (has_ppa) {
    return ppa;
  } else {
    return this->get_pba(addr);
  }
}

inline bool FTL::has_pa(uint64_t addr) {
  bool in_log = this->log_map.map.count(addr) > 0;
  bool in_data = this->data_map.map.count(addr) > 0;
  return in_log || in_data;
}

void FTL::insert_logmap(uint64_t lba, uint64_t pa, uint16_t zone_num) {
  pthread_rwlock_wrlock(&this->log_map.lock);
  this->log_map.map[lba] =
      Addr{.addr = pa, .zone_num = zone_num, .alive = true};
  pthread_rwlock_unlock(&this->log_map.lock);
}

void FTL::insert_datamap(uint64_t lba, uint64_t pa, uint16_t zone_num) {
  pthread_rwlock_wrlock(&this->data_map.lock);
  this->data_map.map[lba] =
      Addr{.addr = pa, .zone_num = zone_num, .alive = true};
  pthread_rwlock_unlock(&this->data_map.lock);
}

int FTL::read(uint64_t lba, void *buffer, uint32_t size) {
  uint64_t pages_num = size / this->lba_size;

  for (uint64_t i = 0; i < pages_num; i++) {
    Addr pa = this->get_pa(lba + i);
    ZNSZone *zone = this->get_zone(pa.zone_num);
    uint32_t read_size;
    int ret = zone->read(pa.addr, buffer, this->lba_size, &read_size);
    if (ret != 0) {
      return ret;
    }

    buffer = (void *)((uint64_t)buffer + this->lba_size);
  }
  return 0;
}

int16_t FTL::get_free_regions() const {
  uint16_t free_count = 0;

  for (int i = 0; i < this->zones_num; i++) {
    const ZNSZone *zone = &this->zones[i];
    if (!(zone->get_current_capacity() == 0)) free_count++;
  }
  return free_count;
}

int FTL::write(uint64_t lba, void *buffer, uint32_t size) {
  // Wait for our GC thread to cleanup

  int16_t free_regions = this->get_free_regions();
  while (free_regions < 3) {
    std::cerr << "Waiting for GC thread" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // write to the log zone.
  // TODO(valentijn): change zones_num to log_zone to enable GC.
  
  for (uint16_t i = 1; i < this->zones_num; i++) {
    ZNSZone *zone = this->get_zone(i);
    if (zone->is_full()) {
      continue;
    } else {
	  pthread_rwlock_wrlock(&zone->lock);		
      std::cout << "Writing to " << i << std::endl;
      if (this->has_pa(lba)) {
        // std::cout << "Address is " << std::hex << lba << " already in FTL"
        // << std::endl;
        Addr pa = this->get_pa(lba);
        pa.alive = false;
        this->zones[pa.zone_num].invalidate_block(pa.addr);
      }

      uint64_t wp_starts = zone->get_wp();
      uint32_t write_size;
      int ret = zone->write(buffer, size, &write_size);
	  pthread_rwlock_unlock(&zone->lock);
	  	  
      if (ret != 0) {
        return ret;
      }

      for (uint64_t wp_ends = zone->get_wp(); wp_starts < wp_ends;
           wp_starts++) {
        this->insert_logmap(lba, wp_starts, i);
        lba++;
      }
      size -= write_size;
      buffer = (void *)((uint64_t)buffer + write_size);
      if (size == 0) {
        return 0;
      }
    }
  }
  
  return 0;
}

#endif
