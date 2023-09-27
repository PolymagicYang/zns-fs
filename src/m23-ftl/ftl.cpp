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

  // Start our reaper rapper and store her as a void pointer in our FTL
  this->cond = PTHREAD_COND_INITIALIZER;
  this->cond_lock = PTHREAD_MUTEX_INITIALIZER;
  Calliope *mori = new Calliope(this, &this->cond, &this->cond_lock);
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
  if (ret == this->log_map.map.end()) {
    pthread_rwlock_unlock(&this->log_map.lock);
    return false;
  } else {
    pthread_rwlock_unlock(&this->log_map.lock);
    *addr = ret->second;
    return true;
  }
}

bool FTL::get_pba(u_int64_t lba, Addr *addr) {
  pthread_rwlock_rdlock(&this->data_map.lock);
  auto ret = this->data_map.map.find(lba);
  if (ret == this->data_map.map.end()) {
    pthread_rwlock_unlock(&this->data_map.lock);
    return false;
  } else {
    pthread_rwlock_unlock(&this->data_map.lock);
    *addr = ret->second;
    return true;
  }
}

Addr FTL::get_pa(uint64_t addr) {
  Addr ppa;
  bool has_ppa = this->get_ppa(addr, &ppa);
  if (has_ppa) {
    return ppa;
  }
  Addr pba;
  bool has_pba = this->get_pba(addr, &pba);
  if (has_pba) {
    return pba;
  } else {
    // return random addr.
    return Addr { .addr = 0, .zone_num = 0, .alive = true };
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

int16_t FTL::get_free_regions() {
  uint16_t free_count = 0;
  for (int i = 0; i < this->zones_num; i++) {
    const ZNSZone *zone = &this->zones[i];
    if (!(zone->get_current_capacity() == 0)) free_count++;
  }
  // std::cout << std::endl;
  return free_count;
}

int FTL::write(uint64_t lba, void *buffer, uint32_t size) {
  // If we don't have enough free regions we wait for our GC
  // to clean our mess. Until that time we are locking the
  // zone since we are reading to it. 

  pthread_rwlock_rdlock(&this->zone_lock);
  // why use volitile here? 
  // We have a lock to sync and data dependency, compiler won't reorder this.
  int16_t free_regions = get_free_regions();
  while (free_regions <= this->gc_wmark) {
    // wake up the gc thread.
    pthread_mutex_lock(&this->cond_lock);
    pthread_cond_signal(&this->cond);
    pthread_mutex_unlock(&this->cond_lock);
    free_regions = get_free_regions();
  }
  pthread_rwlock_unlock(&this->zone_lock);

  // write to the log zone.
  for (uint16_t i = 1; i < this->zones_num; i++) {
    ZNSZone *zone = this->get_zone(i);

    if (zone->is_full()) {
      continue;
    } else {
	  // We found a free region so we lock the zone mutex since
	  // we are going to write to it
	  pthread_mutex_lock(&zone->zone_mutex);

	  // Mark the LBA as invalid and inform the region to invalidate
	  // each block.
      if (this->has_pa(lba)) {
        Addr pa = this->get_pa(lba);
        pa.alive = false;
        this->zones[pa.zone_num].invalidate_block(pa.addr);
      }

      uint64_t wp_starts = zone->get_wp();
      uint32_t write_size;
      int ret = zone->write(buffer, size, &write_size);
	  
	  // We are done writing to the device so we can unlock it from here.	  
	  pthread_mutex_unlock(&zone->zone_mutex);

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
