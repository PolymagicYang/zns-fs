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
#include <cstdio>
#include <vector>

#include "datazone.hpp"
#include "ftlgc.hpp"
#include "logzone.hpp"
#include "zone.hpp"

void create_zones(const int zns_fd, const uint32_t nsid,
                  const uint64_t lba_size, const uint64_t mdts_size,
                  const uint16_t logs, std::vector<ZNSLogZone> *log_zones,
                  std::vector<ZNSDataZone> *rerv_zones,
                  std::vector<ZNSDataZone> *data_zones) {
  // TODO(valentijn): don't hard code this please
  struct nvme_zone_report *zns_report =
      (struct nvme_zone_report *)calloc(1, 0x1000);
  uint64_t zcap;
  uint64_t nr;
  get_zns_zone_info(zns_fd, nsid, &nr, &zcap, zns_report);

  // Go through all the reprots and turn them into zones
  std::vector<ZNSLogZone> zones = std::vector<ZNSLogZone>();
  uint32_t i;
  for (i = 0; i < logs; i++) {
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

    ZNSLogZone zone =
        ZNSLogZone(zns_fd, nsid, i, capacity, capacity, zstate, ztype,
                   zone_slba, HostManaged, write_pointer, lba_size, mdts_size);
    zones.push_back(zone);
  }
  *log_zones = zones;

  std::vector<ZNSDataZone> temp_data_zones = std::vector<ZNSDataZone>();
  std::vector<ZNSDataZone> temp_rerved_zones = std::vector<ZNSDataZone>();
  for (; i < zns_report->nr_zones; i++) {
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

    ZNSDataZone zone =
        ZNSDataZone(zns_fd, nsid, i, capacity, capacity, zstate, ztype,
                    zone_slba, HostManaged, write_pointer, lba_size, mdts_size);

    // if (i == zns_report->nr_zones - 1) {
    //  temp_rerved_zones.push_back(zone);
    //} else {
    temp_data_zones.push_back(zone);
    //}
  }
  *data_zones = temp_data_zones;
  // *rerv_zones = temp_rerved_zones;

  // Reset all the zones in one go so that we are in a valid initial state
  zones.at(0).reset_all_zones();
  free(zns_report);
}

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

  this->zones_reserved = std::vector<ZNSDataZone>();
  this->zones_data = std::vector<ZNSDataZone>();
  this->zones_log = std::vector<ZNSLogZone>();

  create_zones(fd, nsid, lba_size, mdts_size, log_zones, &zones_log,
               &zones_reserved, &zones_data);
  this->zcap = zones_log.at(0).capacity;

  // Start our reaper rapper and store her as a void pointer in our FTL
  this->need_gc = PTHREAD_COND_INITIALIZER;
  this->need_gc_lock = PTHREAD_MUTEX_INITIALIZER;
  this->clean_finish = PTHREAD_COND_INITIALIZER;
  this->clean_finish_lock = PTHREAD_MUTEX_INITIALIZER;

  Calliope *mori = new Calliope(this, &this->need_gc, &this->need_gc_lock,
                                &this->clean_finish, &this->clean_finish_lock);
  mori->initialize();
  this->mori = &mori;
}

ZNSLogZone *FTL::get_free_log_zone(const uint32_t needed) {
  // The unit of needed size if lba.
  for (uint16_t i = 0; i < this->zones_log.size(); i++) {
    if ((&this->zones_log[i])->get_current_capacity() >= needed) {
      return &this->zones_log[i];
    }
  }
  return nullptr;
}

ZNSDataZone *FTL::get_free_data_zone(const uint32_t needed) {
  for (uint16_t i = 0; i < this->zones_data.size(); i++) {
    if ((&this->zones_data[i])->get_current_capacity() >= needed)
      return &this->zones_data[i];
  }
  return nullptr;
}

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

bool FTL::get_pba_by_base(uint64_t base_addr, Addr *addr) {
  pthread_rwlock_rdlock(&this->data_map.lock);
  auto ret = this->data_map.map.find(base_addr);
  if (ret == this->data_map.map.end()) {
    pthread_rwlock_unlock(&this->data_map.lock);
    return false;
  } else {
    pthread_rwlock_unlock(&this->data_map.lock);
    *addr = ret->second;
    return true;
  }
}

bool FTL::get_pba(u_int64_t lba, Addr *addr) {
  pthread_rwlock_rdlock(&this->data_map.lock);
  uint64_t base_addr = ((lba / this->lba_size) / this->zcap) * this->zcap;
  auto ret = this->data_map.map.find(base_addr);
  if (ret == this->data_map.map.end()) {
    pthread_rwlock_unlock(&this->data_map.lock);
    return false;
  } else {
    bool exist = this->zones_data[ret->second.zone_num].exists(lba);
    pthread_rwlock_unlock(&this->data_map.lock);
    *addr = ret->second;
    return exist;
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

void FTL::delete_logmap(uint64_t lba) {
  pthread_rwlock_wrlock(&this->log_map.lock);
  this->log_map.map.erase(lba);
  pthread_rwlock_unlock(&this->log_map.lock);
}

void FTL::insert_datamap(uint64_t base_addr, uint64_t pa, uint16_t zone_num) {
  pthread_rwlock_wrlock(&this->data_map.lock);
  uint64_t pa_base = (pa / this->zcap) * this->zcap;
  this->data_map.map[base_addr] =
      Addr{.addr = pa_base, .zone_num = zone_num, .alive = true};
  pthread_rwlock_unlock(&this->data_map.lock);
}

int FTL::read(uint64_t lba, void *buffer, uint32_t size) {
  uint64_t pages_num = size / this->lba_size;

  for (uint64_t i = 0; i < pages_num; i++) {
    Addr pa;
    bool contains = this->get_ppa(lba + i * this->lba_size, &pa);
    if (contains) {
      ZNSLogZone *zone = &this->zones_log[pa.zone_num];
      uint32_t read_size;
      int ret = zone->read(pa.addr, buffer, this->lba_size, &read_size);
      if (ret != 0) {
        return ret;
      }

      buffer = (void *)((uint64_t)buffer + this->lba_size);
    } else {
      // in the block zones.
      bool contains = this->get_pba(lba, &pa);
      if (contains) {
        ZNSDataZone *zone = &this->zones_data[pa.zone_num];
        uint32_t read_size;
        uint64_t index = (lba / this->lba_size) % this->zcap;
        int ret =
            zone->read(zone->base + index, buffer, this->lba_size, &read_size);
        if (ret != 0) {
          return ret;
        }

        buffer = (void *)((uint64_t)buffer + this->lba_size);
      } else {
      }
    }
  }
  return 0;
}

int16_t FTL::get_free_log_regions() {
  uint16_t free_count = 0;
  for (uint16_t i = 0; i < this->zones_log.size(); i++) {
    const ZNSLogZone *zone = &this->zones_log[i];
    if (!(zone->get_current_capacity() == 0)) free_count++;
  }
  return free_count;
}

bool FTL::pba_exist(uint64_t base_addr) {
  pthread_rwlock_rdlock(&this->data_map.lock);
  bool ret = this->data_map.map.count(base_addr) > 0;
  pthread_rwlock_unlock(&this->data_map.lock);
  return ret;
}

int FTL::write(uint64_t lba, void *buffer, uint32_t size) {
  // If we don't have enough free regions we wait for our GC
  // to clean our mess. Until that time we are locking the
  // zone since we are reading to it.

  // why use volitile here?
  // We have a lock to sync and data dependency, compiler won't reorder this.

  // get none full zone.
  while (size != 0) {
    int16_t free_regions = get_free_log_regions();
    if (free_regions <= 0) {
      // wake up the gc thread.
      pthread_mutex_lock(&this->need_gc_lock);
      pthread_cond_signal(&this->need_gc);
      pthread_mutex_unlock(&this->need_gc_lock);
    }

    // wait until gc clean up.
    ZNSLogZone *zone = get_free_log_zone(1);
    while (zone == nullptr) {
      // wait gc cleans up.
      pthread_mutex_lock(&this->clean_finish_lock);
      pthread_cond_wait(&this->clean_finish, &this->clean_finish_lock);
      pthread_mutex_unlock(&this->clean_finish_lock);
      zone = get_free_log_zone(1);
    }

    uint64_t wp_starts = zone->get_wp();
    uint32_t write_size;
    int ret = zone->write(buffer, size, &write_size, lba);

    if (ret != 0) {
      return ret;
    }

    for (uint64_t wp_ends = zone->get_wp(); wp_starts < wp_ends; wp_starts++) {
      // write to the log zone.
      // Mark the LBA as invalid and inform the region to invalidate
      // each block.
      Addr pa;
      if (this->get_ppa(lba, &pa)) {
        (&this->zones_log[pa.zone_num])->invalidate_block(pa.addr);
      }
      this->insert_logmap(lba, wp_starts, zone->zone_id);
      lba += this->lba_size;
    }
    size -= write_size;
    buffer = (void *)((uint64_t)buffer + write_size);
  }
  return 0;
}

#endif
