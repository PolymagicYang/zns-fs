#ifndef FTL_H
#define FTL_H

#include "ftl.hpp"

#include <pthread.h>
#include <sys/types.h>

#include <cstdint>
#include <vector>
#include "zone.hpp"

FTL::FTL(int fd, uint64_t mdts, uint32_t nsid, uint16_t lba_size, int gc_wmark, int log_zones) {
  this->fd = fd;
  this->mdts_size = mdts;
  this->nsid = nsid;
  this->lba_size = lba_size;
  this->gc_wmark = gc_wmark;
  this->zones_num = zones_num;
  this->log_zones = log_zones;
  this->log_map = Ftlmap{.lock = PTHREAD_RWLOCK_INITIALIZER, .map = raw_map()};
  this->data_map = Ftlmap{.lock = PTHREAD_RWLOCK_INITIALIZER, .map = raw_map()};
  this->zones = create_zones(fd, nsid, lba_size, mdts);
  this->zcap = zones.at(0).capacity;
  this->zones_num = zones.size();
};

ZNSZone *FTL::get_random_datazone() {
  size_t validRange = this->zones_num - log_zones;
  size_t randomIndex = this->log_zones + (std::rand() % validRange);

  return &this->zones[randomIndex];
};

ZNSZone *FTL::get_random_logzone() {
  size_t randomIndex = std::rand() % this->log_zones;
  return &this->zones[randomIndex];
};

// unsafe.
ZNSZone *FTL::get_zone(int i) { return &this->zones[i]; };

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
};

Addr FTL::get_pba(u_int64_t lba) {
  pthread_rwlock_rdlock(&this->data_map.lock);
  Addr ret = this->data_map.map.find(lba)->second;
  pthread_rwlock_unlock(&this->data_map.lock);
  return ret;
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

void FTL::insert_logmap(uint64_t lba, uint64_t pa, uint16_t zone_num) {
  pthread_rwlock_wrlock(&this->log_map.lock);
  this->log_map.map[lba] = Addr{.addr = pa, .zone_num = zone_num};
  pthread_rwlock_unlock(&this->log_map.lock);
}

void FTL::insert_datamap(uint64_t lba, uint64_t pa, uint16_t zone_num) {
  pthread_rwlock_wrlock(&this->data_map.lock);
  this->data_map.map[lba] = Addr{.addr = pa, .zone_num = zone_num};
  pthread_rwlock_unlock(&this->data_map.lock);
}

int FTL::read(uint64_t lba, void *buffer, uint32_t size) {
  uint64_t pages_num = size / this->lba_size;

  Addr pa;
  for (uint64_t i = 0; i < pages_num; i++) {
    pa = this->get_pa(lba + i);
    ZNSZone *zone = this->get_zone(pa.zone_num);
    int ret = zone->read(pa.addr, buffer, this->lba_size);
	if (ret != 0) {
		return ret;
	}
	
    buffer = (void *)((uint64_t)buffer + this->lba_size);
  }
  return 0;
};

int FTL::write(uint64_t lba, void *buffer, uint32_t size) {
  // write to the log zone.
  // TODO: change zones_num to log_zone to enable GC.
  for (uint16_t i = 1; i < this->zones_num; i++) {
    ZNSZone *zone = this->get_zone(i);
    if (zone->is_full()) {
      continue;
    } else {
      uint64_t wp_starts = zone->get_wp();
	  uint32_t write_size; 
      int ret = zone->write(buffer, size, write_size);
	  if (ret != 0) {
		  return ret; 
	  }
		  
      uint64_t wp_ends = zone->get_wp();
      for (; wp_starts < wp_ends; wp_starts++) {
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
};

#endif
