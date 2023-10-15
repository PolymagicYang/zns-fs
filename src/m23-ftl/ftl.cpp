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

#include <algorithm>
#include <cassert>
#include <cstring>
#include <pthread.h>
#include <sys/types.h>

#include <cstdint>
#include <cstdio>
#include <vector>

#include "datazone.hpp"
#include "ftlgc.hpp"
#include "logzone.hpp"
#include "znsblock.hpp"
#include "zone.hpp"


void create_zones(const int zns_fd, const uint32_t nsid,
                  const uint64_t lba_size, const uint64_t mdts_size,
                  const uint16_t logs, std::vector<ZNSLogZone> *log_zones,
                  std::vector<ZNSDataZone> *rerv_zones,
                  std::vector<ZNSDataZone> *data_zones,
				  const bool force_reset) {
  // TODO(valentijn): don't hard code this please
  struct nvme_zone_report *zns_report =
      (struct nvme_zone_report *)calloc(1, 0x1000);
  uint64_t zcap;
  uint64_t nr;
  get_zns_zone_info(zns_fd, nsid, &nr, &zcap, zns_report);

  // Go through all the reprots and turn them into zones
  std::vector<ZNSLogZone> zones = std::vector<ZNSLogZone>();
  uint32_t i;
  if (logs >= nr) {
	  std::cout << "Invalid number of log zones " << logs << " " << " > " << nr  << std::endl;
	  exit(-1);
  }
	  
  for (i = 0; i < logs; i++) {
    struct nvme_zns_desc current = zns_report->entries[i];

    const enum ZoneZNSType ztype = static_cast<ZoneZNSType>(current.zt);
    const enum ZoneState zstate = static_cast<ZoneState>(current.zs);
    // TODO(valentijn): zone attributes (p.28 ZNS Command specification)
    const uint64_t capacity = le64_to_cpu(current.zcap);
    const uint64_t zone_slba = le64_to_cpu(current.zslba);
    uint64_t write_pointer = force_reset ? zone_slba : le64_to_cpu(current.wp);

    if (RESET_ZONE) {
      write_pointer = zone_slba;
    }

    uint64_t full = -1;
    if (write_pointer == full) {
      write_pointer = capacity * (i + 1);
    }

    ZNSLogZone zone =
        ZNSLogZone(zns_fd, nsid, i, capacity, capacity, zstate, ztype,
                   zone_slba, HostManaged, write_pointer, lba_size, mdts_size);
    zones.push_back(zone);

    // printf("zone %d\n", i);
    // printf("current position %lx\n", write_pointer);
    // printf("\n");
  }
  *log_zones = zones;

  std::vector<ZNSDataZone> temp_data_zones = std::vector<ZNSDataZone>();
  std::vector<ZNSDataZone> temp_rerved_zones = std::vector<ZNSDataZone>();
  for (; i < zns_report->nr_zones - 1; i++) {
    struct nvme_zns_desc current = zns_report->entries[i];

    const enum ZoneZNSType ztype = static_cast<ZoneZNSType>(current.zt);
    const enum ZoneState zstate = static_cast<ZoneState>(current.zs);
    // TODO(valentijn): zone attributes (p.28 ZNS Command specification)
    const uint64_t capacity = le64_to_cpu(current.zcap);
    const uint64_t zone_slba = le64_to_cpu(current.zslba);
    uint64_t write_pointer = force_reset ? zone_slba : le64_to_cpu(current.wp);

    if (RESET_ZONE) {
      write_pointer = zone_slba;
    }

    uint64_t full = -1;
    if (write_pointer == full) {
      write_pointer = capacity * (i + 1);
    }

    ZNSDataZone zone =
        ZNSDataZone(zns_fd, nsid, i, capacity, capacity, zstate, ztype,
                    zone_slba, HostManaged, write_pointer, lba_size, mdts_size);

    temp_data_zones.push_back(zone);
  }
  *data_zones = temp_data_zones;
  if (force_reset)
	  zones.at(0).reset_all_zones();


  for (auto &zone : zones) {
	  std::cout << zone.position << " " << zone.slba << std::endl;
  }
  for (auto &zone : *data_zones) {
	  std::cout << zone.position << " " << zone.slba << std::endl;
  }
  // Reset all the zones in one go so that we are in a valid initial state
  free(zns_report);
}

FTL::FTL(int fd, uint64_t mdts, uint32_t nsid, uint16_t lba_size, int gc_wmark,
         int log_zones, bool force_reset) {
  this->fd = fd;
  this->mdts_size = mdts;
  this->nsid = nsid;
  this->lba_size = lba_size;
  this->gc_wmark = gc_wmark;
  this->log_zones = log_zones;
  this->log_map = Ftlmap{.lock = PTHREAD_RWLOCK_INITIALIZER, .map = raw_map()};
  this->data_map = Ftlmap{.lock = PTHREAD_RWLOCK_INITIALIZER, .map = raw_map()};
  this->zone_lock = PTHREAD_RWLOCK_INITIALIZER;
  this->init_code = 2333;
  this->force_reset = force_reset;

  this->zones_reserved = std::vector<ZNSDataZone>();
  this->zones_data = std::vector<ZNSDataZone>();
  this->zones_log = std::vector<ZNSLogZone>();
  
  create_zones(fd, nsid, lba_size, mdts_size, log_zones, &zones_log,
               &zones_reserved, &zones_data, force_reset);
  this->zcap = zones_log.at(0).capacity;

  if (!force_reset) {
	  uint64_t last_zone_addr = zcap * (this->zones_log.size() + this->zones_data.size());
	  char meta_block[lba_size];
	  int ret = ss_nvme_read_wrapper(fd, nsid, last_zone_addr, 0, lba_size, meta_block);
	  assert(ret == 0);

	  uint64_t init_code_disk;
	  memcpy(&init_code_disk, meta_block, sizeof(uint64_t));
	  // printf("init code is %ld, current is %ld\n", init_code_disk, this->init_code);
	  if (init_code_disk == this->init_code) {
		  // restore the previous status of ftl.
		  std::cout << "restore from the previous status." << std::endl;
		  uint64_t buf_size;
		  memcpy(&buf_size, meta_block + sizeof(uint64_t), sizeof(uint64_t));
		  char meta_buffer[buf_size];
		  uint16_t nlb = buf_size / lba_size;
		  int ret = ss_nvme_read_wrapper(fd, nsid, last_zone_addr, nlb, buf_size, meta_buffer);
		  assert(ret == 0);

		  uint64_t lzone_buf_size;
		  uint64_t dzone_buf_size;
		  uint64_t lmap_buf_size;
		  uint64_t dmap_buf_size;
		  memcpy(&lzone_buf_size, meta_buffer + 2 * sizeof(uint64_t), sizeof(uint64_t));
		  memcpy(&dzone_buf_size, meta_buffer + 3 * sizeof(uint64_t), sizeof(uint64_t));
		  memcpy(&lmap_buf_size, meta_buffer + 4 * sizeof(uint64_t), sizeof(uint64_t));
		  memcpy(&dmap_buf_size, meta_buffer + 5 * sizeof(uint64_t), sizeof(uint64_t));
		  uint16_t buffer_index = sizeof(uint64_t) * 6;
		  // restore lzone.
		  // restore lzone pas.
		  std::vector<uint64_t> pas;
		  for (uint16_t i = 0; i < this->zones_log.size(); i++) {
			  uint64_t num;
			  memcpy(&num, meta_buffer + buffer_index, sizeof(uint64_t));
			  if (num == 0) {
				  buffer_index += sizeof(uint64_t);
				  continue;
			  }
			  buffer_index += sizeof(uint64_t);

			  uint64_t lbas[num];
			  ZNSBlock blocks[num];
			  memcpy(&lbas, meta_buffer + buffer_index, sizeof(uint64_t) * num);
			  buffer_index += sizeof(uint64_t) * num;
			  memcpy(&blocks, meta_buffer + buffer_index, sizeof(ZNSBlock) * num);
			  buffer_index += sizeof(ZNSBlock) * num;

			  // printf("\n");
			  for (uint64_t j = 0; j < num; j++) {
				  this->zones_log[i].block_map.map[lbas[j]] = blocks[j];
				  // printf("zone %d lab %lx -> %lx valid: %d \t", i, lbas[j], blocks[j].logical_address, blocks[j].valid);
			  }

			  // printf("\n");
		  }

		  // restore dzone.
		  for (uint16_t i = 0; i < this->zones_data.size(); i++) {
			  int dzone_buf[this->zcap];
			  memcpy(dzone_buf, meta_buffer + buffer_index, sizeof(int) * this->zcap);
			  this->zones_data[i].block_map.assign(dzone_buf, dzone_buf + this->zcap);
			  buffer_index += this->zcap * sizeof(int);

			  /*
				printf("zone %d\n", i);
				for (int j = 0; j < this->zcap; j++) {
				printf("valid :%d\t", this->zones_data[i].block_map[j]);
				}
				printf("\n");
			  */
		  }

		  // restore lmap.
		  // printf("lba map:\n");
		  uint64_t lmap_num = lmap_buf_size / (sizeof(uint64_t) + sizeof(Addr));
		  for (uint64_t i = 0; i < lmap_num; i++) {
			  uint64_t lba;
			  Addr pa;
			  memcpy(&lba, meta_buffer + buffer_index, sizeof(uint64_t));
			  buffer_index += sizeof(uint64_t);
			  memcpy(&pa, meta_buffer + buffer_index, sizeof(Addr));
			  buffer_index += sizeof(Addr);
			  this->log_map.map[lba] = pa;
			  // printf("lba %lx -> pa %lx, zone num: %d\t", lba, pa.addr, pa.zone_num);
		  }
		  // printf("\n");
		  
		  // restore dmap.
		  if (dmap_buf_size) {
			  // printf("data map:\n");
			  uint64_t dmap_num = dmap_buf_size / (sizeof(uint64_t) + sizeof(Addr));
			  for (uint64_t i = 0; i < dmap_num; i++) {
				  uint64_t lba;
				  Addr pa;
				  memcpy(&lba, meta_buffer + buffer_index, sizeof(uint64_t));
				  buffer_index += sizeof(uint64_t);
				  memcpy(&pa, meta_buffer + buffer_index, sizeof(Addr));
				  buffer_index += sizeof(Addr);
				  this->data_map.map[lba] = pa;
				  // printf("lba %lx -> pa %lx, zone num: %d\t", lba, pa.addr, pa.zone_num);
			  }
			  // printf("\n");
		  }
		  ss_device_zone_reset(fd, nsid, last_zone_addr);
	  }
  }
  // zones_log.at(0).reset_all_zones();
  // Start our reaper rapper and store her as a void pointer in our FTL
  this->need_gc = PTHREAD_COND_INITIALIZER;
  this->need_gc_lock = PTHREAD_MUTEX_INITIALIZER;
  this->clean_finish = PTHREAD_COND_INITIALIZER;
  this->clean_finish_lock = PTHREAD_MUTEX_INITIALIZER;

  Calliope *mori = new Calliope(this, &this->need_gc, &this->need_gc_lock,
                                &this->clean_finish, &this->clean_finish_lock);
  mori->initialize();
  this->mori = mori;

  // Setup the free zone logs
  this->zones_lock = PTHREAD_RWLOCK_INITIALIZER;

  this->free_log_zones = std::vector<ZNSLogZone *>();
  this->free_data_zones = std::vector<ZNSDataZone *>();

  for (size_t i = 0; i < this->zones_log.size(); i++) {
    if (!this->zones_log[i].is_full()) {
      this->free_log_zones.push_back(&this->zones_log[i]);
    }
  }

  for (size_t i = 0; i < this->zones_data.size(); i++) {
    if (!this->zones_data[i].is_full()) {
      this->free_data_zones.push_back(&this->zones_data[i]);
    }
  }
}

ZNSLogZone *FTL::get_free_log_zone() {
  return this->free_log_zones.empty() ? nullptr : this->free_log_zones.front();
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

int16_t FTL::get_free_log_regions() { return this->free_log_zones.size(); }

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

  // why use volatile here?
  // We have a lock to sync and data dependency, compiler won't reorder this.

  // get none full zone.
  while (size != 0) {
    int16_t free_regions = get_free_log_regions();
    if (free_regions <= this->gc_wmark) {
      // wake up the gc thread.
      pthread_mutex_lock(&this->need_gc_lock);
      pthread_cond_signal(&this->need_gc);
      pthread_mutex_unlock(&this->need_gc_lock);
    }

    // wait until gc clean up.
    ZNSLogZone *zone = get_free_log_zone();
    while (zone == nullptr) {
      // wait gc cleans up.
      pthread_mutex_lock(&this->clean_finish_lock);
      pthread_cond_wait(&this->clean_finish, &this->clean_finish_lock);
      pthread_mutex_unlock(&this->clean_finish_lock);
      zone = get_free_log_zone();
    }

    uint64_t wp_starts = zone->get_wp();
    uint32_t write_size;
    // printf("zone %d wp is %ld, size is %ld, current cap is %d\n", zone->zone_id, zone->position, size, zone->get_current_capacity());
    int ret = zone->write(buffer, size, &write_size, lba);

    // If we haven't written the entire buffer then we know that the
    // log is full and that we can move on to the next zone
    if (zone->get_current_capacity() <= 0) {
      pthread_rwlock_wrlock(&this->zones_lock);
      this->free_log_zones.erase(this->free_log_zones.begin());
      pthread_rwlock_unlock(&this->zones_lock);
    }
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

void FTL::backup() {
  // Store everything in the last zone.
  // Calculate the last zone address. 
  uint32_t zones_num = this->zones_log.size() + this->zones_data.size();
  uint64_t last_zone_addr = this->zcap * zones_num;

  uint64_t init_code = this->init_code;

  // store log zones data.
  // char logzone_buf[zones_log.size() ]
  std::vector<std::vector<uint64_t>> lbas_group;
  std::vector<std::vector<ZNSBlock>> blocks_group;
  for (uint16_t i = 0; i < this->zones_log.size(); i++) {
    std::vector<uint64_t> lbas;
    std::vector<ZNSBlock> blocks;
    ZNSLogZone* zone = &this->zones_log[i];
    pthread_rwlock_rdlock(&zone->block_map.lock);
    BlockMap bmap = zone->block_map.map;
    pthread_rwlock_unlock(&zone->block_map.lock);

    // printf("\n");
    for (BlockMap::iterator iter = bmap.begin(); iter != bmap.end(); iter++) {
      uint64_t lba = iter->first;
      ZNSBlock block = iter->second;
      // printf("zone %d : %lx -> %lx \t", i, lba, block.logical_address);
      lbas.push_back(lba);
      blocks.push_back(block);
    }
    lbas_group.push_back(lbas);
    blocks_group.push_back(blocks);
    // printf("\n");

  }
  uint64_t lzone_buf_size = 0;

  for (uint16_t i = 0; i < this->zones_log.size(); i++) {
    uint64_t lba_cap = lbas_group[i].size() * sizeof(uint64_t);
    uint64_t block_cap = blocks_group[i].size() * sizeof(ZNSBlock);
    lzone_buf_size += (lba_cap + block_cap + sizeof(uint64_t));
    // [size of lba_cap + block_cap] [lbas] [blocks].
  }

  // store data zones data.
  uint64_t dzone_buf_size = this->zones_data.size() * this->zcap * sizeof(int);
  char datazone_buf[dzone_buf_size];
  uint64_t map_buf_addr = (uint64_t) datazone_buf;
  for (uint16_t i = 0; i < this->zones_data.size(); i++) {
    ZNSDataZone *zone = &this->zones_data[i];
    std::vector<int> map = zone->block_map;
    
    memcpy((void *) map_buf_addr, map.data(), map.size());

    /*
    printf("zone %d\n", i);
    for (int j = 0; j < this->zcap; j++) {
      printf("valid :%d\t", map[j]);
    }
    printf("\n");
    */

    map_buf_addr += map.size();
  }

  // store log zone map.
  pthread_rwlock_rdlock(&this->log_map.lock);
  raw_map logmap = this->log_map.map;
  pthread_rwlock_unlock(&this->log_map.lock);

  uint64_t lmap_buf_size = logmap.size() * (sizeof(uint64_t) + sizeof(Addr));
  char logmap_buf[lmap_buf_size];
  map_buf_addr = (uint64_t) logmap_buf;
  // printf("\n");
  for (raw_map::iterator iter = logmap.begin(); iter != logmap.end(); iter++) {
    uint64_t lba = iter->first;
    Addr pa = iter->second;
    memcpy((void *) map_buf_addr, &lba, sizeof(uint64_t));
    map_buf_addr += sizeof(uint64_t);
    memcpy((void *) map_buf_addr, &pa, sizeof(Addr));
    map_buf_addr += sizeof(Addr);
    // printf("lba %lx -> pa %lx, zone num: %d\t", lba, pa.addr, pa.zone_num);
  }

  // printf("\n");

  // store data zone map.
  pthread_rwlock_rdlock(&this->data_map.lock);
  raw_map datamap = this->data_map.map;
  pthread_rwlock_unlock(&this->data_map.lock);

  uint64_t dmap_buf_size = datamap.size() * (sizeof(uint64_t) + sizeof(Addr));
  uint64_t total_size = lzone_buf_size + dzone_buf_size + lmap_buf_size + dmap_buf_size;
  // [init_code] [buf_size] [lzone_buf_size] [dzone_buf_size] [lmap_buf_size] [dmap_buf_size] [metadata].
  // we know the number of log zones and data zones, so it should be easy to restore them.

  // make sure it always be the multiple of lba_size.
  uint64_t buf_size = ((sizeof(uint64_t) * 6 + total_size) / this->lba_size) * this->lba_size + this->lba_size;
  uint16_t nlb = buf_size / this->lba_size;
  char final_buf[buf_size];
  uint64_t final_buf_addr = (uint64_t) final_buf;

  memcpy((void *) final_buf_addr, &init_code, sizeof(uint64_t));
  final_buf_addr += sizeof(uint64_t);
  memcpy((void *) final_buf_addr, &buf_size, sizeof(uint64_t));
  final_buf_addr += sizeof(uint64_t);
  memcpy((void *) final_buf_addr, &lzone_buf_size, sizeof(uint64_t));
  final_buf_addr += sizeof(uint64_t);
  memcpy((void *) final_buf_addr, &dzone_buf_size, sizeof(uint64_t));
  final_buf_addr += sizeof(uint64_t);
  memcpy((void *) final_buf_addr, &lmap_buf_size, sizeof(uint64_t));
  final_buf_addr += sizeof(uint64_t);
  memcpy((void *) final_buf_addr, &dmap_buf_size, sizeof(uint64_t));
  final_buf_addr += sizeof(uint64_t);

  for (uint16_t i = 0; i < this->zones_log.size(); i++) {
    if (lbas_group[i].size() == 0) {
      uint64_t num = 0;
      memcpy((void *) final_buf_addr, &num, sizeof(uint64_t));
      final_buf_addr += sizeof(uint64_t);
      continue;
    }
    void* lbas_buf = lbas_group[i].data();
    uint64_t lbas_num = lbas_group[i].size();
    void* blocks_buf = blocks_group[i].data();
    uint64_t blocks_num = blocks_group[i].size();
    uint64_t num = lbas_num;

    // printf("zone %d\n", i);
    // printf("current position %lx\n", zones_log[i].position);
    // printf("\n");

    memcpy((void *) final_buf_addr, &num, sizeof(uint64_t));
    final_buf_addr += sizeof(uint64_t);
    memcpy((void *) final_buf_addr, lbas_buf, lbas_num * sizeof(uint64_t));
    final_buf_addr += lbas_num * sizeof(uint64_t);
    memcpy((void *) final_buf_addr, blocks_buf, blocks_num * sizeof(ZNSBlock));
    final_buf_addr += blocks_num * sizeof(ZNSBlock);
  }
  
  memcpy((void *) final_buf_addr, datazone_buf, dzone_buf_size);
  final_buf_addr += dzone_buf_size;
  memcpy((void *) final_buf_addr, logmap_buf, lmap_buf_size);
  final_buf_addr += lmap_buf_size;

  if (dmap_buf_size) {
    // no GC if datamap size is zero.
    char datamap_buf[dmap_buf_size];
    map_buf_addr = (uint64_t) datamap_buf;
    // printf("data zone\n");
    for (raw_map::iterator iter = datamap.begin(); iter != datamap.end(); iter++) {
      uint64_t lba = iter->first;
      Addr pa = iter->second;
      memcpy((void *) map_buf_addr, &lba, sizeof(uint64_t));
      map_buf_addr += sizeof(uint64_t);
      memcpy((void *) map_buf_addr, &pa, sizeof(Addr));
      map_buf_addr += sizeof(Addr);
      // printf("lba %lx -> pa %lx, zone num: %d\t", lba, pa.addr, pa.zone_num);
    }
    memcpy((void *) final_buf_addr, datamap_buf, dmap_buf_size);
  }
  // printf("data zone end\n");
  final_buf_addr += dmap_buf_size;
  printf("meta data size is %d, slba is %lx, zones num is %d, init code is %d\n", buf_size, last_zone_addr, zones_num, init_code);

  ss_nvme_write_wrapper(this->fd, this->nsid, last_zone_addr, nlb, buf_size, final_buf);
}

#endif
