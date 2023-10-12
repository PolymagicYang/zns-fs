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
#include "ftlgc.hpp"

#include <pthread.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <ostream>
#include <unordered_map>
#include <vector>

#include "../common/nvmewrappers.h"
#include "datazone.hpp"
#include "znsblock.hpp"
#include "zone.hpp"


bool death_sensei = true;

Calliope::Calliope(FTL *ftl, pthread_cond_t *cond, pthread_mutex_t *mutex,
                   pthread_cond_t *clean_cond, pthread_mutex_t *clean_lock) {
  this->ftl = ftl;
  this->can_reap = false;
  this->need_gc = cond;
  this->need_gc_lock = mutex;
  this->clean_cond = clean_cond;
  this->clean_lock = clean_lock;
}

bool Calliope::select_log_zone(uint16_t *zone_num) {
  // Select the region with the most undead blocks compared to the
  // total capacity. This safes on the copies we need to do.
  for (uint16_t i = 0; i < ftl->zones_log.size(); i++) {
    ZNSLogZone *current = &ftl->zones_log[i];
    if (current->is_full()) {
      *zone_num = i;
      return true;
    }
  }

  // If we cannot find something decent to do, we flag the thread to
  // just keep going.
  // this->can_reap = max_util != 0.0f;
  return false;
}

bool compare_block(ZNSBlock *block1, ZNSBlock *block2) {
  return (block1->logical_address < block2->logical_address);
}

uint16_t Calliope::wait_for_mutex() {
  uint16_t log_zone_num;
  while (!this->select_log_zone(&log_zone_num)) {
    // if there is no full zone exists, let the consumer consumes.
    pthread_mutex_lock(this->clean_lock);
    pthread_cond_signal(this->clean_cond);
    pthread_mutex_unlock(this->clean_lock);

    pthread_mutex_lock(this->need_gc_lock);
    pthread_cond_wait(this->need_gc, this->need_gc_lock);
    pthread_mutex_unlock(this->need_gc_lock);
	if (death_sensei) exit(0);
    this->select_log_zone(&log_zone_num);
  }

  return log_zone_num;
}

void Calliope::get_blocks_group(
    ZNSLogZone *reapable,
    std::unordered_map<uint64_t, std::vector<ZNSBlock *>> &blocks_group) {
  std::vector<ZNSBlock *> temp_blocks;
  std::vector<ZNSBlock *> blocks = reapable->get_nonfree_blocks();
  // sort the blocks by the logical addresses.
  std::sort(blocks.begin(), blocks.end(), compare_block);
  // group them by base address.
  for (ZNSBlock *block : blocks) {
    uint64_t lba_inblock = block->logical_address / this->ftl->lba_size;
    uint64_t base_addr = (lba_inblock / this->ftl->zcap) * ftl->zcap;
    // std::cout << "logical addr: " << lba_inblock % ftl->zcap << "\t
    // base_addr: " << base_addr << "\n";
    if (blocks_group.count(base_addr) == 0) {
      std::vector<ZNSBlock *> temp_block;
      temp_block.push_back(block);
      blocks_group[base_addr] = temp_block;
    } else {
      blocks_group[base_addr].push_back(block);
    }
  }
}

void Calliope::merge_old_zone(ZNSLogZone *reapable, uint64_t base_addr,
                              std::vector<ZNSBlock *> &log_blocks) {
  // try to merge the old zone.
  // append until can not append, after can't append:
  //
  Addr addr;
  this->ftl->get_pba_by_base(base_addr, &addr);
  uint16_t zone_num = addr.zone_num;
  ZNSDataZone *data_zone = &this->ftl->zones_data[zone_num];
  ZNSDataZone *new_data_zone = this->ftl->get_free_data_zone(this->ftl->zcap);
  if (new_data_zone == nullptr) {
    printf("failed!\n");
  }
  // std::cout << "Reap!" << std::endl;
  // if (new_data_zone == nullptr) {
  // use the reserved zone.
  //  new_data_zone = &this->ftl->zones_reserved[0];
  //}

  uint32_t block_index;
  ZNSBlock *block;
  block = log_blocks.front();
  block_index = (block->logical_address / ftl->lba_size) % ftl->zcap;
  // std::vector<physaddr_t> zone_addrs = data_zone->get_nonfree_blocks();
  for (uint32_t index = 0; index < data_zone->block_map.size(); index++) {
    char buffer[this->ftl->lba_size];
    uint32_t read_size;
    if (block_index == index) {
      reapable->read(block->address, &buffer, ftl->lba_size, &read_size);
      new_data_zone->write_until(&buffer, ftl->lba_size, index);
      log_blocks.erase(log_blocks.begin());
      ftl->delete_logmap(block->logical_address);
      if (log_blocks.size() > 0) {
        block = log_blocks.front();
        block_index = (block->logical_address / ftl->lba_size) % ftl->zcap;
        // std::cout << "block index " << block_index << "index: " <<
        // new_data_zone->zone_id <<  std::endl;
      }
    } else if (data_zone->block_map[index]) {
      data_zone->read(data_zone->base + index, &buffer, ftl->lba_size,
                      &read_size);
      new_data_zone->write_until(&buffer, ftl->lba_size, index);
    }
  }

  this->ftl->insert_datamap(base_addr, data_zone->base,
                            new_data_zone->zone_id - ftl->log_zones);
  // ftl->data_map.map.count(base_addr));
  data_zone->reset();
}

void Calliope::insert_new_zone(ZNSLogZone *reapable, uint64_t base_addr,
                               std::vector<ZNSBlock *> &log_blocks) {
  // get a new data zone and insert.
  // new, no need to invalidate the block, just append to the new zone.
  ZNSDataZone *data_zone = this->ftl->get_free_data_zone(this->ftl->zcap);
  for (uint16_t i = 0; i < log_blocks.size(); i++) {
    ZNSBlock *block = log_blocks[i];
    uint64_t block_lba = block->logical_address / ftl->lba_size;
    uint16_t index = block_lba % this->ftl->zcap;
    char buffer[this->ftl->lba_size];
    uint32_t read_size;
    reapable->read(block->address, &buffer, this->ftl->lba_size, &read_size);
    data_zone->write_until(buffer, read_size, index);
    this->ftl->delete_logmap(block->logical_address);
  }
  this->ftl->insert_datamap(base_addr, data_zone->base,
                            data_zone->zone_id - ftl->log_zones);
}

void Calliope::reap() {
  while (true) {    
    uint16_t log_zone_num = this->wait_for_mutex();

    // Get the zone with the highest win of free blocks, if none is
    // found we just wait until the next loop. This can happen if no
    // data is overwritten
    ZNSLogZone *reapable = &this->ftl->zones_log[log_zone_num];
    std::unordered_map<uint64_t, std::vector<ZNSBlock *>> blocks_group =
        std::unordered_map<uint64_t, std::vector<ZNSBlock *>>();
    this->get_blocks_group(reapable, blocks_group);

    // find the data zone firstly, if find the correct one, try to append, if
    // failed, partial merge. if it doesn't find a data zone, write a new one.
    for (auto &group : blocks_group) {
      uint64_t base_addr = group.first;
      std::vector<ZNSBlock *> log_blocks = group.second;

      if (this->ftl->pba_exist(base_addr)) {
        this->merge_old_zone(reapable, base_addr, log_blocks);
      } else {
        this->insert_new_zone(reapable, base_addr, log_blocks);
      }
    }
		
    reapable->reset();
    pthread_rwlock_wrlock(&this->ftl->zones_lock);
    this->ftl->free_log_zones.push_back(reapable);
    pthread_rwlock_unlock(&this->ftl->zones_lock);
  }
}

void Calliope::initialize() {
  std::cout << "Summons Mori from hell" << std::endl;
  this->thread = std::thread(&Calliope::reap, this);
}
