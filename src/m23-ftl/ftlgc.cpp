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

#include <chrono>

#include "../common/nvmewrappers.h"
#include "znsblock.hpp"

Calliope::Calliope(FTL *ftl, pthread_cond_t *cond, pthread_mutex_t *mutex) {
  this->ftl = ftl;
  this->can_reap = false;
  this->cond = cond;
  this->condmutex = mutex;
}

int Calliope::select_zone() {
  int max = 0;
  float max_util = 0;
  float util = 0;

  // Select the region with the most undead blocks compared to the
  // total capacity. This safes on the copies we need to do. 
  for (int i = 0; i < ftl->zones_num; i++) {
    ZNSZone &current = ftl->zones[i];
    if (!current.is_full()) continue;

    if (current.get_alive_capacity() == 0) {
      util = 0;
    } else {
      util = 1.0 - ((float)current.get_alive_capacity() / current.capacity);
    }
    if (util > max_util) {
      max = i;
      max_util = util;
    }
  }

  // If we cannot find something decent to do, we flag the thread to
  // just keep going. 
  this->can_reap = max_util != 0.0f;
  return max;
}

void Calliope::reap() {

  while (true) {
	  pthread_mutex_lock(this->condmutex);
	  pthread_cond_wait(this->cond, this->condmutex);
	  pthread_mutex_unlock(this->condmutex);

    pthread_rwlock_rdlock(&this->ftl->zone_lock);
    int free_count = this->ftl->get_free_regions();
	
    if (free_count > this->threshold) {
      pthread_rwlock_unlock(&this->ftl->zone_lock);
      std::cout << "Reporting back to death-sama: " << free_count << std::endl;
      continue;
    }
	pthread_rwlock_unlock(&this->ftl->zone_lock);

	// Get the zone with the highest win of free blocks, if none is
	// found we just wait until the next loop. This can happen if no
	// data is overwritten
    int zone_num = this->select_zone();
    ZNSZone *reapable = &this->ftl->zones[zone_num];
    if (!this->can_reap) {
      continue;
    }

    // Lock the zone since we are modifying it from this point
    // onwards. We are using the 0th region as a scratch buffer
  	// where we copy data back and forth from. 
    pthread_mutex_lock(&reapable->zone_mutex);
    this->can_reap = false;
    std::vector<physaddr_t> blocks = reapable->get_nonfree_blocks();
    ZNSZone *zone = &this->ftl->zones[0];
	std::vector<physaddr_t> lbas;

    // Copy data to the new zone block by block
    // TODO(valentijn): move by MDTS chunks instead
	std::vector<physaddr_t> pas;
	
    for (size_t i = 0; i < blocks.size(); i++) {
	  physaddr_t address = blocks.at(i);
      // TODO(valentijn): we have a nice copy command which is not working
      //   use it instead of this garbage	  
      char buffer[this->ftl->lba_size];
      uint32_t read_size;
	  physaddr_t lba = reapable->block_map.map[address].logical_address;

	  uint64_t wp_starts = zone->get_wp();
      reapable->read(address, &buffer, this->ftl->lba_size, &read_size);
      zone->write(address, &buffer, this->ftl->lba_size, &read_size);
	  pas.push_back(wp_starts);
	  lbas.push_back(lba);	  
    }
	reapable->deadbeat = true;
    reapable->reset();
	
    // TODO(valentijn): Do this one round trip instead of N
    uint64_t wp = zone->position;
	for (size_t i = 0; i < pas.size(); i++) {
	  physaddr_t address = pas.at(i);
      char buffer[this->ftl->lba_size];
      uint32_t read_size;
	  uint64_t wp_starts = reapable->get_wp();
      zone->read(address, &buffer, this->ftl->lba_size, &read_size);
      reapable->write(lbas.at(i), &buffer, this->ftl->lba_size, &read_size);
	  wp += this->ftl->lba_size;
	  this->ftl->insert_logmap(lbas.at(i), wp_starts, reapable->zone_id);
    }

	// Unlock the zone since we finished writing and reset our scratch
	// region.
	pthread_mutex_unlock(&reapable->zone_mutex);
    zone->reset();
	
	
  }
}

void Calliope::initialize() {
  std::cout << "Summons Mori from hell" << std::endl;
  this->thread = std::thread(&Calliope::reap, this);
}
