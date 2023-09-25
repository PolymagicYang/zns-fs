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

Calliope::Calliope(FTL *ftl, const uint16_t threshold) {
  this->ftl = ftl;
  this->threshold = 3;
  this->can_reap = false;
}

bool Calliope::needs_reaping() {
  pthread_rwlock_rdlock(&this->ftl->zone_lock);
  uint16_t free_count = ftl->get_free_regions();
  pthread_rwlock_unlock(&this->ftl->zone_lock);
  return free_count < this->threshold;
}

int Calliope::select_zone() {
  int max = 0;
  float max_util = 0;
  float util = 0;

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

  this->can_reap = max_util != 0.0f;
  // std::cout << std::endl;
  return max;
}

void Calliope::reap() {
  std::cout << "Trapped in a stasis—I hate this! I haven't taken a life in "
               "like ages"
            << std::endl;

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    uint16_t free_count = ftl->get_free_regions();

    if (free_count > this->threshold) {
      std::cout << "Reporting back to death-sama" << std::endl;
      continue;
    }

    pthread_rwlock_rdlock(&this->ftl->zone_lock);
    for (int i = 0; i < this->threshold + 1; i++) {
      int zone_num = this->select_zone();
      ZNSZone &reapable = this->ftl->zones[zone_num];
      if (!this->can_reap) {
        continue;
      }
      this->can_reap = false;
      std::cout << "!!!REAP!!!" << std::endl << reapable << std::endl;
      std::vector<ZNSBlock> blocks = reapable.get_nonfree_blocks();

      // Get a free zone to store the blocks into
      ZNSZone *zone = this->ftl->get_free_zone(blocks.size());
      std::cout << "Writing to " << std::dec << zone->zone_id << " "
                << blocks.size() << " " << zone->get_current_capacity()
                << std::endl;

      // Copy data to the new zone block by block
      // TODO(valentijn): move by MDTS chunks instead
      uint64_t wp = zone->position;
      int j = 0;
      for (ZNSBlock block : blocks) {
        // TODO(valentijn): we have a nice copy command which is not working
        //   use it instead of this garbage
        char buffer[this->ftl->lba_size];
        uint32_t read_size;
        reapable.read(block.address, &buffer, this->ftl->lba_size, &read_size);
        zone->write(&buffer, this->ftl->lba_size, &read_size);
        wp += this->ftl->lba_size;
        std::cout << std::dec << j++ << " ";
      }
      std::cout << std::endl
                << "Done writing " << zone->get_current_capacity() << std::endl;
      reapable.reset();
    }
    pthread_rwlock_unlock(&this->ftl->zone_lock);
  }
}

void Calliope::initialize() {
  std::cout << "Summons Mori from hell" << std::endl;
  this->thread = std::thread(&Calliope::reap, this);
}
