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

#ifndef FLT_H
#define FLT_H

#include <pthread.h>
#include <sys/types.h>

#include <cstdint>
#include <map>
#include <unordered_map>
#include <vector>

#include "datazone.hpp"
#include "logzone.hpp"

struct Addr {
  uint64_t addr;
  uint16_t zone_num;
  bool alive;
};

using raw_map = std::unordered_map<uint64_t, struct Addr>;

struct Ftlmap {
  pthread_rwlock_t lock;
  raw_map map;
};

class FTL {
 public:
  int fd;
  int gc_wmark;
  bool force_reset;
  uint32_t zcap;
  uint32_t nsid;
  uint64_t mdts_size;
  uint16_t lba_size;
  pthread_cond_t need_gc;
  pthread_mutex_t need_gc_lock;

  pthread_cond_t clean_finish;
  pthread_mutex_t clean_finish_lock;

  int log_zones;
  uint64_t init_code;

  /** Store a list of all the zones in the system */
  std::vector<ZNSLogZone> zones;

  // A variable to hold our GC object. We set to void so we don't
  // get into a circulair dpeendency of header imports
  void* mori;

  FTL(int fd, uint64_t mdts, uint32_t nsid, uint16_t lba_size, int gc_wmark,
      int log_num, bool force_reset);

  ~FTL() {
    this->zones.clear();
    pthread_rwlock_destroy(&this->log_map.lock);
    pthread_rwlock_destroy(&this->data_map.lock);
    // Destroy the locks
    pthread_rwlock_destroy(&log_map.lock);
    pthread_rwlock_destroy(&data_map.lock);

    pthread_rwlock_destroy(&zone_lock);
    data_map.map.clear();
    log_map.map.clear();
  }

  inline bool has_pa(uint64_t);
  int read(uint64_t addr, void* buffer, uint32_t size);
  int write(uint64_t addr, void* buffer, uint32_t size);

  // return index of all the free log zones.
  std::vector<int> get_free_logzones();

  void backup();

  // return index of all the free log zones.
  std::vector<int> get_free_datazones();

  /** Get a free zone with enough space to hold a number of blocks. */
  ZNSLogZone* get_free_log_zone();

  ZNSDataZone* get_free_data_zone(const uint32_t needed);

  void insert_logmap(uint64_t lba, uint64_t pa, uint16_t zone_num);

  /** Get the number of free regions in our system */
  int16_t get_free_log_regions();

  void insert_datamap(uint64_t lba, uint64_t pa, uint16_t zone_num);

  void delete_logmap(uint64_t lba);

  struct Ftlmap log_map;
  struct Ftlmap data_map;
  pthread_rwlock_t zone_lock;

  std::vector<ZNSLogZone> zones_log;
  std::vector<ZNSDataZone> zones_reserved;
  std::vector<ZNSDataZone> zones_data;
  // return physical page address from log map.
  bool get_ppa(uint64_t, Addr*);

  bool get_pba_by_base(uint64_t, Addr*);

  // return physical block address from data map.
  bool get_pba(uint64_t, Addr*);

  // Given a base address to check the existence of the data entry.
  bool pba_exist(uint64_t);

  pthread_rwlock_t zones_lock;
  std::vector<ZNSLogZone*> free_log_zones;
  std::vector<ZNSDataZone*> free_data_zones;
};

#endif
