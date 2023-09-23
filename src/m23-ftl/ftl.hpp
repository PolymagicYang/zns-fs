#ifndef FLT_H
#define FLT_H

#include <pthread.h>
#include <sys/types.h>

#include <cstdint>
#include <map>
#include <unordered_map>
#include <vector>

#include "zone.hpp"

struct Addr {
  uint64_t addr;
  uint16_t zone_num;
};

using raw_map = std::map<uint64_t, struct Addr>;

struct Ftlmap {
  pthread_rwlock_t lock;
  raw_map map;
};

class FTL {
 public:
  int fd;
  int gc_wmark;
  int zones_num;
  uint32_t zcap;
  uint32_t nsid;
  uint64_t mdts_size;
  uint16_t lba_size;
  int log_zones;
  std::vector<ZNSZone> zones;

  FTL(int fd, uint64_t mdts, uint32_t nsid, uint16_t lba_size, int gc_wmark,
      int log_num);

  ~FTL() {
    this->zones.clear();
    pthread_rwlock_destroy(&this->log_map.lock);
    pthread_rwlock_destroy(&this->data_map.lock);
    // Destroy the locks
    pthread_rwlock_destroy(&log_map.lock);
    pthread_rwlock_destroy(&data_map.lock);

    data_map.map.clear();
    log_map.map.clear();
  }

  Addr get_pa(uint64_t);
  int read(uint64_t addr, void* buffer, uint32_t size);
  int write(uint64_t addr, void* buffer, uint32_t size);

  // return index of all the free log zones.
  std::vector<int> get_free_logzones();

  // return index of all the free log zones.
  std::vector<int> get_free_datazones();

  ZNSZone* get_zone(int index);
  ZNSZone* get_random_logzone();
  ZNSZone* get_random_datazone();

  void insert_logmap(uint64_t lba, uint64_t pa, uint16_t zone_num);

  void insert_datamap(uint64_t lba, uint64_t pa, uint16_t zone_num);

 private:
  struct Ftlmap log_map;
  struct Ftlmap data_map;
  // return physical page address from log map.
  bool get_ppa(uint64_t, Addr*);

  // return physical block address from data map.
  Addr get_pba(uint64_t);
};

#endif
