

#include "zone.hpp"

#include <stdlib.h>
#include <string.h>

#include "../common/nvmewrappers.h"

ZNSZone::ZNSZone(const int zns_fd, const uint32_t nsid, const uint32_t zone_id,
                 const uint64_t size, const uint64_t capacity,
                 const enum ZoneState state, const enum ZoneZNSType zns_type,
                 const uint64_t slba, const enum ZoneFTLType ftl_type,
                 const enum ZoneModel model, const uint64_t position,
                 const uint64_t lba_size, const uint64_t mdts_size) {
  this->zns_fd = zns_fd;
  this->nsid = nsid;
  this->zone_id = zone_id;
  this->size = size;
  this->capacity = capacity;
  this->state = state;
  this->ftl_type = ftl_type;
  this->model = model;
  this->position = position;
  this->base = position;
  this->slba = slba;
  this->lba_size = lba_size;
  this->mdts_size = mdts_size;
}

// TODO(valentijn) update so it throws exceptions
inline int ZNSZone::send_management_command(
    const enum nvme_zns_send_action action) const {
  int ret = nvme_zns_mgmt_send(this->zns_fd, this->nsid, this->slba, false,
                               action, 0, NULL);
  if (ret != 0) {
    print_nvme_error(ret);
  }
  return ret;
}

int ZNSZone::get_index() {
  pthread_rwlock_rdlock(&this->lock);
  int ret = this->zone_id;
  pthread_rwlock_unlock(&this->lock);
  return ret;
}

int ZNSZone::reset() {
  pthread_rwlock_wrlock(&this->lock);
  this->position = this->base;
  int ret = this->reset_zone();
  pthread_rwlock_unlock(&this->lock);
  return ret;
};

bool ZNSZone::is_full() {
  pthread_rwlock_rdlock(&this->lock);
  bool ret = this->get_current_capacity() == 0;
  pthread_rwlock_unlock(&this->lock);
  return ret;
}

/*
WARN: unsafe function, read and write will call this function with their own
lock. Use this method with a lock!
*/
uint32_t ZNSZone::get_current_capacity() const {
  return this->capacity + this->base - this->position;
}

uint64_t ZNSZone::get_wp() {
  pthread_rwlock_rdlock(&this->lock);
  uint64_t ret = this->position;
  pthread_rwlock_unlock(&this->lock);
  return ret;
}

int ZNSZone::close_zone(void) const {
  return send_management_command(NVME_ZNS_ZSA_CLOSE);
}

int ZNSZone::open_zone(void) const {
  return send_management_command(NVME_ZNS_ZSA_OPEN);
}

int ZNSZone::finish_zone(void) const {
  return send_management_command(NVME_ZNS_ZSA_FINISH);
}

int ZNSZone::reset_zone(void) const {
  return send_management_command(NVME_ZNS_ZSA_RESET);
}

const char *get_state_text(const enum ZoneState state) {
  return (state == Empty)
             ? "Empty"
             : (state == Full)
                   ? "Full"
                   : (state == ImplicitOpen)
                         ? "Implicit Open"
                         : (state == ExplicitOpen)
                               ? "Explicit Open"
                               : (state == Closed)
                                     ? "Closed"
                                     : (state == ReadOnly)
                                           ? "Read Only"
                                           : (state == Offline) ? "Offline"
                                                                : "Unknown";
}

const char *get_ftl_type_text(const enum ZoneFTLType type) {
  return (type == Log) ? "Log" : (type == Data) ? "Data" : "Unknown";
}

const char *get_zone_model_text(const enum ZoneModel model) {
  return (model == HostManaged)
             ? "Host Managed"
             : (model == HostAware) ? "Host Aware" : "Unknown";
}

const char *get_zone_zns_type(const enum ZoneZNSType type) {
  return (type == Convential) ? "Convential"
                              : (type == SequentialWritePreferred)
                                    ? "Sequential Write Preferred"
                                    : (type == SequentialWriteRequired)
                                          ? "Sequential Write Required"
                                          : "Unknown";
}

std::ostream &operator<<(std::ostream &os, ZNSZone const &tc) {
  const char *state = get_state_text(tc.state);
  const char *ftl_type = get_ftl_type_text(tc.ftl_type);
  const char *model = get_zone_model_text(tc.model);

  return os << "Zone " << tc.zone_id << std::dec << std::endl
            << "====================================" << std::endl
            << "Starting Logical Block Address: "
            << "0x" << std::hex << tc.slba << std::dec << std::endl
            << "Usage: " << tc.get_current_capacity() << " out of " << tc.size
            << std::endl
            << "Metadata: " << state << " | " << ftl_type << " | " << model
            << std::endl;
}

int ZNSZone::ss_sequential_write(const void *buffer,
                                 const uint16_t max_nlb_per_round,
                                 const uint16_t total_nlb) {
  uint64_t i;
  uint64_t data_len = max_nlb_per_round * this->lba_size;
  void *buffer_ptr;
  for (i = 0; i < total_nlb; i += max_nlb_per_round) {
    buffer_ptr = (void *)((uint64_t)buffer + i * this->lba_size);
    int ret = ss_nvme_write(this->zns_fd, this->nsid, this->position,
                            max_nlb_per_round - 1, 0, 0, 0, 0, 0, 0, data_len,
                            buffer_ptr, 0, nullptr);
    if (ret != 0) {
      return ret;
    }
    this->position += max_nlb_per_round;
  }
  if (i > total_nlb) {
    // write remaining blocks.
    uint64_t ream_blocks = (total_nlb - (i - max_nlb_per_round));
    buffer_ptr = (void *)((uint64_t)buffer + i * this->lba_size);
    data_len = ream_blocks * this->lba_size;
    int ret =
        ss_nvme_write(this->zns_fd, this->nsid, this->position, ream_blocks - 1,
                      0, 0, 0, 0, 0, 0, data_len, buffer_ptr, 0, nullptr);
    if (ret != 0) {
      return ret;
    }
    this->position += ream_blocks;
  }

  return 0;
}

/*
return the size of the inserted buffer.
*/
uint32_t ZNSZone::write(void *buffer, uint32_t size, uint32_t &write_size) {
  pthread_rwlock_wrlock(&this->lock);
  uint32_t max_writes = this->get_current_capacity() * this->lba_size;
  write_size = (size > max_writes) ? max_writes : size;

  // size is the multiple of lba_size.
  uint16_t total_nlb = size / this->lba_size;
  uint16_t max_nlb_per_round = this->mdts_size / this->lba_size;

  if (size <= this->mdts_size) {
    int ret =
        ss_nvme_write(this->zns_fd, this->nsid, this->position, total_nlb - 1,
                      0, 0, 0, 0, 0, 0, size, buffer, 0, nullptr);
    if (ret != 0) {
      return ret;
    }
    this->position += total_nlb;
  } else {
    int ret = ss_sequential_write(buffer, max_nlb_per_round, total_nlb);
    if (ret != 0) return ret;
  }

  pthread_rwlock_unlock(&this->lock);
  
  return 0;
};

uint64_t ZNSZone::write_block(const uint16_t total_nlb,
                              const uint16_t max_nlb_per_round,
                              const uint64_t address, const void *buffer,
                              const uint32_t size) {
  __u64 written_slba;
  ss_nvme_zns_append(this->zns_fd, this->nsid, this->slba, total_nlb - 1, 0, 0,
                     0, 0, size, (void *)buffer, 0, nullptr, &written_slba);

  const uint16_t iterations = (size != this->capacity) ? total_nlb : 1;

  // Update the block address to the new address based on the LBA size.
  for (uint16_t i = 0; i < iterations; i++) {
    ZNSBlock block = ZNSBlock(address + i, written_slba + 1, buffer);

    this->blocks.push_back(block);
  }

  // Update the write pointer.
  this->position = written_slba + 1;
  return written_slba;
}

int get_zns_zone_info(const int fd, const int nsid, uint64_t *zcap,
                      uint32_t *nr, struct nvme_zns_desc *desc[]) {
  // copied from m1 to fetch zcap.
  struct nvme_zone_report zns_report {};
  int ret;
  uint64_t num_zones;
  ret = nvme_zns_mgmt_recv(fd, nsid, 0, NVME_ZNS_ZRA_REPORT_ZONES,
                           NVME_ZNS_ZRAS_REPORT_ALL, 0, sizeof(zns_report),
                           (void *)&zns_report);
  if (ret != 0) {
    fprintf(stderr, "failed to report zones, ret %d \n", ret);
    return ret;
  }
  num_zones = le64_to_cpu(zns_report.nr_zones);
  uint64_t total_size =
      sizeof(zns_report) + (num_zones * sizeof(struct nvme_zns_desc));

  struct nvme_zone_report *zone_reports =
      (struct nvme_zone_report *)calloc(1, total_size);
  ret = nvme_zns_mgmt_recv(fd, nsid, 0, NVME_ZNS_ZRA_REPORT_ZONES,
                           NVME_ZNS_ZRAS_REPORT_ALL, 1, total_size,
                           (void *)zone_reports);
  if (ret != 0) {
    fprintf(stderr, "failed to report zones, ret %d \n", ret);
    return ret;
  }

  num_zones = le64_to_cpu(zone_reports->nr_zones);
  memcpy(*desc, (const void *)&zone_reports->entries,
         sizeof(struct nvme_zns_desc) * num_zones);
  *zcap = (uint64_t)le64_to_cpu(zone_reports->entries[1].zcap);
  *nr = num_zones;

  free(zone_reports);
  return ret;
}

uint32_t ZNSZone::read(const uint64_t lba, const void *buffer, uint32_t size) {
  pthread_rwlock_rdlock(&this->lock);
  if (lba + size / this->lba_size > this->position + this->capacity) {
    // cross boundary read.
    size = (this->position + this->capacity - lba) * this->lba_size;
  }
  uint32_t nlb = size / this->lba_size;
  uint32_t ret = size;

  int read_ret =
      ss_nvme_read(this->zns_fd, this->nsid, lba, nlb - 1, 0, 0, 0, 0, 0,
                   nlb * this->lba_size, (void *)buffer, 0, nullptr);
  if (read_ret != 0) {
    return read_ret;
  }

  pthread_rwlock_unlock(&this->lock);
  return ret;
}

extern "C" {
std::vector<ZNSZone> create_zones(const int zns_fd, const uint32_t nsid,
                                  const uint64_t lba_size,
                                  const uint64_t mdts_size) {
  // TODO(valentijn): don't hard code this please
  struct nvme_zns_desc *desc =
      (struct nvme_zns_desc *)calloc(128, sizeof(struct nvme_zns_desc));

  uint32_t zcap;
  uint64_t nr;
  get_zns_zone_info(zns_fd, nsid, &nr, &zcap, (struct nvme_zns_desc **)&desc);
  std::vector<ZNSZone> zones = std::vector<ZNSZone>();

  for (uint32_t i = 0; i < nr; i++) {
    struct nvme_zns_desc current = desc[i];

    const enum ZoneZNSType ztype = static_cast<ZoneZNSType>(current.zt);
    const enum ZoneState zstate = static_cast<ZoneState>(current.zs);
    // TODO(valentijn): zone attributes (p.28 ZNS Command specification)
    const uint64_t capacity = current.zcap;
    const uint64_t zone_slba = le64_to_cpu(current.zslba);
    const uint64_t write_pointer = current.wp;

    ZNSZone zone =
        ZNSZone(zns_fd, nsid, i, capacity, capacity, zstate, ztype, zone_slba,
                Data, HostManaged, write_pointer, lba_size, mdts_size);
    zone.reset_zone();
    zones.push_back(zone);
  }

  free(desc);
  return zones;
}
}
