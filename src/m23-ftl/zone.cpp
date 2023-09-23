

#include "zone.hpp"
#include "../common/nvmewrappers.h"
#include <stdlib.h>
#include <string.h>

ZNSZone::ZNSZone(const int zns_fd, const uint32_t zone_id, const uint64_t size,
				 const uint64_t capacity, const enum ZoneState state,
				 const enum ZoneZNSType zns_type, const uint64_t slba,
				 const enum ZoneFTLType ftl_type, const enum ZoneModel model,
				 const uint64_t position) {
	this->zns_fd = zns_fd;
	this->zone_id = zone_id;
	this->size = size;
	this->capacity = capacity;
	this->state = state;
	this->ftl_type = ftl_type;
	this->model = model;
	this->position = position;
	this->slba = slba;
}

// TODO(valentijn) update so it throws exceptions
inline void ZNSZone::send_management_command(const enum nvme_zns_send_action action) const {
	int ret = nmve_zns_mgmt_send(this->zns_fd, this->zone_id, this->slba, false,
								 action, 0, NULL);
	if (ret != 0) {
		print_nvme_error(ret);
	}
	   
}

void ZNSZone::close_zone(void) const {
	send_management_command(NVME_ZNS_ZSA_CLOSE);
}

void ZNSZone::open_zone(void) const {
	send_management_command(NVME_ZNS_ZSA_OPEN);
}

void ZNSZone::finish_zone(void) const {
	send_management_command(NVME_ZNS_ZSA_FINISH);
}

void ZNSZone::reset_zone(void) const {
	send_management_command(NVME_ZNS_ZSA_RESET);
}

uint64_t ZNSZone::get_current_capacity() const {
	return this->blocks.size();
}

std::ostream &operator<<(std::ostream &os, ZNSZone const &tc) {
	return os << "Zone " << tc.zone_id << " (" << tc.slba << ")" << ": " \
			  << tc.get_current_capacity() << tc.size \
			  << tc.state << " | " << tc.ftl_type << tc << tc.model \
			  << std::endl;
}

uint64_t ZNSZone::write_block(const uint16_t total_nlb, const uint16_t max_nlb_per_round,
							  const uint64_t address, const void *buffer, const uint32_t size) {
	__u64 written_slba;
	ss_nvme_zns_append(this->zns_fd, this->zone_id, this->slba, total_nlb - 1, 0, 0,
					   0, 0, size, buffer, 0, nullptr, &written_slba);
	
	const uint16_t iterations = (size != this->capacity) ? total_nlb : 1;
	
	// Update the block address to the new address based on the LBA size.
	for (uint16_t i = 0; i < iterations; i++) {
		ZNSBlock block = new ZNSBlock(address+i, written_slba+1);
		this->blocks.push_back(block);
	}
	
	// Update the write pointer.
	this->position = written_slba;
	return written_slba;	
}

int get_zns_zone_info(const int fd, const int nsid, uint64_t *zcap, uint32_t *nr,
                      struct nvme_zns_desc *desc[]) {
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

std::vector<ZNSZone&> create_zones(const int zns_fd, uint32_t nsid) {
	// TODO(valentijn): don't hard code this please
	struct nvme_zns_desc *desc = (struct nvme_zns_desc *) calloc(128, sizeof(struct nvme_zns_desc));

	uint64_t zcap;
	uint32_t nr; 
	get_zns_zone_info(zns_fd, nsid, &nr, &zcap, (struct nvme_zns_desc **) &desc);
	std::vector<ZNSZone&> zones = std::vector<ZNSZone&>();
	
	for (uint32_t i = 0; i < nr; i++) {
		struct nvme_zns_desc current = desc[i];

		const enum ZoneZNSType ztype = static_cast<ZoneZNSType>(current.zt);
		const enum ZoneState zstate = static_cast<ZoneState>(current.zs);
		// TODO(valentijn): zone attributes (p.28 ZNS Command specification)
		const uint64_t capacity = current.zcap;
		const uint64_t zone_slba = current.zslba;
		const uint64_t write_pointer = current.wp;
		
	   
		ZNSZone zone = new ZNSZone(zns_fd, i, capacity, capacity, zstate, ztype,
								   zone_slba, Data, HostManaged, write_pointer);
		zones.push_back(zone);
	}
	
	free(desc);
	return zones;
}
