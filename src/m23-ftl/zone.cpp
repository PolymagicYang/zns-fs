

#include "zone.hpp"
#include "../common/nvmewrappers.h"

ZNSZone::ZNSZone(int zns_fd, uint32_t zone_id, uint64_t size, uint64_t capacity,
				 enum ZoneState state, enum ZoneFTLType ftl_type, enum ZoneModel model,
				 uint64_t position) {
	this->zns_fd = zns_fd;
	this->zone_id = zone_id;
	this->size = size;
	this->capacity = capacity;
	this->state = state;
	this->ftl_type = ftl_type;
	this->model = model;
	this->position = position;
	this->slba = 0;
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





