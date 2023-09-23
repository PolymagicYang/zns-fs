#ifndef STOSYS_PROJECT_ZNS_ZONE_H
#define STOSYS_PROJECT_ZNS_ZONE_H
#include <cstdint>
#include <vector>
#include <iostream>
#include <libnvme.h>

#include "znsblock.hpp"

enum ZoneState {
	Empty,
	Full,
	ImplicitOpen,
	ExplicitOpen,
	Closed,
	ReadOnly,
	Offline
};

/** Type that this zone is allocated in our system. */
enum ZoneFTLType {
	Log,
	Data
};

/** Interface used for this zone.*/
enum ZoneModel {
	/** Sequential work loads and full control for the host. */
	HostManaged,
	/** Backwards compatible with block devices so that random writes
		can be issued. */
	HostAware
};

/** Zone type as defined by the ZNS standard. */
enum ZoneZNSType {
	/** Similar to block devices. Accepts random writes and does not
		have write pointers. */
	Convential,
	/** Random writes are allowed, but has a write pointer.*/
	SequentialWritePreferred,
	/** Can only be used sequentially with a write pointer. Write
		commands must be aligned with the write pointer. */
	SequentialWriteRequired
};

class ZNSZone {
public:
	uint32_t zone_id;
	ZNSZone(const int zns_fd, const uint32_t zone_id, const uint64_t size,
			const uint64_t capacity, const enum ZoneState state,
			const enum ZoneZNSType zns_type, const uint64_t slba,
			const enum ZoneFTLType ftl_type, const enum ZoneModel model,
			const uint64_t position);

	/** Zone capacity is the total optimized number of blocks in the
		region. This is always less than the zone size. */
	uint64_t capacity;

	/** Zone size is the maximum amount of blocks that can be stored
		in the zone. Typically not useful for our purposes. */
	uint64_t size;

	/** Starting logical block address of the zone. Typically used for zone commands. */
	uint64_t slba;
	
	enum ZoneState state;
	enum ZoneFTLType ftl_type;
	enum ZoneModel model;
	
	/** Writes a block to the zone and returns the updated write pointer. */
	uint32_t write_block(const uint16_t total_nlb, const uint16_t max_nlb_per_round,
						 const uint64_t address, const void *buffer, const uint32_t size);
	
	/** Removes a block from the zone and returns the updated write pointer. */
	uint64_t remove_block(const ZNSBlock &block);
	
	/** Calculates the current capacity of the block. */
	uint64_t get_current_capacity() const;

	/** Gets a block from the zone based on the block id. */
	ZNSBlock &get_block(const uint64_t block_id) const;

	/** Get victim block */
	ZNSBlock &get_victim(void);

	/** Reset the write pointer to the start. */
	void reset_zone(void) const;

	/** Open the zone explicitely so more resources are allocated. */
	void open_zone(void) const;
	
	/** Explicitely close a zone and free the resources. */
	void close_zone(void) const;

	/** Close the zone so that writes cannot be performed until it is reset. */
	void finish_zone(void) const;
	
	friend std::ostream &operator<<(std::ostream &os, ZNSZone const &tc);
	
private:
	uint64_t position;
	uint64_t lba_size_bytes;
	uint64_t mdts_size;
	int zns_fd;
	
	/** Number of blocks in the zone reserved for GC in percentages. */
	const uint64_t overcapacity = 5;
	
	std::vector<ZNSBlock> blocks;
	inline void send_management_command(const enum nvme_zns_send_action action) const;
};
	
std::ostream &operator<<(std::iostream &os, ZNSZone const &tc);
int get_zns_zone_info(const int fd, const int nsid, uint64_t *zcap, uint32_t *nr, struct nvme_zns_desc *desc[]);
std::vector<ZNSZone&> create_zones(const int zns_fd, uint32_t nsid);


