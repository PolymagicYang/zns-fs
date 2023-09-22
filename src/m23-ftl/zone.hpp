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
	ZNSZone(int zns_fd, uint32_t zone_id, uint64_t zone_size, uint64_t capacity,
			enum ZoneState state, enum ZoneFTLType ftl_type, enum ZoneModel model,
			uint64_t position);

	/** Zone capacity is the total optimized number of blocks in the
		region. This is always less than the zone size. */
	const uint64_t capacity;

	/** Zone size is the maximum amount of blocks that can be stored
		in the zone. Typically not useful for our purposes. */
	const uint64_t size;

	/** Starting logical block address of the zone. Typically used for zone commands. */
	const uint64_t slba;
	
	enum ZoneState state;
	enum ZoneFTLType ftl_type;
	enum ZoneModel model;
	
	/** Writes a block to the zone and returns the updated write pointer. */
	uint64_t write_block(const ZNSBlock &block);
	
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
	int zns_fd;
	
	/** Number of blocks in the zone reserved for GC in percentages. */
	uint64_t overcapacity = 5;
	
	std::vector<ZNSBlock> blocks;
	inline void send_management_command(const enum nvme_zns_send_action action) const;
};
	
std::ostream &operator<<(std::iostream &os, ZNSZone const &tc);


