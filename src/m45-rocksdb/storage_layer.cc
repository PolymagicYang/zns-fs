#include "storage_layer.hpp"
#include <iostream>
#include <random>

std::map<uint64_t, struct ss_data> fake_data_storage = std::map<uint64_t, struct ss_data>();
inline std::random_device rd;
inline std::mt19937 rng(rd());
inline std::uniform_int_distribution<uint64_t> uni(1 << 4, 1 << 12);

void store_block_on_disk(const uint64_t lba, const struct ss_data *data) {	
	fake_data_storage[lba] = *data;
}

uint64_t store_segment_on_disk(const uint8_t nblocks, const struct ss_data *data) {
	uint64_t lba = static_cast<uint64_t>(uni(rng));

	for (uint8_t i = 0; i < nblocks; i++) {
		store_block_on_disk(lba+i, &data[i]);
	}

	return lba;
}


struct ss_data *get_from_disk(const uint64_t lba) {
	return &fake_data_storage[lba];
}
