#ifndef STOSYS_PROJECT_STORAGE_H
#define STOSYS_PROJECT_STORAGE_H

#include "structures.h"

void *get_from_disk(const uint64_t lba, const uint32_t size);
void store_on_disk(const uint64_t lba, const uint32_t size, const char *data);

	


#endif
