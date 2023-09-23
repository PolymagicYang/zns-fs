#ifndef STOSYS_PROJECT_ZNSBLOCK_H
#define STOSYS_PROJECT_ZNSBLOCK_H
#include <cstdint>

typedef uint64_t physaddr_t;

class ZNSBlock {
public:
	physaddr_t address;
	uint64_t size;
	void *buffer;

	ZNSBlock(const physaddr_t address, const uint64_t size, const void *buffer);
};

#endif 
	
