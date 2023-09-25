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
#include <chrono>

#include "ftlgc.hpp"
#include <pthread.h>

Calliope::Calliope(FTL *ftl, const uint16_t threshold) {
	this->ftl = ftl;
	this->threshold = 3;
}

bool Calliope::needs_reaping() {
	uint16_t free_count = 0;
	
	for (int i = 0; i < ftl->zones_num; i++) {
		ZNSZone zone = ftl->zones[i];
		if (!zone.is_full())
			free_count++;
	}
	
	return free_count < this->threshold;	
}

ZNSZone Calliope::select_zone() {
	ZNSZone min = ftl->zones[0];
	float min_util = 1;
	float util = 0;
	
	for (int i = 0; i < ftl->zones_num; i++) {
		std::cout << i << " ";
		ZNSZone current = ftl->zones[i];
		if (!current.is_full()) continue;		

		if (min.get_alive_capacity() == 0)
			min_util = 1;
		else 
			min_util = min.get_current_capacity() / min.get_alive_capacity();
		if (current.get_alive_capacity() == 0)
			util = 1;
		else 
			util = current.get_current_capacity() / current.get_alive_capacity();
		
		if (util < min_util)
			min = current;
	}
	std::cout << std::endl;
	return min;
}

void Calliope::reap() {
	std::cout << "Trapped in a stasis—I hate this! I haven't taken a life in like ages, okay" << std::endl;
		
	while(true) {
		std::cout << "Test" << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		pthread_rwlock_rdlock(&this->ftl->log_map.lock);
		if (!needs_reaping()) {
			std::cout << "Reporting back to death-sama" << std::endl;
			pthread_rwlock_unlock(&this->ftl->log_map.lock);
			continue;
		}

		std::cout << "Finding a dead beat" << std::endl;
		ZNSZone reapable = this->select_zone();
		std::cout << "!!!REAP!!!" << std::endl << reapable << std::endl;
		pthread_rwlock_unlock(&this->ftl->log_map.lock);
	}	
}

void Calliope::initialize() {
	std::cout << "Summons Mori from hell" << std::endl;
	this->thread = std::thread(&Calliope::reap, this);
}
