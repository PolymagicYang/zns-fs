/*
 * MIT License
Copyright (c) 2021 - current
Authors:  Animesh Trivedi
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


#include <cerrno>
#include <cstdint>
#include <nvme/ioctl.h>
#include <nvme/types.h>
#include <nvme/util.h>
#include <stdio.h>
#include <sys/mman.h>
#include <math.h>

#include <libgen.h>
#include <sys/types.h>
#include <unistd.h>
#include <unordered_map>
#include <variant>
#include "../common/unused.h"
#include "zns_device.h"

using Ftlmap = std::unordered_map<u_int64_t, u_int64_t>; 
struct None {};
struct Addr { u_int64_t addr; };
using AddressOp = std::variant<None, Addr>;
// TODO: use lock to enable map thread-safe (e.g. rw-lock), maybe we should design a map with fined-grained parallelism.

int get_zns_zone_zcap(int fd, int nsid, u_int64_t *zcap){
    // coypied from m1 to fetch zcap.
    char* zone_reports;

    struct nvme_zone_report zns_report{};
    struct nvme_zns_desc *desc = NULL;
    int ret;
    uint64_t num_zones;
    ret = nvme_zns_mgmt_recv(fd, nsid, 0,
                             NVME_ZNS_ZRA_REPORT_ZONES, NVME_ZNS_ZRAS_REPORT_ALL,
                             0, sizeof(zns_report), (void *)&zns_report);
    if(ret != 0) {
        fprintf(stderr, "failed to report zones, ret %d \n", ret);
        return ret;
    }
    num_zones = le64_to_cpu(zns_report.nr_zones);
    uint64_t total_size = sizeof(zns_report) + (num_zones * sizeof(struct nvme_zns_desc));
    zone_reports = (char*) calloc (1, total_size);
    ret = nvme_zns_mgmt_recv(fd, nsid, 0,
                             NVME_ZNS_ZRA_REPORT_ZONES, NVME_ZNS_ZRAS_REPORT_ALL,
                             1, total_size, (void *) zone_reports);
    if(ret !=0) {
        fprintf(stderr, "failed to report zones, ret %d \n", ret);
        return ret;
    }
    desc = ((struct nvme_zone_report*) zone_reports)->entries;
    num_zones = le64_to_cpu(((struct nvme_zone_report*) zone_reports)->nr_zones);
    *zcap = (uint64_t)le64_to_cpu(desc->zcap);

    free(zone_reports);
    return ret;
}

template<typename T>
class List {
    public:
        virtual void append();
        // if buffer size equals logical block size, map directly.
        virtual void remove(u_int64_t index);
        virtual T get(u_int64_t index);
        virtual T& get_range(u_int64_t start, u_int64_t end); // not include upper bound: [1: 2](1, 2) => [1].
        ~List<T>();

};

template<typename T>
class ArrayList : public List<T> {

};

template<typename T>
class LinkedList : public List<T> {

};

template<typename T>
class FTL {
    public:
        int fd;
        u_int32_t nsid; 
        u_int64_t mdts_size;
        FTL<T>(int fd, u_int64_t mdts_size, u_int32_t nsid, u_int64_t wp) {
            this->fd = fd;
            this->wp = wp; // write pointer, record the current log address.
            this->mdts_size = mdts_size;
            this->nsid = nsid;

            this->log_map = std::unordered_map<u_int64_t, u_int64_t>();
            this->data_map = std::unordered_map<u_int64_t, u_int64_t>();
            this->list = LinkedList<int>();
        };

        AddressOp get_pa(u_int64_t addr) {
            // search log map firstly, if no return then try data log.
            AddressOp ret = this->get_ppa(addr);
            if (Addr *addr =  std::get_if<Addr>(&ret)) {
                return *addr;
            } else {
                AddressOp ret = this->get_pba(addr->addr);
                if (Addr *addr = std::get_if<Addr>(&ret)) {
                    return *addr;
                } else {
                    return AddressOp { None {} };
                }
            }
        };

        void insert_lba(u_int64_t addr) {
            
        }
        
    private:
        Ftlmap log_map;
        Ftlmap data_map;
        List<T> list;
        u_int64_t wp;
        u_int32_t zcap;
        // return physical page address from log map.
        AddressOp get_ppa(u_int64_t lba) {
            auto ret = this->log_map.find(lba);
            if (ret == this->log_map.end()) {
                return AddressOp { None {} };
            } else {
                return AddressOp { Addr { .addr = ret->first } };
            }
        }

        // return physical block address from data map.
        AddressOp get_pba(u_int64_t lba) {
            auto ret = this->data_map.find(lba);
            if (ret == this->data_map.end()) {
                return AddressOp { None {} };
            } else {
                return AddressOp { Addr { .addr = ret->first } };
            }
        }
};

extern "C" {
int deinit_ss_zns_device(struct user_zns_device *my_dev) {    
    int ret = -ENOSYS;
    // this is to supress gcc warnings, remove it when you complete this function 
    UNUSED(my_dev);
    return ret;
}

int init_ss_zns_device(struct zdev_init_params *params, struct user_zns_device **my_dev) {    
    int fd;
    int ret;
    void *regs;
    u_int64_t wp = 0;
    LinkedList<int> list;
    struct nvme_id_ns ns {};
    struct nvme_id_ctrl ctrl;

    fd = nvme_open(params->name);
    if (fd < 0) {
        printf("device %s opening failed %d errno %d \n", params->name, fd, errno);
        return -fd;
    }
    uint32_t nsid;
    ret = nvme_get_nsid(fd, &nsid);
    if (ret != 0) {
        printf("ERROR: failed to retrieve the nsid %d \n", ret);
        return ret;
    }

    ret = nvme_identify_ns(fd, nsid, &ns);
    if (ret) {
        printf("ERROR: failed to retrieve the nsid %d \n", ret);
        return ret;
    }
    u_int8_t lba_size_in_use = 1 << ns.lbaf[(ns.flbas & 0xf)].ds;

    // Calculate MDTS_size for later use.
    regs = mmap(NULL, getpagesize(), PROT_READ, MAP_SHARED, fd, 0);
    munmap(regs, getpagesize());
    uint64_t cap = nvme_mmio_read64((void *) ((uint64_t) regs + NVME_REG_CAP));
    uint64_t mpsmin_raw = (cap & ((uint64_t) NVME_CAP_MPSMIN_SHIFT << NVME_CAP_MPSMIN_SHIFT)) >> NVME_CAP_MPSMIN_SHIFT;
    uint64_t MPSMIN = pow(2, 12 + mpsmin_raw);
    nvme_identify_ctrl(fd, &ctrl);
    uint64_t MDTS = (uint64_t) ctrl.mdts - 1;
    uint64_t MDTS_SIZE = pow(2, MDTS) * MPSMIN; 

    FTL<int> ftl = FTL<int>(fd, MDTS_SIZE, nsid, wp);
    (*my_dev)->_private = &ftl; // Store FTL metadata.
    (*my_dev)->lba_size_bytes = lba_size_in_use; // multiple of lba size?
    // FTL metadata: dynamic allocation? Suppose we have 
    (*my_dev)->capacity_bytes = ns.ncap * lba_size_in_use; // ZNS capacity - (log zones?) - FTL metadata.

    return 0;    
}

int zns_udevice_read(struct user_zns_device *my_dev, uint64_t address, void *buffer, uint32_t size) {
    // Check log map firstly beacuse of fresh data, then check data map, one page size at a time (to avoid overwrite).
    // e.g. full sequantially writes on address 0x0 to make a data map entry, then 1 page write on 0x2, the data in log map will be newer than data map.
    FTL<int> *flt = (FTL<int> *) my_dev->_private;
    u_int32_t pages_num = size / my_dev->lba_size_bytes;

    u_int64_t phas[pages_num];
    for (u_int64_t i = 0; i < pages_num; i++) {
        AddressOp addrop = flt->get_pa(address + i);
        if (Addr *addr_wrapper = std::get_if<Addr>(&addrop)) {
            u_int64_t addr = addr_wrapper->addr;
            phas[i] = addr;
        } else {
            // Invalid logical page address.
            return -1;
        }
    }
    // find a algoritm that check all the sequential physical addresses to make it more effcient.

    /*
    if (size > flt->mdts_size) {
        // make use of mdts_size, number of blocks: (maximum data we can tranfer) / (zns logical block size).
        uint16_t nlb = flt->mdts_size / my_dev->lba_size_bytes;
        nvme_read(flt->fd, flt->mdts_size, addr, nlb, 0, 0, 0, 0, 0, __u32 data_len, void *data, 0, 0);
    } else {

    }
    */

    for (int i = 0; i < pages_num; i++) {
        // should be optimized later.
        u_int32_t block_size = my_dev->lba_size_bytes;
        void *data_ptr = (void *) (((u_int64_t) buffer) + i * block_size);
        int ret = nvme_read(flt->fd, flt->mdts_size, phas[i], 1, 0, 0, 0, 0, 0, block_size, data_ptr, 0, 0);
        if (ret != 0) {
            return ret;
        }
    }
    return 0;
}

int zns_udevice_write(struct user_zns_device *my_dev, uint64_t address, void *buffer, uint32_t size) {
    u_int64_t *zcap;
    FTL<int> *flt = (FTL<int> *) my_dev->_private;
    u_int32_t pages_num = size / my_dev->lba_size_bytes;
    get_zns_zone_zcap(flt->fd, flt->nsid, zcap);

    // if data size == zone size => store them in the block map.
    if (*zcap == size) {
        
    }
}



}
