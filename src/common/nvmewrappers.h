#ifndef NVME_WRAPPERS_H_
#define NVME_WRAPPERS_H_
#pragma once

#define EARLY_EXIT
#include <libnvme.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
void print_nvme_error(const char *type, const int ret);
int ss_nvme_write(int fd, __u32 nsid, __u64 slba, __u16 nlb, __u16 control,
                  __u8 dsm, __u16 dspec, __u32 reftag, __u16 apptag,
                  __u16 appmask, __u32 data_len, void *data, __u32 metadata_len,
                  void *metadata);
int ss_nvme_read(int fd, __u32 nsid, __u64 slba, __u16 nlb, __u16 control,
                 __u8 dsm, __u32 reftag, __u16 apptag, __u16 appmask,
                 __u32 data_len, void *data, __u32 metadata_len,
                 void *metadata);

int ss_nvme_zns_append(int fd, __u32 nsid, __u64 zslba, __u16 nlb,
                       __u16 control, __u32 ilbrt, __u16 lbat, __u16 lbatm,
                       __u32 data_len, void *data, __u32 metadata_len,
                       void *metadata, __u64 *result);

int ss_device_zone_reset(int fd, uint32_t nsid, uint64_t slba);

int ss_nvme_write_zeros(int fd, __u32 nsid, __u64 slba, __u16 nlb,
                        __u16 control, __u32 reftag, __u16 apptag,
                        __u16 appmask);

int ss_nvme_zns_mgmt_send(int, unsigned int, unsigned long long, bool,
                          nvme_zns_send_action, unsigned int, void *);

int ss_nvme_write_wrapper(int fd, uint32_t nsid, uint64_t slba, uint16_t nlb, uint32_t size, void *data);

int ss_nvme_read_wrapper(int fd, uint32_t nsid, uint64_t slba, uint16_t nlb, uint32_t size, void *data); 

#ifdef __cplusplus
}
#endif

#endif
