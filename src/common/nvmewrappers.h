#ifndef NVME_WRAPPERS_H_
#define NVME_WRAPPERS_H_
#pragma once

#define EARLY_EXIT
#include <libnvme.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
void print_nvme_error(const int ret);
int ss_nvme_write(int fd, __u32 nsid, __u64 slba, __u16 nlb, __u16 control,
                  __u8 dsm, __u16 dspec, __u32 reftag, __u16 apptag,
                  __u16 appmask, __u32 data_len, void *data, __u32 metadata_len,
                  void *metadata);
int ss_nvme_read(int fd, __u32 nsid, __u64 slba, __u16 nlb, __u16 control,
                 __u8 dsm, __u32 reftag, __u16 apptag, __u16 appmask,
                 __u32 data_len, void *data, __u32 metadata_len,
                 void *metadata);

#ifdef __cplusplus
}
#endif

#endif
