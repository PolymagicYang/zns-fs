// Simple wrapper around nvme_write that takes the function and prints
// the human readable error messagea
#include "nvmewrappers.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern "C" {
void print_nvme_error(const char *type, const int ret) {
  fprintf(stderr, "NVMe %s error: %s\n", type,
          nvme_status_to_string(ret, false));
}

int ss_nvme_write(int fd, __u32 nsid, __u64 slba, __u16 nlb, __u16 control,
                  __u8 dsm, __u16 dspec, __u32 reftag, __u16 apptag,
                  __u16 appmask, __u32 data_len, void *data, __u32 metadata_len,
                  void *metadata) {
  int32_t ret =
      nvme_write(fd, nsid, slba, nlb, control, dsm, dspec, reftag, apptag,
                 appmask, data_len, data, metadata_len, metadata);

  if (ret == -1) {
    perror("ss_nvme_write() failed");
  } else if (ret != 0) {
    printf("%d\n", ret);
    print_nvme_error("write", ret);
#ifdef EARLY_EXIT
    exit(ret);
#endif
  }

  return ret;
}

int ss_nvme_zns_append(int fd, __u32 nsid, __u64 zslba, __u16 nlb,
                       __u16 control, __u32 ilbrt, __u16 lbat, __u16 lbatm,
                       __u32 data_len, void *data, __u32 metadata_len,
                       void *metadata, __u64 *result) {
  uint32_t ret =
      nvme_zns_append(fd, nsid, zslba, nlb, control, ilbrt, lbat, lbatm,
                      data_len, data, metadata_len, metadata, result);
  if (ret == -1) {
    perror("ss_nvme_append failed");
  } else if (ret != 0) {
    print_nvme_error("append", ret);
#ifdef EARLY_EXIT
    exit(ret);
#endif
  }

  return ret;
}

int ss_nvme_write_zeros(int fd, __u32 nsid, __u64 slba, __u16 nlb,
                        __u16 control, __u32 reftag, __u16 apptag,
                        __u16 appmask) {
  return nvme_write_zeros(fd, nsid, slba, nlb, control, reftag, apptag,
                          appmask);
}

int ss_device_zone_reset(int fd, uint32_t nsid, uint64_t slba) {
  // this is to supress gcc warnings, remove it when you complete this function
  __u32 cdw10 = slba & 0xffffffff;
  __u32 cdw11 = slba >> 32;
  __u32 cdw13 =
      1 << 2;  // 08 sets to 0, 04h as the reset zone, and others reserved.
  return nvme_io_passthru(fd, nvme_zns_cmd_mgmt_send, 0, 0, nsid, 0, 0, cdw10,
                          cdw11, 0, cdw13, 0, 0, 0, nullptr, 0, nullptr, 0,
                          NULL);
}

int ss_nvme_zns_mgmt_send(int t1, unsigned int t2, unsigned long long t3,
                          bool t4, nvme_zns_send_action t5, unsigned int t6,
                          void *t7) {
  return nvme_zns_mgmt_send(t1, t2, t3, t4, t5, t6, t7);
}

int ss_nvme_read(int fd, __u32 nsid, __u64 slba, __u16 nlb, __u16 control,
                 __u8 dsm, __u32 reftag, __u16 apptag, __u16 appmask,
                 __u32 data_len, void *data, __u32 metadata_len,
                 void *metadata) {
  int32_t ret = nvme_read(fd, nsid, slba, nlb, control, dsm, reftag, apptag,
                          appmask, data_len, data, metadata_len, metadata);

  if (ret == -1) {
    perror("ss_nvme_write() failed");
  } else if (ret != 0) {
    print_nvme_error("read", ret);
#ifdef EARLY_EXIT
    exit(ret);
#endif
  }
  return ret;
}
}
