// Simple wrapper around nvme_write that takes the function and prints
// the human readable error messagea
#include "nvmewrappers.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern "C" {
void print_nvme_error(const int ret) {
  fprintf(stderr, "NVMe error: %s\n", nvme_status_to_string(ret, false));
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
    print_nvme_error(ret);
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
    print_nvme_error(ret);
#ifdef EARLY_EXIT
    exit(ret);
#endif
  }

  return ret;
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
    print_nvme_error(ret);
#ifdef EARLY_EXIT
    exit(ret);
#endif
  }
  return ret;
}
}
