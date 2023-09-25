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

/**
 * Args:
 *   fd: file descriptor of the NVMe device
 *   nsid: namespace id for our current namespace
 *   sdlba: LBA of the destination
 *   lr: limited rewrite type
 *   fua: if set 1 the controller will write data before command completion
 *   prinfow: protection information for the writing
 *   prinfor: protection information for reading
 *   dtype: directive type for writing
 *   format: format of the source range entries
 *   dspec: directive speicfic value for the type field
 *   ilbr: Initial logical block reference tag
 *   lbatm: Application tag mask for the write
 *   lbat: Application tag for write
 *   copy: a struct containing the copy range.
 *   nr: number of ranges in the command (0 based)
 */
// TODO: fix this shit
int ss_nvme_copy(int fd, uint32_t nsid, struct nvme_copy_range *copy,
                 uint64_t sdlba, uint16_t nr) {
  printf("nlb and nr: %d and %d\n", copy->nlb, nr);
  int ret = nvme_copy(fd, cpu_to_le32(nsid), copy, cpu_to_le64(sdlba),
                      cpu_to_le16(nr), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  if (ret == -1) {
    perror("ss_nvme_copy() failed");
  } else if (ret != 0) {
    print_nvme_error("copy", ret);
#ifdef EARLY_EXIT
    exit(ret);
#endif
  }

  return ret;
}

struct nvme_copy_range ss_nvme_create_range(const uint64_t slba,
                                            const uint16_t nlb) {
  return {// .rsvd0 = {0, 0, 0, 0, 0, 0, 0},
          .slba = cpu_to_le64(slba),
          .nlb = cpu_to_le16(nlb),
          // .rsvd18 = {0, 0, 0, 0, 0},
          .eilbrt = cpu_to_le32(0),
          .elbatm = cpu_to_le16(0),
          .elbat = cpu_to_le16(0)};
}
}
