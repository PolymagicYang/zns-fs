#ifndef STOSYS_PROJECT_ZNS_ZONE
#define STOSYS_PROJECT_ZNS_ZONE
#include "zone.hpp"

int get_zns_zone_info(const int fd, const int nsid, uint64_t *zcap,
                      uint64_t *nr, struct nvme_zone_report *zns_report) {
  // copied from m1 to fetch zcap.
  int ret;
  ret = nvme_zns_mgmt_recv(fd, nsid, 0, NVME_ZNS_ZRA_REPORT_ZONES,
                           NVME_ZNS_ZRAS_REPORT_ALL, 0x0, 0x1000,
                           (void *)zns_report);
  if (ret != 0) {
    fprintf(stderr, "failed to report zones, ret %d \n", ret);
    return ret;
  }
  *nr = le64_to_cpu(zns_report->nr_zones);
  // memcpy(*desc, (const void *)&zns_report->entries, 0x1000);
  *zcap = (uint64_t)le64_to_cpu(zns_report->entries[1].zcap);
  return ret;
}

const char *get_state_text(const enum ZoneState state) {
  return (state == Empty)
             ? "Empty"
             : (state == Full)
                   ? "Full"
                   : (state == ImplicitOpen)
                         ? "Implicit Open"
                         : (state == ExplicitOpen)
                               ? "Explicit Open"
                               : (state == Closed)
                                     ? "Closed"
                                     : (state == ReadOnly)
                                           ? "Read Only"
                                           : (state == Offline) ? "Offline"
                                                                : "Unknown";
}

const char *get_zone_model_text(const enum ZoneModel model) {
  return (model == HostManaged)
             ? "Host Managed"
             : (model == HostAware) ? "Host Aware" : "Unknown";
}

const char *get_zone_zns_type(const enum ZoneZNSType type) {
  return (type == Convential) ? "Convential"
                              : (type == SequentialWritePreferred)
                                    ? "Sequential Write Preferred"
                                    : (type == SequentialWriteRequired)
                                          ? "Sequential Write Required"
                                          : "Unknown";
}

#endif
