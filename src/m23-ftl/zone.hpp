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

#ifndef STOSYS_PROJECT_ZNS_ZONE_H
#define STOSYS_PROJECT_ZNS_ZONE_H
#pragma once

#include <libnvme.h>
#include <pthread.h>

#include <cstdint>
#include <iostream>
#include <map>
#include <mutex>
#include <vector>

#include "znsblock.hpp"

using BlockMap = std::map<uint64_t, ZNSBlock>;

struct ZoneMap {
  pthread_rwlock_t lock;
  BlockMap map;
};

#define RESET_ZONE true

enum ZoneState {
  Empty = 0x0,
  Full = 0x1,
  ImplicitOpen = 0x2,
  ExplicitOpen = 0x3,
  Closed = 0x4,
  ReadOnly = 0xD,
  Offline = 0xF,
  Unknown = 16,
  Bad = 224,
};

/** Interface used for this zone.*/
enum ZoneModel {
  /** Sequential work loads and full control for the host. */
  HostManaged,
  /** Backwards compatible with block devices so that random writes
          can be issued. */
  HostAware
};

/** Zone type as defined by the ZNS standard. */
enum ZoneZNSType {
  /** Similar to block devices. Accepts random writes and does not
          have write pointers. */
  Convential,
  /** Random writes are allowed, but has a write pointer.*/
  SequentialWritePreferred,
  /** Can only be used sequentially with a write pointer. Write
          commands must be aligned with the write pointer. */
  SequentialWriteRequired
};

int get_zns_zone_info(const int fd, const int nsid, uint64_t *zcap,
                      uint64_t *nr, struct nvme_zone_report *zns_report);

const char *get_state_text(const enum ZoneState state);

const char *get_zone_model_text(const enum ZoneModel model);

const char *get_zone_zns_type(const enum ZoneZNSType type);

#endif
