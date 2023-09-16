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

#include "m1_assignment.h"

#include <assert.h>
#include <errno.h>
#include <libnvme.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "../common/unused.h"

extern "C" {
int ss_nvme_device_io_with_mdts(int fd, uint32_t nsid, uint64_t slba,
                                uint16_t numbers, void *buffer,
                                uint64_t buf_size, uint64_t lba_size,
                                uint64_t mdts_size, bool read) {
  int ret;
  if (buf_size < mdts_size) {
    if (read) {
      ret = ss_nvme_device_read(fd, nsid, slba, numbers, buffer, buf_size);
    } else {
      ret = ss_nvme_device_write(fd, nsid, slba, numbers, buffer, buf_size);
    }
    if (ret != 0) {
      return ret;
    }
    return 0;
  }

  uint64_t times = buf_size / mdts_size;  // how many times needed.
  uint64_t nums =
      mdts_size /
      lba_size;  // calculate how how many blocks need for one time I/O;

  for (uint64_t i = 0; i < times; i++) {
    if (read) {
      ret = ss_nvme_device_read(fd, nsid, slba + nums * i, nums,
                                (void *)(((uint64_t)buffer) + mdts_size * i),
                                mdts_size);
    } else {
      ret = ss_nvme_device_write(fd, nsid, slba + nums * i, nums,
                                 (void *)(((uint64_t)buffer) + mdts_size * i),
                                 mdts_size);
    }
    if (ret != 0) {
      return ret;
    }
  }

  return 0;
}

int ss_nvme_device_read(int fd, uint32_t nsid, uint64_t slba, uint16_t numbers,
                        void *buffer, uint64_t buf_size) {
  // this is to supress gcc warnings, remove it when you complete this function
  __u32 cdw10 = slba & 0xffffffff;
  __u32 cdw11 = slba >> 32;
  __u32 cdw12 = 0x7fff & (numbers - 1);

  return nvme_io_passthru(fd, nvme_cmd_read, 0, 0, nsid, 0, 0, cdw10, cdw11,
                          cdw12, 0, 0, 0, buf_size, buffer, 0, nullptr, 0,
                          NULL);
}

int ss_nvme_device_write(int fd, uint32_t nsid, uint64_t slba, uint16_t numbers,
                         void *buffer, uint64_t buf_size) {
  /*
  __u32 cdw10 = slba & 0xffffffff;
  __u32 cdw11 = slba >> 32;
  __u32 cdw12 = 0x7fff & numbers;

  struct nvme_passthru_cmd cmd = {
      .opcode = nvme_cmd_write,
      .nsid = nsid,
      .addr = (__u64) buffer,
      .data_len = buf_size,
      .cdw10 = cdw10,
      .cdw11 = cdw11,
      .cdw12 = cdw12,
  };
  return nvme_submit_io_passthru(fd, &cmd, NULL);
  */
  return nvme_write(fd, nsid, slba, numbers - 1, 0, 0, 0, 0, 0, 0, buf_size,
                    buffer, 0, 0);
}

int ss_zns_device_zone_reset(int fd, uint32_t nsid, uint64_t slba) {
  // this is to supress gcc warnings, remove it when you complete this function
  __u32 cdw10 = slba & 0xffffffff;
  __u32 cdw11 = slba >> 32;
  __u32 cdw13 =
      1 << 2;  // 08 sets to 0, 04h as the reset zone, and others reserved.
  return nvme_io_passthru(fd, nvme_zns_cmd_mgmt_send, 0, 0, nsid, 0, 0, cdw10,
                          cdw11, 0, cdw13, 0, 0, 0, nullptr, 0, nullptr, 0,
                          NULL);
}

// this does not take slba because it will return that
int ss_zns_device_zone_append(int fd, uint32_t nsid, uint64_t zslba,
                              int numbers, void *buffer, uint32_t buf_size,
                              uint64_t *written_slba) {
  /*
  //see section 4.5 how to write an append command
  __u32 cdw10 = zslba & 0xffffffff;
  __u32 cdw11 = zslba >> 32;
  // number of block size 15:00, others are reserved.
  __u32 cdw12 = 0x7fff & (numbers - 1);
  __u32 cdw14 = 0; // reversed.
  __u32 cdw15 = 0; // same as above.
  return nvme_io_passthru(fd, nvme_zns_cmd_append, 0, 0, nsid, 0, 0, cdw10,
  cdw11, cdw12, 0, cdw14, cdw15, buf_size, addr, 0, nullptr, 0, (__u64 *)
  written_slba);
  */
  int ret = nvme_zns_append(fd, nsid, zslba, numbers - 1, 0, 0, 0, 0, buf_size,
                            buffer, 0, nullptr, (__u64 *)written_slba);

  return ret;
}

void update_lba(uint64_t &write_lba, const uint32_t lba_size, const int count) {
  write_lba = write_lba + (uint64_t)count;
}

// see 5.15.2.2 Identify Controller data structure (CNS 01h)
// see how to pass any number of variables in a C/C++ program
// https://stackoverflow.com/questions/1579719/variable-number-of-parameters-in-function-in-c
// feel free to pass any relevant function parameter to this function extract
// MDTS you must return the MDTS as the return value of this function
uint64_t get_mdts_size(int count, ...) {
  UNUSED(count);
  // get CAP register firstly.
  // NVME_REG_CAP defined in type.h.

  char buffer[128];
  char result[128];
  va_list ap;
  int j;
  int fd;
  va_start(ap, count);
  for (j = 0; j < count; j++) {
    fd = va_arg(ap, int);
  }
  char str[10];
  char dest[20] = "/proc/self/fd/";
  sprintf(str, "%d", fd);
  strcat(dest, str);
  char filepath[100] = "";
  readlink(dest, filepath, 100);

  char cmd[100] = "./nvme-cli/nvme show-regs ";
  strcat(cmd, filepath);
  FILE *pipe = popen(cmd, "r");
  if (fgets(buffer, 128, pipe) != NULL) strcat(result, buffer);

  char cap[64];
  int r = 0;
  for (r = 0; result[r] != ':'; r++)
    ;
  r++;
  for (int i = 0; result[r] != '\n'; i++) {
    cap[i] = result[r];
    r++;
  }

  pclose(pipe);
  uint64_t CAP = (uint64_t)strtol(cap, NULL, 16);
  // 51:48
  //
  uint64_t MPSMIN_RAW =
      (CAP & ((uint64_t)NVME_CAP_MPSMIN_SHIFT << NVME_CAP_MPSMIN_SHIFT)) >>
      NVME_CAP_MPSMIN_SHIFT;
  uint64_t MPSMIN = pow(2, 12 + MPSMIN_RAW);

  struct nvme_id_ctrl ctrl;
  nvme_identify_ctrl(fd, &ctrl);
  uint64_t mdts = (uint64_t)ctrl.mdts - 1;
  uint64_t MDTS = pow(2, mdts) * MPSMIN;
  return MDTS;
}
}
