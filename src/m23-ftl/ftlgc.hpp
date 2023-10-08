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
#ifndef STOSYS_PROJECT_FTLGC_H
#define STOSYS_PROJECT_FTLGC_H
#include <pthread.h>

#include <exception>
#pragma once

#include <thread>

#include "ftl.hpp"
#include "znsblock.hpp"
#include "zone.hpp"

class Calliope {
 public:
  FTL *ftl;

  Calliope(FTL *ftl, pthread_cond_t *cond, pthread_mutex_t *mutex,
           pthread_cond_t *clean_cond, pthread_mutex_t *clean_lock);

  /** Select the zone that we want to free from memory */
  int select_zone();

  bool select_log_zone(uint16_t *zone_num);

  /** Main method of the GC */
  void reap();

  /** Initialize the thread with the GC*/
  void initialize();

  ~Calliope() {
    this->terminated = true;
    this->thread.join();
  }
  bool terminated = false;
  // Our thread
  std::thread thread;

 private:
  // Number of regions we ought to keep clean
  uint16_t threshold;

  // Flag to see if we found something useful in our sweep could be
  // replaced using a NULL value.
  bool can_reap;

  // Cond used to wake up the GC.
  pthread_cond_t *need_gc;

  // Coupled with cond.
  pthread_mutex_t *need_gc_lock;

  pthread_cond_t *clean_cond;
  pthread_mutex_t *clean_lock;
};

#endif
