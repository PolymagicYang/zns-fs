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
#pragma once

#include <thread>

#include "ftl.hpp"
#include "znsblock.hpp"
#include "zone.hpp"

class Calliope {
 public:
  FTL *ftl;

  /** Select the zone that we want to free form memory */
  int select_zone();

/** Main method of the GC */
  void reap();
  
  /** Initialize the thread with the GC*/  
  void initialize();
  
  Calliope(FTL *ftl, const uint16_t threshold);

 private:
  // Number of regions we ought to keep clean
  uint16_t threshold;

  // Our thread
  std::thread thread;
  
  // Flag to see if we found something useful in our sweep could be
  // replaced using a NULL value. 
  bool can_reap;
};

#endif
