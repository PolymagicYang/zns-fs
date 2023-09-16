#/*
 # * MIT License
 #Copyright (c) 2021 - current
 #Authors:  Animesh Trivedi, Valentijn Dymphnus van de Beek & Zhiyang Wang
 #This code is part of the Storage System Course at VU Amsterdam
 #Permission is hereby granted, free of charge, to any person obtaining a copy
 #of this software and associated documentation files (the "Software"), to deal
 #in the Software without restriction, including without limitation the rights
 #to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 #copies of the Software, and to permit persons to whom the Software is
 #furnished to do so, subject to the following conditions:
 #The above copyright notice and this permission notice shall be included in all
 #copies or substantial portions of the Software.
 #THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 #IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 #FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 #AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 #LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 #OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 #SOFTWARE.
 # */

s2fs_SOURCES = rocks_s2fs.cc
#s2fs_HEADERS = rocks_s2fs.h
# we need the name of the function that will become available later

# from the gcc man page
#       -u symbol
#           Pretend the symbol symbol is undefined, to force linking of
#           library modules to define it.  You can use -u multiple times
#           with different symbols to force loading of additional library
#           modules.
s2fs_LDFLAGS = -lzbd -u stosys_s2fs_reg
