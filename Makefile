# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/user/2023-msc-stosys-skeleton-af65a8c

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/2023-msc-stosys-skeleton-af65a8c

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target install/strip
install/strip: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing the project stripped..."
	/usr/bin/cmake -DCMAKE_INSTALL_DO_STRIP=1 -P cmake_install.cmake
.PHONY : install/strip

# Special rule for the target install/strip
install/strip/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing the project stripped..."
	/usr/bin/cmake -DCMAKE_INSTALL_DO_STRIP=1 -P cmake_install.cmake
.PHONY : install/strip/fast

# Special rule for the target install
install: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Install the project..."
	/usr/bin/cmake -P cmake_install.cmake
.PHONY : install

# Special rule for the target install
install/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Install the project..."
	/usr/bin/cmake -P cmake_install.cmake
.PHONY : install/fast

# Special rule for the target list_install_components
list_install_components:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Available install components are: \"Unspecified\""
.PHONY : list_install_components

# Special rule for the target list_install_components
list_install_components/fast: list_install_components

.PHONY : list_install_components/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# Special rule for the target install/local
install/local: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing only the local directory..."
	/usr/bin/cmake -DCMAKE_INSTALL_LOCAL_ONLY=1 -P cmake_install.cmake
.PHONY : install/local

# Special rule for the target install/local
install/local/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing only the local directory..."
	/usr/bin/cmake -DCMAKE_INSTALL_LOCAL_ONLY=1 -P cmake_install.cmake
.PHONY : install/local/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/user/2023-msc-stosys-skeleton-af65a8c/CMakeFiles /home/user/2023-msc-stosys-skeleton-af65a8c/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/user/2023-msc-stosys-skeleton-af65a8c/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named m3

# Build rule for target.
m3: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 m3
.PHONY : m3

# fast build rule for target.
m3/fast:
	$(MAKE) -f CMakeFiles/m3.dir/build.make CMakeFiles/m3.dir/build
.PHONY : m3/fast

#=============================================================================
# Target rules for targets named m2

# Build rule for target.
m2: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 m2
.PHONY : m2

# fast build rule for target.
m2/fast:
	$(MAKE) -f CMakeFiles/m2.dir/build.make CMakeFiles/m2.dir/build
.PHONY : m2/fast

#=============================================================================
# Target rules for targets named stosys

# Build rule for target.
stosys: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 stosys
.PHONY : stosys

# fast build rule for target.
stosys/fast:
	$(MAKE) -f CMakeFiles/stosys.dir/build.make CMakeFiles/stosys.dir/build
.PHONY : stosys/fast

#=============================================================================
# Target rules for targets named m1

# Build rule for target.
m1: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 m1
.PHONY : m1

# fast build rule for target.
m1/fast:
	$(MAKE) -f CMakeFiles/m1.dir/build.make CMakeFiles/m1.dir/build
.PHONY : m1/fast

src/common/nvmeprint.o: src/common/nvmeprint.cpp.o

.PHONY : src/common/nvmeprint.o

# target to build an object file
src/common/nvmeprint.cpp.o:
	$(MAKE) -f CMakeFiles/stosys.dir/build.make CMakeFiles/stosys.dir/src/common/nvmeprint.cpp.o
	$(MAKE) -f CMakeFiles/m1.dir/build.make CMakeFiles/m1.dir/src/common/nvmeprint.cpp.o
.PHONY : src/common/nvmeprint.cpp.o

src/common/nvmeprint.i: src/common/nvmeprint.cpp.i

.PHONY : src/common/nvmeprint.i

# target to preprocess a source file
src/common/nvmeprint.cpp.i:
	$(MAKE) -f CMakeFiles/stosys.dir/build.make CMakeFiles/stosys.dir/src/common/nvmeprint.cpp.i
	$(MAKE) -f CMakeFiles/m1.dir/build.make CMakeFiles/m1.dir/src/common/nvmeprint.cpp.i
.PHONY : src/common/nvmeprint.cpp.i

src/common/nvmeprint.s: src/common/nvmeprint.cpp.s

.PHONY : src/common/nvmeprint.s

# target to generate assembly for a file
src/common/nvmeprint.cpp.s:
	$(MAKE) -f CMakeFiles/stosys.dir/build.make CMakeFiles/stosys.dir/src/common/nvmeprint.cpp.s
	$(MAKE) -f CMakeFiles/m1.dir/build.make CMakeFiles/m1.dir/src/common/nvmeprint.cpp.s
.PHONY : src/common/nvmeprint.cpp.s

src/common/utils.o: src/common/utils.cpp.o

.PHONY : src/common/utils.o

# target to build an object file
src/common/utils.cpp.o:
	$(MAKE) -f CMakeFiles/stosys.dir/build.make CMakeFiles/stosys.dir/src/common/utils.cpp.o
	$(MAKE) -f CMakeFiles/m1.dir/build.make CMakeFiles/m1.dir/src/common/utils.cpp.o
.PHONY : src/common/utils.cpp.o

src/common/utils.i: src/common/utils.cpp.i

.PHONY : src/common/utils.i

# target to preprocess a source file
src/common/utils.cpp.i:
	$(MAKE) -f CMakeFiles/stosys.dir/build.make CMakeFiles/stosys.dir/src/common/utils.cpp.i
	$(MAKE) -f CMakeFiles/m1.dir/build.make CMakeFiles/m1.dir/src/common/utils.cpp.i
.PHONY : src/common/utils.cpp.i

src/common/utils.s: src/common/utils.cpp.s

.PHONY : src/common/utils.s

# target to generate assembly for a file
src/common/utils.cpp.s:
	$(MAKE) -f CMakeFiles/stosys.dir/build.make CMakeFiles/stosys.dir/src/common/utils.cpp.s
	$(MAKE) -f CMakeFiles/m1.dir/build.make CMakeFiles/m1.dir/src/common/utils.cpp.s
.PHONY : src/common/utils.cpp.s

src/m1/device.o: src/m1/device.cpp.o

.PHONY : src/m1/device.o

# target to build an object file
src/m1/device.cpp.o:
	$(MAKE) -f CMakeFiles/m1.dir/build.make CMakeFiles/m1.dir/src/m1/device.cpp.o
.PHONY : src/m1/device.cpp.o

src/m1/device.i: src/m1/device.cpp.i

.PHONY : src/m1/device.i

# target to preprocess a source file
src/m1/device.cpp.i:
	$(MAKE) -f CMakeFiles/m1.dir/build.make CMakeFiles/m1.dir/src/m1/device.cpp.i
.PHONY : src/m1/device.cpp.i

src/m1/device.s: src/m1/device.cpp.s

.PHONY : src/m1/device.s

# target to generate assembly for a file
src/m1/device.cpp.s:
	$(MAKE) -f CMakeFiles/m1.dir/build.make CMakeFiles/m1.dir/src/m1/device.cpp.s
.PHONY : src/m1/device.cpp.s

src/m1/m1.o: src/m1/m1.cpp.o

.PHONY : src/m1/m1.o

# target to build an object file
src/m1/m1.cpp.o:
	$(MAKE) -f CMakeFiles/m1.dir/build.make CMakeFiles/m1.dir/src/m1/m1.cpp.o
.PHONY : src/m1/m1.cpp.o

src/m1/m1.i: src/m1/m1.cpp.i

.PHONY : src/m1/m1.i

# target to preprocess a source file
src/m1/m1.cpp.i:
	$(MAKE) -f CMakeFiles/m1.dir/build.make CMakeFiles/m1.dir/src/m1/m1.cpp.i
.PHONY : src/m1/m1.cpp.i

src/m1/m1.s: src/m1/m1.cpp.s

.PHONY : src/m1/m1.s

# target to generate assembly for a file
src/m1/m1.cpp.s:
	$(MAKE) -f CMakeFiles/m1.dir/build.make CMakeFiles/m1.dir/src/m1/m1.cpp.s
.PHONY : src/m1/m1.cpp.s

src/m1/m1_assignment.o: src/m1/m1_assignment.cpp.o

.PHONY : src/m1/m1_assignment.o

# target to build an object file
src/m1/m1_assignment.cpp.o:
	$(MAKE) -f CMakeFiles/m1.dir/build.make CMakeFiles/m1.dir/src/m1/m1_assignment.cpp.o
.PHONY : src/m1/m1_assignment.cpp.o

src/m1/m1_assignment.i: src/m1/m1_assignment.cpp.i

.PHONY : src/m1/m1_assignment.i

# target to preprocess a source file
src/m1/m1_assignment.cpp.i:
	$(MAKE) -f CMakeFiles/m1.dir/build.make CMakeFiles/m1.dir/src/m1/m1_assignment.cpp.i
.PHONY : src/m1/m1_assignment.cpp.i

src/m1/m1_assignment.s: src/m1/m1_assignment.cpp.s

.PHONY : src/m1/m1_assignment.s

# target to generate assembly for a file
src/m1/m1_assignment.cpp.s:
	$(MAKE) -f CMakeFiles/m1.dir/build.make CMakeFiles/m1.dir/src/m1/m1_assignment.cpp.s
.PHONY : src/m1/m1_assignment.cpp.s

src/m23-ftl/backup_zns_device_file.o: src/m23-ftl/backup_zns_device_file.cpp.o

.PHONY : src/m23-ftl/backup_zns_device_file.o

# target to build an object file
src/m23-ftl/backup_zns_device_file.cpp.o:
	$(MAKE) -f CMakeFiles/stosys.dir/build.make CMakeFiles/stosys.dir/src/m23-ftl/backup_zns_device_file.cpp.o
.PHONY : src/m23-ftl/backup_zns_device_file.cpp.o

src/m23-ftl/backup_zns_device_file.i: src/m23-ftl/backup_zns_device_file.cpp.i

.PHONY : src/m23-ftl/backup_zns_device_file.i

# target to preprocess a source file
src/m23-ftl/backup_zns_device_file.cpp.i:
	$(MAKE) -f CMakeFiles/stosys.dir/build.make CMakeFiles/stosys.dir/src/m23-ftl/backup_zns_device_file.cpp.i
.PHONY : src/m23-ftl/backup_zns_device_file.cpp.i

src/m23-ftl/backup_zns_device_file.s: src/m23-ftl/backup_zns_device_file.cpp.s

.PHONY : src/m23-ftl/backup_zns_device_file.s

# target to generate assembly for a file
src/m23-ftl/backup_zns_device_file.cpp.s:
	$(MAKE) -f CMakeFiles/stosys.dir/build.make CMakeFiles/stosys.dir/src/m23-ftl/backup_zns_device_file.cpp.s
.PHONY : src/m23-ftl/backup_zns_device_file.cpp.s

src/m23-ftl/m2.o: src/m23-ftl/m2.cpp.o

.PHONY : src/m23-ftl/m2.o

# target to build an object file
src/m23-ftl/m2.cpp.o:
	$(MAKE) -f CMakeFiles/m2.dir/build.make CMakeFiles/m2.dir/src/m23-ftl/m2.cpp.o
.PHONY : src/m23-ftl/m2.cpp.o

src/m23-ftl/m2.i: src/m23-ftl/m2.cpp.i

.PHONY : src/m23-ftl/m2.i

# target to preprocess a source file
src/m23-ftl/m2.cpp.i:
	$(MAKE) -f CMakeFiles/m2.dir/build.make CMakeFiles/m2.dir/src/m23-ftl/m2.cpp.i
.PHONY : src/m23-ftl/m2.cpp.i

src/m23-ftl/m2.s: src/m23-ftl/m2.cpp.s

.PHONY : src/m23-ftl/m2.s

# target to generate assembly for a file
src/m23-ftl/m2.cpp.s:
	$(MAKE) -f CMakeFiles/m2.dir/build.make CMakeFiles/m2.dir/src/m23-ftl/m2.cpp.s
.PHONY : src/m23-ftl/m2.cpp.s

src/m23-ftl/m3.o: src/m23-ftl/m3.cpp.o

.PHONY : src/m23-ftl/m3.o

# target to build an object file
src/m23-ftl/m3.cpp.o:
	$(MAKE) -f CMakeFiles/m3.dir/build.make CMakeFiles/m3.dir/src/m23-ftl/m3.cpp.o
.PHONY : src/m23-ftl/m3.cpp.o

src/m23-ftl/m3.i: src/m23-ftl/m3.cpp.i

.PHONY : src/m23-ftl/m3.i

# target to preprocess a source file
src/m23-ftl/m3.cpp.i:
	$(MAKE) -f CMakeFiles/m3.dir/build.make CMakeFiles/m3.dir/src/m23-ftl/m3.cpp.i
.PHONY : src/m23-ftl/m3.cpp.i

src/m23-ftl/m3.s: src/m23-ftl/m3.cpp.s

.PHONY : src/m23-ftl/m3.s

# target to generate assembly for a file
src/m23-ftl/m3.cpp.s:
	$(MAKE) -f CMakeFiles/m3.dir/build.make CMakeFiles/m3.dir/src/m23-ftl/m3.cpp.s
.PHONY : src/m23-ftl/m3.cpp.s

src/m23-ftl/zns_device.o: src/m23-ftl/zns_device.cpp.o

.PHONY : src/m23-ftl/zns_device.o

# target to build an object file
src/m23-ftl/zns_device.cpp.o:
	$(MAKE) -f CMakeFiles/stosys.dir/build.make CMakeFiles/stosys.dir/src/m23-ftl/zns_device.cpp.o
.PHONY : src/m23-ftl/zns_device.cpp.o

src/m23-ftl/zns_device.i: src/m23-ftl/zns_device.cpp.i

.PHONY : src/m23-ftl/zns_device.i

# target to preprocess a source file
src/m23-ftl/zns_device.cpp.i:
	$(MAKE) -f CMakeFiles/stosys.dir/build.make CMakeFiles/stosys.dir/src/m23-ftl/zns_device.cpp.i
.PHONY : src/m23-ftl/zns_device.cpp.i

src/m23-ftl/zns_device.s: src/m23-ftl/zns_device.cpp.s

.PHONY : src/m23-ftl/zns_device.s

# target to generate assembly for a file
src/m23-ftl/zns_device.cpp.s:
	$(MAKE) -f CMakeFiles/stosys.dir/build.make CMakeFiles/stosys.dir/src/m23-ftl/zns_device.cpp.s
.PHONY : src/m23-ftl/zns_device.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... install/strip"
	@echo "... install"
	@echo "... list_install_components"
	@echo "... rebuild_cache"
	@echo "... edit_cache"
	@echo "... m3"
	@echo "... install/local"
	@echo "... m2"
	@echo "... stosys"
	@echo "... m1"
	@echo "... src/common/nvmeprint.o"
	@echo "... src/common/nvmeprint.i"
	@echo "... src/common/nvmeprint.s"
	@echo "... src/common/utils.o"
	@echo "... src/common/utils.i"
	@echo "... src/common/utils.s"
	@echo "... src/m1/device.o"
	@echo "... src/m1/device.i"
	@echo "... src/m1/device.s"
	@echo "... src/m1/m1.o"
	@echo "... src/m1/m1.i"
	@echo "... src/m1/m1.s"
	@echo "... src/m1/m1_assignment.o"
	@echo "... src/m1/m1_assignment.i"
	@echo "... src/m1/m1_assignment.s"
	@echo "... src/m23-ftl/backup_zns_device_file.o"
	@echo "... src/m23-ftl/backup_zns_device_file.i"
	@echo "... src/m23-ftl/backup_zns_device_file.s"
	@echo "... src/m23-ftl/m2.o"
	@echo "... src/m23-ftl/m2.i"
	@echo "... src/m23-ftl/m2.s"
	@echo "... src/m23-ftl/m3.o"
	@echo "... src/m23-ftl/m3.i"
	@echo "... src/m23-ftl/m3.s"
	@echo "... src/m23-ftl/zns_device.o"
	@echo "... src/m23-ftl/zns_device.i"
	@echo "... src/m23-ftl/zns_device.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

