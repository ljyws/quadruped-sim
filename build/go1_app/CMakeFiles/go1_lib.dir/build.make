# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/li/ljyws/quadruped-sim/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/li/ljyws/quadruped-sim/build

# Include any dependencies generated for this target.
include go1_app/CMakeFiles/go1_lib.dir/depend.make

# Include the progress variables for this target.
include go1_app/CMakeFiles/go1_lib.dir/progress.make

# Include the compile flags for this target's objects.
include go1_app/CMakeFiles/go1_lib.dir/flags.make

go1_app/CMakeFiles/go1_lib.dir/src/RosInterface.cpp.o: go1_app/CMakeFiles/go1_lib.dir/flags.make
go1_app/CMakeFiles/go1_lib.dir/src/RosInterface.cpp.o: /home/li/ljyws/quadruped-sim/src/go1_app/src/RosInterface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/li/ljyws/quadruped-sim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object go1_app/CMakeFiles/go1_lib.dir/src/RosInterface.cpp.o"
	cd /home/li/ljyws/quadruped-sim/build/go1_app && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/go1_lib.dir/src/RosInterface.cpp.o -c /home/li/ljyws/quadruped-sim/src/go1_app/src/RosInterface.cpp

go1_app/CMakeFiles/go1_lib.dir/src/RosInterface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/go1_lib.dir/src/RosInterface.cpp.i"
	cd /home/li/ljyws/quadruped-sim/build/go1_app && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/li/ljyws/quadruped-sim/src/go1_app/src/RosInterface.cpp > CMakeFiles/go1_lib.dir/src/RosInterface.cpp.i

go1_app/CMakeFiles/go1_lib.dir/src/RosInterface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/go1_lib.dir/src/RosInterface.cpp.s"
	cd /home/li/ljyws/quadruped-sim/build/go1_app && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/li/ljyws/quadruped-sim/src/go1_app/src/RosInterface.cpp -o CMakeFiles/go1_lib.dir/src/RosInterface.cpp.s

go1_app/CMakeFiles/go1_lib.dir/src/RosInterface.cpp.o.requires:

.PHONY : go1_app/CMakeFiles/go1_lib.dir/src/RosInterface.cpp.o.requires

go1_app/CMakeFiles/go1_lib.dir/src/RosInterface.cpp.o.provides: go1_app/CMakeFiles/go1_lib.dir/src/RosInterface.cpp.o.requires
	$(MAKE) -f go1_app/CMakeFiles/go1_lib.dir/build.make go1_app/CMakeFiles/go1_lib.dir/src/RosInterface.cpp.o.provides.build
.PHONY : go1_app/CMakeFiles/go1_lib.dir/src/RosInterface.cpp.o.provides

go1_app/CMakeFiles/go1_lib.dir/src/RosInterface.cpp.o.provides.build: go1_app/CMakeFiles/go1_lib.dir/src/RosInterface.cpp.o


# Object files for target go1_lib
go1_lib_OBJECTS = \
"CMakeFiles/go1_lib.dir/src/RosInterface.cpp.o"

# External object files for target go1_lib
go1_lib_EXTERNAL_OBJECTS =

/home/li/ljyws/quadruped-sim/devel/lib/libgo1_lib.so: go1_app/CMakeFiles/go1_lib.dir/src/RosInterface.cpp.o
/home/li/ljyws/quadruped-sim/devel/lib/libgo1_lib.so: go1_app/CMakeFiles/go1_lib.dir/build.make
/home/li/ljyws/quadruped-sim/devel/lib/libgo1_lib.so: go1_app/CMakeFiles/go1_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/li/ljyws/quadruped-sim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/li/ljyws/quadruped-sim/devel/lib/libgo1_lib.so"
	cd /home/li/ljyws/quadruped-sim/build/go1_app && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/go1_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
go1_app/CMakeFiles/go1_lib.dir/build: /home/li/ljyws/quadruped-sim/devel/lib/libgo1_lib.so

.PHONY : go1_app/CMakeFiles/go1_lib.dir/build

go1_app/CMakeFiles/go1_lib.dir/requires: go1_app/CMakeFiles/go1_lib.dir/src/RosInterface.cpp.o.requires

.PHONY : go1_app/CMakeFiles/go1_lib.dir/requires

go1_app/CMakeFiles/go1_lib.dir/clean:
	cd /home/li/ljyws/quadruped-sim/build/go1_app && $(CMAKE_COMMAND) -P CMakeFiles/go1_lib.dir/cmake_clean.cmake
.PHONY : go1_app/CMakeFiles/go1_lib.dir/clean

go1_app/CMakeFiles/go1_lib.dir/depend:
	cd /home/li/ljyws/quadruped-sim/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/li/ljyws/quadruped-sim/src /home/li/ljyws/quadruped-sim/src/go1_app /home/li/ljyws/quadruped-sim/build /home/li/ljyws/quadruped-sim/build/go1_app /home/li/ljyws/quadruped-sim/build/go1_app/CMakeFiles/go1_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : go1_app/CMakeFiles/go1_lib.dir/depend
