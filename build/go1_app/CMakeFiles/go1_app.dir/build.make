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
include go1_app/CMakeFiles/go1_app.dir/depend.make

# Include the progress variables for this target.
include go1_app/CMakeFiles/go1_app.dir/progress.make

# Include the compile flags for this target's objects.
include go1_app/CMakeFiles/go1_app.dir/flags.make

go1_app/CMakeFiles/go1_app.dir/src/main.cpp.o: go1_app/CMakeFiles/go1_app.dir/flags.make
go1_app/CMakeFiles/go1_app.dir/src/main.cpp.o: /home/li/ljyws/quadruped-sim/src/go1_app/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/li/ljyws/quadruped-sim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object go1_app/CMakeFiles/go1_app.dir/src/main.cpp.o"
	cd /home/li/ljyws/quadruped-sim/build/go1_app && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/go1_app.dir/src/main.cpp.o -c /home/li/ljyws/quadruped-sim/src/go1_app/src/main.cpp

go1_app/CMakeFiles/go1_app.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/go1_app.dir/src/main.cpp.i"
	cd /home/li/ljyws/quadruped-sim/build/go1_app && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/li/ljyws/quadruped-sim/src/go1_app/src/main.cpp > CMakeFiles/go1_app.dir/src/main.cpp.i

go1_app/CMakeFiles/go1_app.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/go1_app.dir/src/main.cpp.s"
	cd /home/li/ljyws/quadruped-sim/build/go1_app && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/li/ljyws/quadruped-sim/src/go1_app/src/main.cpp -o CMakeFiles/go1_app.dir/src/main.cpp.s

go1_app/CMakeFiles/go1_app.dir/src/main.cpp.o.requires:

.PHONY : go1_app/CMakeFiles/go1_app.dir/src/main.cpp.o.requires

go1_app/CMakeFiles/go1_app.dir/src/main.cpp.o.provides: go1_app/CMakeFiles/go1_app.dir/src/main.cpp.o.requires
	$(MAKE) -f go1_app/CMakeFiles/go1_app.dir/build.make go1_app/CMakeFiles/go1_app.dir/src/main.cpp.o.provides.build
.PHONY : go1_app/CMakeFiles/go1_app.dir/src/main.cpp.o.provides

go1_app/CMakeFiles/go1_app.dir/src/main.cpp.o.provides.build: go1_app/CMakeFiles/go1_app.dir/src/main.cpp.o


# Object files for target go1_app
go1_app_OBJECTS = \
"CMakeFiles/go1_app.dir/src/main.cpp.o"

# External object files for target go1_app
go1_app_EXTERNAL_OBJECTS =

/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: go1_app/CMakeFiles/go1_app.dir/src/main.cpp.o
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: go1_app/CMakeFiles/go1_app.dir/build.make
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /home/li/ljyws/quadruped-sim/devel/lib/libgo1_lib.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /opt/ros/melodic/lib/libroscpp.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /opt/ros/melodic/lib/librosconsole.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /opt/ros/melodic/lib/librostime.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /opt/ros/melodic/lib/libcpp_common.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /usr/local/lib/libOsqpEigen.so.0.8.1
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: /usr/local/lib/libosqp.so
/home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app: go1_app/CMakeFiles/go1_app.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/li/ljyws/quadruped-sim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app"
	cd /home/li/ljyws/quadruped-sim/build/go1_app && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/go1_app.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
go1_app/CMakeFiles/go1_app.dir/build: /home/li/ljyws/quadruped-sim/devel/lib/go1_app/go1_app

.PHONY : go1_app/CMakeFiles/go1_app.dir/build

go1_app/CMakeFiles/go1_app.dir/requires: go1_app/CMakeFiles/go1_app.dir/src/main.cpp.o.requires

.PHONY : go1_app/CMakeFiles/go1_app.dir/requires

go1_app/CMakeFiles/go1_app.dir/clean:
	cd /home/li/ljyws/quadruped-sim/build/go1_app && $(CMAKE_COMMAND) -P CMakeFiles/go1_app.dir/cmake_clean.cmake
.PHONY : go1_app/CMakeFiles/go1_app.dir/clean

go1_app/CMakeFiles/go1_app.dir/depend:
	cd /home/li/ljyws/quadruped-sim/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/li/ljyws/quadruped-sim/src /home/li/ljyws/quadruped-sim/src/go1_app /home/li/ljyws/quadruped-sim/build /home/li/ljyws/quadruped-sim/build/go1_app /home/li/ljyws/quadruped-sim/build/go1_app/CMakeFiles/go1_app.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : go1_app/CMakeFiles/go1_app.dir/depend

