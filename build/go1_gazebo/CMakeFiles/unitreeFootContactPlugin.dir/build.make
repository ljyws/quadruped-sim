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
include go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/depend.make

# Include the progress variables for this target.
include go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/progress.make

# Include the compile flags for this target's objects.
include go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/flags.make

go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.o: go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/flags.make
go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.o: /home/li/ljyws/quadruped-sim/src/go1_gazebo/plugin/foot_contact_plugin.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/li/ljyws/quadruped-sim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.o"
	cd /home/li/ljyws/quadruped-sim/build/go1_gazebo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.o -c /home/li/ljyws/quadruped-sim/src/go1_gazebo/plugin/foot_contact_plugin.cc

go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.i"
	cd /home/li/ljyws/quadruped-sim/build/go1_gazebo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/li/ljyws/quadruped-sim/src/go1_gazebo/plugin/foot_contact_plugin.cc > CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.i

go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.s"
	cd /home/li/ljyws/quadruped-sim/build/go1_gazebo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/li/ljyws/quadruped-sim/src/go1_gazebo/plugin/foot_contact_plugin.cc -o CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.s

go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.o.requires:

.PHONY : go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.o.requires

go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.o.provides: go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.o.requires
	$(MAKE) -f go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/build.make go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.o.provides.build
.PHONY : go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.o.provides

go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.o.provides.build: go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.o


# Object files for target unitreeFootContactPlugin
unitreeFootContactPlugin_OBJECTS = \
"CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.o"

# External object files for target unitreeFootContactPlugin
unitreeFootContactPlugin_EXTERNAL_OBJECTS =

/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.o
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/build.make
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libcontroller_manager.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libjoint_state_controller.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/librealtime_tools.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/librobot_state_publisher_solver.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libjoint_state_listener.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libkdl_parser.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/liburdf.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libclass_loader.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/libPocoFoundation.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libdl.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/librosconsole_bridge.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libgazebo_ros_api_plugin.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libgazebo_ros_paths_plugin.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libroslib.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/librospack.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libtf.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libtf2_ros.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libactionlib.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libmessage_filters.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libroscpp.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libtf2.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/librosconsole.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/librostime.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libcpp_common.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libSimTKsimbody.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libSimTKmath.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libSimTKcommon.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libblas.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/liblapack.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libblas.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libignition-transport4.so.4.0.0
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libignition-msgs1.so.1.0.0
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libignition-common1.so.1.0.1
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libignition-fuel_tools1.so.1.0.0
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libtf.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libtf2_ros.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libactionlib.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libmessage_filters.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libroscpp.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libtf2.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/librosconsole.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/librostime.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /opt/ros/melodic/lib/libcpp_common.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libSimTKsimbody.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libSimTKmath.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libSimTKcommon.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libblas.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/liblapack.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libignition-math4.so.4.0.0
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libswscale.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libswscale.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libavformat.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libavformat.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libavutil.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: /usr/lib/x86_64-linux-gnu/libavutil.so
/home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so: go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/li/ljyws/quadruped-sim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so"
	cd /home/li/ljyws/quadruped-sim/build/go1_gazebo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/unitreeFootContactPlugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/build: /home/li/ljyws/quadruped-sim/devel/lib/libunitreeFootContactPlugin.so

.PHONY : go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/build

go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/requires: go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/plugin/foot_contact_plugin.cc.o.requires

.PHONY : go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/requires

go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/clean:
	cd /home/li/ljyws/quadruped-sim/build/go1_gazebo && $(CMAKE_COMMAND) -P CMakeFiles/unitreeFootContactPlugin.dir/cmake_clean.cmake
.PHONY : go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/clean

go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/depend:
	cd /home/li/ljyws/quadruped-sim/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/li/ljyws/quadruped-sim/src /home/li/ljyws/quadruped-sim/src/go1_gazebo /home/li/ljyws/quadruped-sim/build /home/li/ljyws/quadruped-sim/build/go1_gazebo /home/li/ljyws/quadruped-sim/build/go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : go1_gazebo/CMakeFiles/unitreeFootContactPlugin.dir/depend

