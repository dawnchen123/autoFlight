# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/dji/autoFlight/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dji/autoFlight/build

# Include any dependencies generated for this target.
include dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/depend.make

# Include the progress variables for this target.
include dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/progress.make

# Include the compile flags for this target's objects.
include dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/flags.make

dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.o: dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/flags.make
dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.o: /home/dji/autoFlight/src/dji_sdk_demo/src/demo_local_position_control.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dji/autoFlight/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.o"
	cd /home/dji/autoFlight/build/dji_sdk_demo && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.o -c /home/dji/autoFlight/src/dji_sdk_demo/src/demo_local_position_control.cpp

dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.i"
	cd /home/dji/autoFlight/build/dji_sdk_demo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dji/autoFlight/src/dji_sdk_demo/src/demo_local_position_control.cpp > CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.i

dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.s"
	cd /home/dji/autoFlight/build/dji_sdk_demo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dji/autoFlight/src/dji_sdk_demo/src/demo_local_position_control.cpp -o CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.s

dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.o.requires:

.PHONY : dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.o.requires

dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.o.provides: dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.o.requires
	$(MAKE) -f dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/build.make dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.o.provides.build
.PHONY : dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.o.provides

dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.o.provides.build: dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.o


# Object files for target demo_local_position_control
demo_local_position_control_OBJECTS = \
"CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.o"

# External object files for target demo_local_position_control
demo_local_position_control_EXTERNAL_OBJECTS =

/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.o
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/build.make
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /opt/ros/kinetic/lib/libcv_bridge.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /opt/ros/kinetic/lib/aarch64-linux-gnu/libopencv_core3.so.3.3.1
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /opt/ros/kinetic/lib/aarch64-linux-gnu/libopencv_imgproc3.so.3.3.1
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /opt/ros/kinetic/lib/aarch64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /opt/ros/kinetic/lib/libimage_transport.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /opt/ros/kinetic/lib/libmessage_filters.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /opt/ros/kinetic/lib/libclass_loader.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /usr/lib/libPocoFoundation.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /usr/lib/aarch64-linux-gnu/libdl.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /opt/ros/kinetic/lib/libroscpp.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /usr/lib/aarch64-linux-gnu/libboost_signals.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /opt/ros/kinetic/lib/librosconsole.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /opt/ros/kinetic/lib/libroslib.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /opt/ros/kinetic/lib/librospack.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /usr/lib/aarch64-linux-gnu/libtinyxml.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /opt/ros/kinetic/lib/librostime.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /opt/ros/kinetic/lib/libcpp_common.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: /usr/local/lib/libdjiosdk-core.a
/home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control: dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dji/autoFlight/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control"
	cd /home/dji/autoFlight/build/dji_sdk_demo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/demo_local_position_control.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/build: /home/dji/autoFlight/devel/lib/dji_sdk_demo/demo_local_position_control

.PHONY : dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/build

dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/requires: dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/src/demo_local_position_control.cpp.o.requires

.PHONY : dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/requires

dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/clean:
	cd /home/dji/autoFlight/build/dji_sdk_demo && $(CMAKE_COMMAND) -P CMakeFiles/demo_local_position_control.dir/cmake_clean.cmake
.PHONY : dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/clean

dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/depend:
	cd /home/dji/autoFlight/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dji/autoFlight/src /home/dji/autoFlight/src/dji_sdk_demo /home/dji/autoFlight/build /home/dji/autoFlight/build/dji_sdk_demo /home/dji/autoFlight/build/dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : dji_sdk_demo/CMakeFiles/demo_local_position_control.dir/depend

