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

# Utility rule file for dji_sdk_genlisp.

# Include the progress variables for this target.
include dji_sdk/CMakeFiles/dji_sdk_genlisp.dir/progress.make

dji_sdk_genlisp: dji_sdk/CMakeFiles/dji_sdk_genlisp.dir/build.make

.PHONY : dji_sdk_genlisp

# Rule to build all files generated by this target.
dji_sdk/CMakeFiles/dji_sdk_genlisp.dir/build: dji_sdk_genlisp

.PHONY : dji_sdk/CMakeFiles/dji_sdk_genlisp.dir/build

dji_sdk/CMakeFiles/dji_sdk_genlisp.dir/clean:
	cd /home/dji/autoFlight/build/dji_sdk && $(CMAKE_COMMAND) -P CMakeFiles/dji_sdk_genlisp.dir/cmake_clean.cmake
.PHONY : dji_sdk/CMakeFiles/dji_sdk_genlisp.dir/clean

dji_sdk/CMakeFiles/dji_sdk_genlisp.dir/depend:
	cd /home/dji/autoFlight/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dji/autoFlight/src /home/dji/autoFlight/src/dji_sdk /home/dji/autoFlight/build /home/dji/autoFlight/build/dji_sdk /home/dji/autoFlight/build/dji_sdk/CMakeFiles/dji_sdk_genlisp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : dji_sdk/CMakeFiles/dji_sdk_genlisp.dir/depend

