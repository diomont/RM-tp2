# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/diogo/Desktop/RM/assign2/ciberRatoTools

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/diogo/Desktop/RM/assign2/ciberRatoTools/build

# Utility rule file for simulator_autogen.

# Include the progress variables for this target.
include simulator/CMakeFiles/simulator_autogen.dir/progress.make

simulator/CMakeFiles/simulator_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/diogo/Desktop/RM/assign2/ciberRatoTools/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC and UIC for target simulator"
	cd /home/diogo/Desktop/RM/assign2/ciberRatoTools/build/simulator && /usr/bin/cmake -E cmake_autogen /home/diogo/Desktop/RM/assign2/ciberRatoTools/build/simulator/CMakeFiles/simulator_autogen.dir/AutogenInfo.json Release

simulator_autogen: simulator/CMakeFiles/simulator_autogen
simulator_autogen: simulator/CMakeFiles/simulator_autogen.dir/build.make

.PHONY : simulator_autogen

# Rule to build all files generated by this target.
simulator/CMakeFiles/simulator_autogen.dir/build: simulator_autogen

.PHONY : simulator/CMakeFiles/simulator_autogen.dir/build

simulator/CMakeFiles/simulator_autogen.dir/clean:
	cd /home/diogo/Desktop/RM/assign2/ciberRatoTools/build/simulator && $(CMAKE_COMMAND) -P CMakeFiles/simulator_autogen.dir/cmake_clean.cmake
.PHONY : simulator/CMakeFiles/simulator_autogen.dir/clean

simulator/CMakeFiles/simulator_autogen.dir/depend:
	cd /home/diogo/Desktop/RM/assign2/ciberRatoTools/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/diogo/Desktop/RM/assign2/ciberRatoTools /home/diogo/Desktop/RM/assign2/ciberRatoTools/simulator /home/diogo/Desktop/RM/assign2/ciberRatoTools/build /home/diogo/Desktop/RM/assign2/ciberRatoTools/build/simulator /home/diogo/Desktop/RM/assign2/ciberRatoTools/build/simulator/CMakeFiles/simulator_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : simulator/CMakeFiles/simulator_autogen.dir/depend
