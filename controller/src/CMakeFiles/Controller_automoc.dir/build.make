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
CMAKE_SOURCE_DIR = /home/jorge/robocomp/components/afuentesjmontero/controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jorge/robocomp/components/afuentesjmontero/controller

# Utility rule file for Controller_automoc.

# Include the progress variables for this target.
include src/CMakeFiles/Controller_automoc.dir/progress.make

src/CMakeFiles/Controller_automoc:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/jorge/robocomp/components/afuentesjmontero/controller/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic moc for target Controller"
	cd /home/jorge/robocomp/components/afuentesjmontero/controller/src && /usr/bin/cmake -E cmake_autogen /home/jorge/robocomp/components/afuentesjmontero/controller/src/CMakeFiles/Controller_automoc.dir/ ""

Controller_automoc: src/CMakeFiles/Controller_automoc
Controller_automoc: src/CMakeFiles/Controller_automoc.dir/build.make

.PHONY : Controller_automoc

# Rule to build all files generated by this target.
src/CMakeFiles/Controller_automoc.dir/build: Controller_automoc

.PHONY : src/CMakeFiles/Controller_automoc.dir/build

src/CMakeFiles/Controller_automoc.dir/clean:
	cd /home/jorge/robocomp/components/afuentesjmontero/controller/src && $(CMAKE_COMMAND) -P CMakeFiles/Controller_automoc.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/Controller_automoc.dir/clean

src/CMakeFiles/Controller_automoc.dir/depend:
	cd /home/jorge/robocomp/components/afuentesjmontero/controller && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jorge/robocomp/components/afuentesjmontero/controller /home/jorge/robocomp/components/afuentesjmontero/controller/src /home/jorge/robocomp/components/afuentesjmontero/controller /home/jorge/robocomp/components/afuentesjmontero/controller/src /home/jorge/robocomp/components/afuentesjmontero/controller/src/CMakeFiles/Controller_automoc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/Controller_automoc.dir/depend

