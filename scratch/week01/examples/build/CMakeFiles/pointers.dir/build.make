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
CMAKE_SOURCE_DIR = /home/ajal/git/pfms-2020a-ajalsingh/scratch/week01/examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ajal/git/pfms-2020a-ajalsingh/scratch/week01/examples/build

# Include any dependencies generated for this target.
include CMakeFiles/pointers.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pointers.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pointers.dir/flags.make

CMakeFiles/pointers.dir/pointers.cpp.o: CMakeFiles/pointers.dir/flags.make
CMakeFiles/pointers.dir/pointers.cpp.o: ../pointers.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ajal/git/pfms-2020a-ajalsingh/scratch/week01/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pointers.dir/pointers.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pointers.dir/pointers.cpp.o -c /home/ajal/git/pfms-2020a-ajalsingh/scratch/week01/examples/pointers.cpp

CMakeFiles/pointers.dir/pointers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pointers.dir/pointers.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ajal/git/pfms-2020a-ajalsingh/scratch/week01/examples/pointers.cpp > CMakeFiles/pointers.dir/pointers.cpp.i

CMakeFiles/pointers.dir/pointers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pointers.dir/pointers.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ajal/git/pfms-2020a-ajalsingh/scratch/week01/examples/pointers.cpp -o CMakeFiles/pointers.dir/pointers.cpp.s

CMakeFiles/pointers.dir/pointers.cpp.o.requires:

.PHONY : CMakeFiles/pointers.dir/pointers.cpp.o.requires

CMakeFiles/pointers.dir/pointers.cpp.o.provides: CMakeFiles/pointers.dir/pointers.cpp.o.requires
	$(MAKE) -f CMakeFiles/pointers.dir/build.make CMakeFiles/pointers.dir/pointers.cpp.o.provides.build
.PHONY : CMakeFiles/pointers.dir/pointers.cpp.o.provides

CMakeFiles/pointers.dir/pointers.cpp.o.provides.build: CMakeFiles/pointers.dir/pointers.cpp.o


# Object files for target pointers
pointers_OBJECTS = \
"CMakeFiles/pointers.dir/pointers.cpp.o"

# External object files for target pointers
pointers_EXTERNAL_OBJECTS =

pointers: CMakeFiles/pointers.dir/pointers.cpp.o
pointers: CMakeFiles/pointers.dir/build.make
pointers: CMakeFiles/pointers.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ajal/git/pfms-2020a-ajalsingh/scratch/week01/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable pointers"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pointers.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pointers.dir/build: pointers

.PHONY : CMakeFiles/pointers.dir/build

CMakeFiles/pointers.dir/requires: CMakeFiles/pointers.dir/pointers.cpp.o.requires

.PHONY : CMakeFiles/pointers.dir/requires

CMakeFiles/pointers.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pointers.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pointers.dir/clean

CMakeFiles/pointers.dir/depend:
	cd /home/ajal/git/pfms-2020a-ajalsingh/scratch/week01/examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ajal/git/pfms-2020a-ajalsingh/scratch/week01/examples /home/ajal/git/pfms-2020a-ajalsingh/scratch/week01/examples /home/ajal/git/pfms-2020a-ajalsingh/scratch/week01/examples/build /home/ajal/git/pfms-2020a-ajalsingh/scratch/week01/examples/build /home/ajal/git/pfms-2020a-ajalsingh/scratch/week01/examples/build/CMakeFiles/pointers.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pointers.dir/depend
