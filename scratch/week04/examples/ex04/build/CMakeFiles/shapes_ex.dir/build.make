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
CMAKE_SOURCE_DIR = /home/ajal/git/pfms-2020a-ajalsingh/scratch/week04/examples/ex04

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ajal/git/pfms-2020a-ajalsingh/scratch/week04/examples/ex04/build

# Include any dependencies generated for this target.
include CMakeFiles/shapes_ex.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/shapes_ex.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/shapes_ex.dir/flags.make

CMakeFiles/shapes_ex.dir/main.cpp.o: CMakeFiles/shapes_ex.dir/flags.make
CMakeFiles/shapes_ex.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ajal/git/pfms-2020a-ajalsingh/scratch/week04/examples/ex04/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/shapes_ex.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/shapes_ex.dir/main.cpp.o -c /home/ajal/git/pfms-2020a-ajalsingh/scratch/week04/examples/ex04/main.cpp

CMakeFiles/shapes_ex.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shapes_ex.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ajal/git/pfms-2020a-ajalsingh/scratch/week04/examples/ex04/main.cpp > CMakeFiles/shapes_ex.dir/main.cpp.i

CMakeFiles/shapes_ex.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shapes_ex.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ajal/git/pfms-2020a-ajalsingh/scratch/week04/examples/ex04/main.cpp -o CMakeFiles/shapes_ex.dir/main.cpp.s

CMakeFiles/shapes_ex.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/shapes_ex.dir/main.cpp.o.requires

CMakeFiles/shapes_ex.dir/main.cpp.o.provides: CMakeFiles/shapes_ex.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/shapes_ex.dir/build.make CMakeFiles/shapes_ex.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/shapes_ex.dir/main.cpp.o.provides

CMakeFiles/shapes_ex.dir/main.cpp.o.provides.build: CMakeFiles/shapes_ex.dir/main.cpp.o


# Object files for target shapes_ex
shapes_ex_OBJECTS = \
"CMakeFiles/shapes_ex.dir/main.cpp.o"

# External object files for target shapes_ex
shapes_ex_EXTERNAL_OBJECTS =

shapes_ex: CMakeFiles/shapes_ex.dir/main.cpp.o
shapes_ex: CMakeFiles/shapes_ex.dir/build.make
shapes_ex: CMakeFiles/shapes_ex.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ajal/git/pfms-2020a-ajalsingh/scratch/week04/examples/ex04/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable shapes_ex"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/shapes_ex.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/shapes_ex.dir/build: shapes_ex

.PHONY : CMakeFiles/shapes_ex.dir/build

CMakeFiles/shapes_ex.dir/requires: CMakeFiles/shapes_ex.dir/main.cpp.o.requires

.PHONY : CMakeFiles/shapes_ex.dir/requires

CMakeFiles/shapes_ex.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/shapes_ex.dir/cmake_clean.cmake
.PHONY : CMakeFiles/shapes_ex.dir/clean

CMakeFiles/shapes_ex.dir/depend:
	cd /home/ajal/git/pfms-2020a-ajalsingh/scratch/week04/examples/ex04/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ajal/git/pfms-2020a-ajalsingh/scratch/week04/examples/ex04 /home/ajal/git/pfms-2020a-ajalsingh/scratch/week04/examples/ex04 /home/ajal/git/pfms-2020a-ajalsingh/scratch/week04/examples/ex04/build /home/ajal/git/pfms-2020a-ajalsingh/scratch/week04/examples/ex04/build /home/ajal/git/pfms-2020a-ajalsingh/scratch/week04/examples/ex04/build/CMakeFiles/shapes_ex.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/shapes_ex.dir/depend

