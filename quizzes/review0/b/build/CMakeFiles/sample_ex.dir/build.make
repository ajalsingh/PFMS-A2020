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
CMAKE_SOURCE_DIR = /home/ajal/git/pfms-2020a-ajalsingh/quizzes/review0/b

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ajal/git/pfms-2020a-ajalsingh/quizzes/review0/b/build

# Include any dependencies generated for this target.
include CMakeFiles/sample_ex.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sample_ex.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sample_ex.dir/flags.make

CMakeFiles/sample_ex.dir/main.cpp.o: CMakeFiles/sample_ex.dir/flags.make
CMakeFiles/sample_ex.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ajal/git/pfms-2020a-ajalsingh/quizzes/review0/b/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sample_ex.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sample_ex.dir/main.cpp.o -c /home/ajal/git/pfms-2020a-ajalsingh/quizzes/review0/b/main.cpp

CMakeFiles/sample_ex.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sample_ex.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ajal/git/pfms-2020a-ajalsingh/quizzes/review0/b/main.cpp > CMakeFiles/sample_ex.dir/main.cpp.i

CMakeFiles/sample_ex.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sample_ex.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ajal/git/pfms-2020a-ajalsingh/quizzes/review0/b/main.cpp -o CMakeFiles/sample_ex.dir/main.cpp.s

CMakeFiles/sample_ex.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/sample_ex.dir/main.cpp.o.requires

CMakeFiles/sample_ex.dir/main.cpp.o.provides: CMakeFiles/sample_ex.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/sample_ex.dir/build.make CMakeFiles/sample_ex.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/sample_ex.dir/main.cpp.o.provides

CMakeFiles/sample_ex.dir/main.cpp.o.provides.build: CMakeFiles/sample_ex.dir/main.cpp.o


CMakeFiles/sample_ex.dir/sample.cpp.o: CMakeFiles/sample_ex.dir/flags.make
CMakeFiles/sample_ex.dir/sample.cpp.o: ../sample.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ajal/git/pfms-2020a-ajalsingh/quizzes/review0/b/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/sample_ex.dir/sample.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sample_ex.dir/sample.cpp.o -c /home/ajal/git/pfms-2020a-ajalsingh/quizzes/review0/b/sample.cpp

CMakeFiles/sample_ex.dir/sample.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sample_ex.dir/sample.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ajal/git/pfms-2020a-ajalsingh/quizzes/review0/b/sample.cpp > CMakeFiles/sample_ex.dir/sample.cpp.i

CMakeFiles/sample_ex.dir/sample.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sample_ex.dir/sample.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ajal/git/pfms-2020a-ajalsingh/quizzes/review0/b/sample.cpp -o CMakeFiles/sample_ex.dir/sample.cpp.s

CMakeFiles/sample_ex.dir/sample.cpp.o.requires:

.PHONY : CMakeFiles/sample_ex.dir/sample.cpp.o.requires

CMakeFiles/sample_ex.dir/sample.cpp.o.provides: CMakeFiles/sample_ex.dir/sample.cpp.o.requires
	$(MAKE) -f CMakeFiles/sample_ex.dir/build.make CMakeFiles/sample_ex.dir/sample.cpp.o.provides.build
.PHONY : CMakeFiles/sample_ex.dir/sample.cpp.o.provides

CMakeFiles/sample_ex.dir/sample.cpp.o.provides.build: CMakeFiles/sample_ex.dir/sample.cpp.o


# Object files for target sample_ex
sample_ex_OBJECTS = \
"CMakeFiles/sample_ex.dir/main.cpp.o" \
"CMakeFiles/sample_ex.dir/sample.cpp.o"

# External object files for target sample_ex
sample_ex_EXTERNAL_OBJECTS =

sample_ex: CMakeFiles/sample_ex.dir/main.cpp.o
sample_ex: CMakeFiles/sample_ex.dir/sample.cpp.o
sample_ex: CMakeFiles/sample_ex.dir/build.make
sample_ex: CMakeFiles/sample_ex.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ajal/git/pfms-2020a-ajalsingh/quizzes/review0/b/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable sample_ex"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sample_ex.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sample_ex.dir/build: sample_ex

.PHONY : CMakeFiles/sample_ex.dir/build

CMakeFiles/sample_ex.dir/requires: CMakeFiles/sample_ex.dir/main.cpp.o.requires
CMakeFiles/sample_ex.dir/requires: CMakeFiles/sample_ex.dir/sample.cpp.o.requires

.PHONY : CMakeFiles/sample_ex.dir/requires

CMakeFiles/sample_ex.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sample_ex.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sample_ex.dir/clean

CMakeFiles/sample_ex.dir/depend:
	cd /home/ajal/git/pfms-2020a-ajalsingh/quizzes/review0/b/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ajal/git/pfms-2020a-ajalsingh/quizzes/review0/b /home/ajal/git/pfms-2020a-ajalsingh/quizzes/review0/b /home/ajal/git/pfms-2020a-ajalsingh/quizzes/review0/b/build /home/ajal/git/pfms-2020a-ajalsingh/quizzes/review0/b/build /home/ajal/git/pfms-2020a-ajalsingh/quizzes/review0/b/build/CMakeFiles/sample_ex.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sample_ex.dir/depend

