# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/david/proyecto/src/prueba

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/david/proyecto/src/prueba

# Include any dependencies generated for this target.
include CMakeFiles/vandet.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/vandet.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/vandet.dir/flags.make

CMakeFiles/vandet.dir/src/vandet.cpp.o: CMakeFiles/vandet.dir/flags.make
CMakeFiles/vandet.dir/src/vandet.cpp.o: src/vandet.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/david/proyecto/src/prueba/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/vandet.dir/src/vandet.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/vandet.dir/src/vandet.cpp.o -c /home/david/proyecto/src/prueba/src/vandet.cpp

CMakeFiles/vandet.dir/src/vandet.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vandet.dir/src/vandet.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/david/proyecto/src/prueba/src/vandet.cpp > CMakeFiles/vandet.dir/src/vandet.cpp.i

CMakeFiles/vandet.dir/src/vandet.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vandet.dir/src/vandet.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/david/proyecto/src/prueba/src/vandet.cpp -o CMakeFiles/vandet.dir/src/vandet.cpp.s

CMakeFiles/vandet.dir/src/vandet.cpp.o.requires:
.PHONY : CMakeFiles/vandet.dir/src/vandet.cpp.o.requires

CMakeFiles/vandet.dir/src/vandet.cpp.o.provides: CMakeFiles/vandet.dir/src/vandet.cpp.o.requires
	$(MAKE) -f CMakeFiles/vandet.dir/build.make CMakeFiles/vandet.dir/src/vandet.cpp.o.provides.build
.PHONY : CMakeFiles/vandet.dir/src/vandet.cpp.o.provides

CMakeFiles/vandet.dir/src/vandet.cpp.o.provides.build: CMakeFiles/vandet.dir/src/vandet.cpp.o

# Object files for target vandet
vandet_OBJECTS = \
"CMakeFiles/vandet.dir/src/vandet.cpp.o"

# External object files for target vandet
vandet_EXTERNAL_OBJECTS =

devel/lib/prueba/vandet: CMakeFiles/vandet.dir/src/vandet.cpp.o
devel/lib/prueba/vandet: CMakeFiles/vandet.dir/build.make
devel/lib/prueba/vandet: CMakeFiles/vandet.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable devel/lib/prueba/vandet"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vandet.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/vandet.dir/build: devel/lib/prueba/vandet
.PHONY : CMakeFiles/vandet.dir/build

CMakeFiles/vandet.dir/requires: CMakeFiles/vandet.dir/src/vandet.cpp.o.requires
.PHONY : CMakeFiles/vandet.dir/requires

CMakeFiles/vandet.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/vandet.dir/cmake_clean.cmake
.PHONY : CMakeFiles/vandet.dir/clean

CMakeFiles/vandet.dir/depend:
	cd /home/david/proyecto/src/prueba && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/david/proyecto/src/prueba /home/david/proyecto/src/prueba /home/david/proyecto/src/prueba /home/david/proyecto/src/prueba /home/david/proyecto/src/prueba/CMakeFiles/vandet.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/vandet.dir/depend

