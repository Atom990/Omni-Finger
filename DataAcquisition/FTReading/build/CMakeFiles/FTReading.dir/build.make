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
CMAKE_SOURCE_DIR = /home/zixin/BionicDL/FTReading

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zixin/BionicDL/FTReading/build

# Include any dependencies generated for this target.
include CMakeFiles/FTReading.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/FTReading.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/FTReading.dir/flags.make

CMakeFiles/FTReading.dir/src/Force.cpp.o: CMakeFiles/FTReading.dir/flags.make
CMakeFiles/FTReading.dir/src/Force.cpp.o: ../src/Force.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zixin/BionicDL/FTReading/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/FTReading.dir/src/Force.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FTReading.dir/src/Force.cpp.o -c /home/zixin/BionicDL/FTReading/src/Force.cpp

CMakeFiles/FTReading.dir/src/Force.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FTReading.dir/src/Force.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zixin/BionicDL/FTReading/src/Force.cpp > CMakeFiles/FTReading.dir/src/Force.cpp.i

CMakeFiles/FTReading.dir/src/Force.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FTReading.dir/src/Force.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zixin/BionicDL/FTReading/src/Force.cpp -o CMakeFiles/FTReading.dir/src/Force.cpp.s

CMakeFiles/FTReading.dir/src/Force.cpp.o.requires:

.PHONY : CMakeFiles/FTReading.dir/src/Force.cpp.o.requires

CMakeFiles/FTReading.dir/src/Force.cpp.o.provides: CMakeFiles/FTReading.dir/src/Force.cpp.o.requires
	$(MAKE) -f CMakeFiles/FTReading.dir/build.make CMakeFiles/FTReading.dir/src/Force.cpp.o.provides.build
.PHONY : CMakeFiles/FTReading.dir/src/Force.cpp.o.provides

CMakeFiles/FTReading.dir/src/Force.cpp.o.provides.build: CMakeFiles/FTReading.dir/src/Force.cpp.o


CMakeFiles/FTReading.dir/src/pyFT.cpp.o: CMakeFiles/FTReading.dir/flags.make
CMakeFiles/FTReading.dir/src/pyFT.cpp.o: ../src/pyFT.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zixin/BionicDL/FTReading/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/FTReading.dir/src/pyFT.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FTReading.dir/src/pyFT.cpp.o -c /home/zixin/BionicDL/FTReading/src/pyFT.cpp

CMakeFiles/FTReading.dir/src/pyFT.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FTReading.dir/src/pyFT.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zixin/BionicDL/FTReading/src/pyFT.cpp > CMakeFiles/FTReading.dir/src/pyFT.cpp.i

CMakeFiles/FTReading.dir/src/pyFT.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FTReading.dir/src/pyFT.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zixin/BionicDL/FTReading/src/pyFT.cpp -o CMakeFiles/FTReading.dir/src/pyFT.cpp.s

CMakeFiles/FTReading.dir/src/pyFT.cpp.o.requires:

.PHONY : CMakeFiles/FTReading.dir/src/pyFT.cpp.o.requires

CMakeFiles/FTReading.dir/src/pyFT.cpp.o.provides: CMakeFiles/FTReading.dir/src/pyFT.cpp.o.requires
	$(MAKE) -f CMakeFiles/FTReading.dir/build.make CMakeFiles/FTReading.dir/src/pyFT.cpp.o.provides.build
.PHONY : CMakeFiles/FTReading.dir/src/pyFT.cpp.o.provides

CMakeFiles/FTReading.dir/src/pyFT.cpp.o.provides.build: CMakeFiles/FTReading.dir/src/pyFT.cpp.o


# Object files for target FTReading
FTReading_OBJECTS = \
"CMakeFiles/FTReading.dir/src/Force.cpp.o" \
"CMakeFiles/FTReading.dir/src/pyFT.cpp.o"

# External object files for target FTReading
FTReading_EXTERNAL_OBJECTS =

FTReading.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/FTReading.dir/src/Force.cpp.o
FTReading.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/FTReading.dir/src/pyFT.cpp.o
FTReading.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/FTReading.dir/build.make
FTReading.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/FTReading.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zixin/BionicDL/FTReading/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared module FTReading.cpython-37m-x86_64-linux-gnu.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FTReading.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/FTReading.dir/build: FTReading.cpython-37m-x86_64-linux-gnu.so

.PHONY : CMakeFiles/FTReading.dir/build

CMakeFiles/FTReading.dir/requires: CMakeFiles/FTReading.dir/src/Force.cpp.o.requires
CMakeFiles/FTReading.dir/requires: CMakeFiles/FTReading.dir/src/pyFT.cpp.o.requires

.PHONY : CMakeFiles/FTReading.dir/requires

CMakeFiles/FTReading.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/FTReading.dir/cmake_clean.cmake
.PHONY : CMakeFiles/FTReading.dir/clean

CMakeFiles/FTReading.dir/depend:
	cd /home/zixin/BionicDL/FTReading/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zixin/BionicDL/FTReading /home/zixin/BionicDL/FTReading /home/zixin/BionicDL/FTReading/build /home/zixin/BionicDL/FTReading/build /home/zixin/BionicDL/FTReading/build/CMakeFiles/FTReading.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/FTReading.dir/depend
