# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/zain/.local/lib/python3.10/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/zain/.local/lib/python3.10/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zain/PhD/IJCAI_23/AMAPF-Flow-BS

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zain/PhD/IJCAI_23/AMAPF-Flow-BS/build

# Include any dependencies generated for this target.
include CMakeFiles/flow_bs.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/flow_bs.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/flow_bs.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/flow_bs.dir/flags.make

CMakeFiles/flow_bs.dir/src/flow-bs.cpp.o: CMakeFiles/flow_bs.dir/flags.make
CMakeFiles/flow_bs.dir/src/flow-bs.cpp.o: /home/zain/PhD/IJCAI_23/AMAPF-Flow-BS/src/flow-bs.cpp
CMakeFiles/flow_bs.dir/src/flow-bs.cpp.o: CMakeFiles/flow_bs.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/zain/PhD/IJCAI_23/AMAPF-Flow-BS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/flow_bs.dir/src/flow-bs.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/flow_bs.dir/src/flow-bs.cpp.o -MF CMakeFiles/flow_bs.dir/src/flow-bs.cpp.o.d -o CMakeFiles/flow_bs.dir/src/flow-bs.cpp.o -c /home/zain/PhD/IJCAI_23/AMAPF-Flow-BS/src/flow-bs.cpp

CMakeFiles/flow_bs.dir/src/flow-bs.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/flow_bs.dir/src/flow-bs.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zain/PhD/IJCAI_23/AMAPF-Flow-BS/src/flow-bs.cpp > CMakeFiles/flow_bs.dir/src/flow-bs.cpp.i

CMakeFiles/flow_bs.dir/src/flow-bs.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/flow_bs.dir/src/flow-bs.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zain/PhD/IJCAI_23/AMAPF-Flow-BS/src/flow-bs.cpp -o CMakeFiles/flow_bs.dir/src/flow-bs.cpp.s

# Object files for target flow_bs
flow_bs_OBJECTS = \
"CMakeFiles/flow_bs.dir/src/flow-bs.cpp.o"

# External object files for target flow_bs
flow_bs_EXTERNAL_OBJECTS =

flow_bs: CMakeFiles/flow_bs.dir/src/flow-bs.cpp.o
flow_bs: CMakeFiles/flow_bs.dir/build.make
flow_bs: CMakeFiles/flow_bs.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/zain/PhD/IJCAI_23/AMAPF-Flow-BS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable flow_bs"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/flow_bs.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/flow_bs.dir/build: flow_bs
.PHONY : CMakeFiles/flow_bs.dir/build

CMakeFiles/flow_bs.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/flow_bs.dir/cmake_clean.cmake
.PHONY : CMakeFiles/flow_bs.dir/clean

CMakeFiles/flow_bs.dir/depend:
	cd /home/zain/PhD/IJCAI_23/AMAPF-Flow-BS/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zain/PhD/IJCAI_23/AMAPF-Flow-BS /home/zain/PhD/IJCAI_23/AMAPF-Flow-BS /home/zain/PhD/IJCAI_23/AMAPF-Flow-BS/build /home/zain/PhD/IJCAI_23/AMAPF-Flow-BS/build /home/zain/PhD/IJCAI_23/AMAPF-Flow-BS/build/CMakeFiles/flow_bs.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/flow_bs.dir/depend
