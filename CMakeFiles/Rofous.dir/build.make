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
CMAKE_SOURCE_DIR = /home/m_dyse/Dyse-Robotics/Projects/Rofous

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/m_dyse/Dyse-Robotics/Projects/Rofous

# Include any dependencies generated for this target.
include CMakeFiles/Rofous.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Rofous.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Rofous.dir/flags.make

CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.o: CMakeFiles/Rofous.dir/flags.make
CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.o: test/AD_U_Test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/m_dyse/Dyse-Robotics/Projects/Rofous/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.o -c /home/m_dyse/Dyse-Robotics/Projects/Rofous/test/AD_U_Test.cpp

CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/m_dyse/Dyse-Robotics/Projects/Rofous/test/AD_U_Test.cpp > CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.i

CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/m_dyse/Dyse-Robotics/Projects/Rofous/test/AD_U_Test.cpp -o CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.s

CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.o.requires:

.PHONY : CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.o.requires

CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.o.provides: CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.o.requires
	$(MAKE) -f CMakeFiles/Rofous.dir/build.make CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.o.provides.build
.PHONY : CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.o.provides

CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.o.provides.build: CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.o


CMakeFiles/Rofous.dir/src/main.cpp.o: CMakeFiles/Rofous.dir/flags.make
CMakeFiles/Rofous.dir/src/main.cpp.o: src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/m_dyse/Dyse-Robotics/Projects/Rofous/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Rofous.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Rofous.dir/src/main.cpp.o -c /home/m_dyse/Dyse-Robotics/Projects/Rofous/src/main.cpp

CMakeFiles/Rofous.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Rofous.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/m_dyse/Dyse-Robotics/Projects/Rofous/src/main.cpp > CMakeFiles/Rofous.dir/src/main.cpp.i

CMakeFiles/Rofous.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Rofous.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/m_dyse/Dyse-Robotics/Projects/Rofous/src/main.cpp -o CMakeFiles/Rofous.dir/src/main.cpp.s

CMakeFiles/Rofous.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/Rofous.dir/src/main.cpp.o.requires

CMakeFiles/Rofous.dir/src/main.cpp.o.provides: CMakeFiles/Rofous.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Rofous.dir/build.make CMakeFiles/Rofous.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/Rofous.dir/src/main.cpp.o.provides

CMakeFiles/Rofous.dir/src/main.cpp.o.provides.build: CMakeFiles/Rofous.dir/src/main.cpp.o


# Object files for target Rofous
Rofous_OBJECTS = \
"CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.o" \
"CMakeFiles/Rofous.dir/src/main.cpp.o"

# External object files for target Rofous
Rofous_EXTERNAL_OBJECTS =

Rofous: CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.o
Rofous: CMakeFiles/Rofous.dir/src/main.cpp.o
Rofous: CMakeFiles/Rofous.dir/build.make
Rofous: CMakeFiles/Rofous.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/m_dyse/Dyse-Robotics/Projects/Rofous/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable Rofous"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Rofous.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Rofous.dir/build: Rofous

.PHONY : CMakeFiles/Rofous.dir/build

CMakeFiles/Rofous.dir/requires: CMakeFiles/Rofous.dir/test/AD_U_Test.cpp.o.requires
CMakeFiles/Rofous.dir/requires: CMakeFiles/Rofous.dir/src/main.cpp.o.requires

.PHONY : CMakeFiles/Rofous.dir/requires

CMakeFiles/Rofous.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Rofous.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Rofous.dir/clean

CMakeFiles/Rofous.dir/depend:
	cd /home/m_dyse/Dyse-Robotics/Projects/Rofous && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/m_dyse/Dyse-Robotics/Projects/Rofous /home/m_dyse/Dyse-Robotics/Projects/Rofous /home/m_dyse/Dyse-Robotics/Projects/Rofous /home/m_dyse/Dyse-Robotics/Projects/Rofous /home/m_dyse/Dyse-Robotics/Projects/Rofous/CMakeFiles/Rofous.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Rofous.dir/depend

