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
CMAKE_SOURCE_DIR = /home/erik/Documents/Robotics/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/erik/Documents/Robotics/build

# Include any dependencies generated for this target.
include project/CMakeFiles/project.dir/depend.make

# Include the progress variables for this target.
include project/CMakeFiles/project.dir/progress.make

# Include the compile flags for this target's objects.
include project/CMakeFiles/project.dir/flags.make

project/CMakeFiles/project.dir/src/project/project_node.cpp.o: project/CMakeFiles/project.dir/flags.make
project/CMakeFiles/project.dir/src/project/project_node.cpp.o: /home/erik/Documents/Robotics/src/project/src/project/project_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/erik/Documents/Robotics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object project/CMakeFiles/project.dir/src/project/project_node.cpp.o"
	cd /home/erik/Documents/Robotics/build/project && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/project.dir/src/project/project_node.cpp.o -c /home/erik/Documents/Robotics/src/project/src/project/project_node.cpp

project/CMakeFiles/project.dir/src/project/project_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/project.dir/src/project/project_node.cpp.i"
	cd /home/erik/Documents/Robotics/build/project && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/erik/Documents/Robotics/src/project/src/project/project_node.cpp > CMakeFiles/project.dir/src/project/project_node.cpp.i

project/CMakeFiles/project.dir/src/project/project_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/project.dir/src/project/project_node.cpp.s"
	cd /home/erik/Documents/Robotics/build/project && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/erik/Documents/Robotics/src/project/src/project/project_node.cpp -o CMakeFiles/project.dir/src/project/project_node.cpp.s

project/CMakeFiles/project.dir/src/project/KDLSubscriber.cpp.o: project/CMakeFiles/project.dir/flags.make
project/CMakeFiles/project.dir/src/project/KDLSubscriber.cpp.o: /home/erik/Documents/Robotics/src/project/src/project/KDLSubscriber.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/erik/Documents/Robotics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object project/CMakeFiles/project.dir/src/project/KDLSubscriber.cpp.o"
	cd /home/erik/Documents/Robotics/build/project && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/project.dir/src/project/KDLSubscriber.cpp.o -c /home/erik/Documents/Robotics/src/project/src/project/KDLSubscriber.cpp

project/CMakeFiles/project.dir/src/project/KDLSubscriber.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/project.dir/src/project/KDLSubscriber.cpp.i"
	cd /home/erik/Documents/Robotics/build/project && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/erik/Documents/Robotics/src/project/src/project/KDLSubscriber.cpp > CMakeFiles/project.dir/src/project/KDLSubscriber.cpp.i

project/CMakeFiles/project.dir/src/project/KDLSubscriber.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/project.dir/src/project/KDLSubscriber.cpp.s"
	cd /home/erik/Documents/Robotics/build/project && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/erik/Documents/Robotics/src/project/src/project/KDLSubscriber.cpp -o CMakeFiles/project.dir/src/project/KDLSubscriber.cpp.s

# Object files for target project
project_OBJECTS = \
"CMakeFiles/project.dir/src/project/project_node.cpp.o" \
"CMakeFiles/project.dir/src/project/KDLSubscriber.cpp.o"

# External object files for target project
project_EXTERNAL_OBJECTS =

/home/erik/Documents/Robotics/devel/lib/libproject.so: project/CMakeFiles/project.dir/src/project/project_node.cpp.o
/home/erik/Documents/Robotics/devel/lib/libproject.so: project/CMakeFiles/project.dir/src/project/KDLSubscriber.cpp.o
/home/erik/Documents/Robotics/devel/lib/libproject.so: project/CMakeFiles/project.dir/build.make
/home/erik/Documents/Robotics/devel/lib/libproject.so: project/CMakeFiles/project.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/erik/Documents/Robotics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library /home/erik/Documents/Robotics/devel/lib/libproject.so"
	cd /home/erik/Documents/Robotics/build/project && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/project.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
project/CMakeFiles/project.dir/build: /home/erik/Documents/Robotics/devel/lib/libproject.so

.PHONY : project/CMakeFiles/project.dir/build

project/CMakeFiles/project.dir/clean:
	cd /home/erik/Documents/Robotics/build/project && $(CMAKE_COMMAND) -P CMakeFiles/project.dir/cmake_clean.cmake
.PHONY : project/CMakeFiles/project.dir/clean

project/CMakeFiles/project.dir/depend:
	cd /home/erik/Documents/Robotics/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/erik/Documents/Robotics/src /home/erik/Documents/Robotics/src/project /home/erik/Documents/Robotics/build /home/erik/Documents/Robotics/build/project /home/erik/Documents/Robotics/build/project/CMakeFiles/project.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : project/CMakeFiles/project.dir/depend
