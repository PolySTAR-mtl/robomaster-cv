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
CMAKE_SOURCE_DIR = /home/ozone/robomaster-2023-cv/ros_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ozone/robomaster-2023-cv/ros_ws/src

# Include any dependencies generated for this target.
include decision/CMakeFiles/simple_tracker.dir/depend.make

# Include the progress variables for this target.
include decision/CMakeFiles/simple_tracker.dir/progress.make

# Include the compile flags for this target's objects.
include decision/CMakeFiles/simple_tracker.dir/flags.make

decision/CMakeFiles/simple_tracker.dir/src/simple_tracker.cpp.o: decision/CMakeFiles/simple_tracker.dir/flags.make
decision/CMakeFiles/simple_tracker.dir/src/simple_tracker.cpp.o: decision/src/simple_tracker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ozone/robomaster-2023-cv/ros_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object decision/CMakeFiles/simple_tracker.dir/src/simple_tracker.cpp.o"
	cd /home/ozone/robomaster-2023-cv/ros_ws/src/decision && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simple_tracker.dir/src/simple_tracker.cpp.o -c /home/ozone/robomaster-2023-cv/ros_ws/src/decision/src/simple_tracker.cpp

decision/CMakeFiles/simple_tracker.dir/src/simple_tracker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simple_tracker.dir/src/simple_tracker.cpp.i"
	cd /home/ozone/robomaster-2023-cv/ros_ws/src/decision && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ozone/robomaster-2023-cv/ros_ws/src/decision/src/simple_tracker.cpp > CMakeFiles/simple_tracker.dir/src/simple_tracker.cpp.i

decision/CMakeFiles/simple_tracker.dir/src/simple_tracker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simple_tracker.dir/src/simple_tracker.cpp.s"
	cd /home/ozone/robomaster-2023-cv/ros_ws/src/decision && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ozone/robomaster-2023-cv/ros_ws/src/decision/src/simple_tracker.cpp -o CMakeFiles/simple_tracker.dir/src/simple_tracker.cpp.s

# Object files for target simple_tracker
simple_tracker_OBJECTS = \
"CMakeFiles/simple_tracker.dir/src/simple_tracker.cpp.o"

# External object files for target simple_tracker
simple_tracker_EXTERNAL_OBJECTS =

/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: decision/CMakeFiles/simple_tracker.dir/src/simple_tracker.cpp.o
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: decision/CMakeFiles/simple_tracker.dir/build.make
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: /opt/ros/noetic/lib/libroscpp.so
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: /opt/ros/noetic/lib/librosconsole.so
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: /opt/ros/noetic/lib/librostime.so
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: /opt/ros/noetic/lib/libcpp_common.so
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker: decision/CMakeFiles/simple_tracker.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ozone/robomaster-2023-cv/ros_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker"
	cd /home/ozone/robomaster-2023-cv/ros_ws/src/decision && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simple_tracker.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
decision/CMakeFiles/simple_tracker.dir/build: /home/ozone/robomaster-2023-cv/ros_ws/devel/lib/decision/simple_tracker

.PHONY : decision/CMakeFiles/simple_tracker.dir/build

decision/CMakeFiles/simple_tracker.dir/clean:
	cd /home/ozone/robomaster-2023-cv/ros_ws/src/decision && $(CMAKE_COMMAND) -P CMakeFiles/simple_tracker.dir/cmake_clean.cmake
.PHONY : decision/CMakeFiles/simple_tracker.dir/clean

decision/CMakeFiles/simple_tracker.dir/depend:
	cd /home/ozone/robomaster-2023-cv/ros_ws/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ozone/robomaster-2023-cv/ros_ws/src /home/ozone/robomaster-2023-cv/ros_ws/src/decision /home/ozone/robomaster-2023-cv/ros_ws/src /home/ozone/robomaster-2023-cv/ros_ws/src/decision /home/ozone/robomaster-2023-cv/ros_ws/src/decision/CMakeFiles/simple_tracker.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : decision/CMakeFiles/simple_tracker.dir/depend
