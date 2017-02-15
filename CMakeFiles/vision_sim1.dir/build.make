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
CMAKE_SOURCE_DIR = /home/robot/catkin_ws/src/truffle

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robot/catkin_ws/src/truffle

# Include any dependencies generated for this target.
include CMakeFiles/vision_sim1.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/vision_sim1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/vision_sim1.dir/flags.make

CMakeFiles/vision_sim1.dir/src/vision1.cpp.o: CMakeFiles/vision_sim1.dir/flags.make
CMakeFiles/vision_sim1.dir/src/vision1.cpp.o: src/vision1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot/catkin_ws/src/truffle/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/vision_sim1.dir/src/vision1.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vision_sim1.dir/src/vision1.cpp.o -c /home/robot/catkin_ws/src/truffle/src/vision1.cpp

CMakeFiles/vision_sim1.dir/src/vision1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vision_sim1.dir/src/vision1.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robot/catkin_ws/src/truffle/src/vision1.cpp > CMakeFiles/vision_sim1.dir/src/vision1.cpp.i

CMakeFiles/vision_sim1.dir/src/vision1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vision_sim1.dir/src/vision1.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robot/catkin_ws/src/truffle/src/vision1.cpp -o CMakeFiles/vision_sim1.dir/src/vision1.cpp.s

CMakeFiles/vision_sim1.dir/src/vision1.cpp.o.requires:

.PHONY : CMakeFiles/vision_sim1.dir/src/vision1.cpp.o.requires

CMakeFiles/vision_sim1.dir/src/vision1.cpp.o.provides: CMakeFiles/vision_sim1.dir/src/vision1.cpp.o.requires
	$(MAKE) -f CMakeFiles/vision_sim1.dir/build.make CMakeFiles/vision_sim1.dir/src/vision1.cpp.o.provides.build
.PHONY : CMakeFiles/vision_sim1.dir/src/vision1.cpp.o.provides

CMakeFiles/vision_sim1.dir/src/vision1.cpp.o.provides.build: CMakeFiles/vision_sim1.dir/src/vision1.cpp.o


# Object files for target vision_sim1
vision_sim1_OBJECTS = \
"CMakeFiles/vision_sim1.dir/src/vision1.cpp.o"

# External object files for target vision_sim1
vision_sim1_EXTERNAL_OBJECTS =

devel/lib/truffle/vision_sim1: CMakeFiles/vision_sim1.dir/src/vision1.cpp.o
devel/lib/truffle/vision_sim1: CMakeFiles/vision_sim1.dir/build.make
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_xphoto3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_xobjdetect3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_tracking3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_surface_matching3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_structured_light3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_stereo3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_saliency3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_rgbd3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_reg3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_plot3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_optflow3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_line_descriptor3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_hdf3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_fuzzy3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_dpm3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_dnn3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_datasets3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_cvv3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_ccalib3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_bioinspired3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_bgsegm3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_aruco3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_viz3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_videostab3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_superres3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_stitching3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_photo3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libcv_bridge.so
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_xphoto3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_xobjdetect3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_ximgproc3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_xfeatures2d3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_tracking3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_text3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_surface_matching3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_structured_light3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_stereo3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_saliency3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_rgbd3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_reg3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_plot3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_optflow3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_line_descriptor3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_hdf3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_fuzzy3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_face3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_dpm3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_dnn3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_datasets3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_cvv3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_ccalib3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_bioinspired3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_bgsegm3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_aruco3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_viz3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_videostab3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_videoio3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_video3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_superres3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_stitching3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_shape3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_photo3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_objdetect3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_ml3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_highgui3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_flann3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_features2d3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_core3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_calib3d3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libimage_transport.so
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libmessage_filters.so
devel/lib/truffle/vision_sim1: /usr/lib/x86_64-linux-gnu/libtinyxml.so
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libclass_loader.so
devel/lib/truffle/vision_sim1: /usr/lib/libPocoFoundation.so
devel/lib/truffle/vision_sim1: /usr/lib/x86_64-linux-gnu/libdl.so
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libroslib.so
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libroscpp.so
devel/lib/truffle/vision_sim1: /usr/lib/x86_64-linux-gnu/libboost_signals.so
devel/lib/truffle/vision_sim1: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/librosconsole.so
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
devel/lib/truffle/vision_sim1: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/truffle/vision_sim1: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libxmlrpcpp.so
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libroscpp_serialization.so
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/librostime.so
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libcpp_common.so
devel/lib/truffle/vision_sim1: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/truffle/vision_sim1: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/truffle/vision_sim1: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/truffle/vision_sim1: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/truffle/vision_sim1: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/truffle/vision_sim1: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/truffle/vision_sim1: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_text3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_face3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_ximgproc3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_xfeatures2d3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_shape3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_video3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_objdetect3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_calib3d3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_features2d3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_ml3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_highgui3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_videoio3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_flann3.so.3.1.0
devel/lib/truffle/vision_sim1: /opt/ros/kinetic/lib/libopencv_core3.so.3.1.0
devel/lib/truffle/vision_sim1: CMakeFiles/vision_sim1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robot/catkin_ws/src/truffle/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable devel/lib/truffle/vision_sim1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vision_sim1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/vision_sim1.dir/build: devel/lib/truffle/vision_sim1

.PHONY : CMakeFiles/vision_sim1.dir/build

CMakeFiles/vision_sim1.dir/requires: CMakeFiles/vision_sim1.dir/src/vision1.cpp.o.requires

.PHONY : CMakeFiles/vision_sim1.dir/requires

CMakeFiles/vision_sim1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/vision_sim1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/vision_sim1.dir/clean

CMakeFiles/vision_sim1.dir/depend:
	cd /home/robot/catkin_ws/src/truffle && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/catkin_ws/src/truffle /home/robot/catkin_ws/src/truffle /home/robot/catkin_ws/src/truffle /home/robot/catkin_ws/src/truffle /home/robot/catkin_ws/src/truffle/CMakeFiles/vision_sim1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/vision_sim1.dir/depend

