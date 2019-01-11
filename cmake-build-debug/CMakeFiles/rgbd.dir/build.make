# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /home/wang/software/clion-2018.3.2/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/wang/software/clion-2018.3.2/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/wang/workspace/SimpleORB-SLAM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wang/workspace/SimpleORB-SLAM/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/rgbd.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rgbd.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rgbd.dir/flags.make

CMakeFiles/rgbd.dir/Examples/rgbd.cpp.o: CMakeFiles/rgbd.dir/flags.make
CMakeFiles/rgbd.dir/Examples/rgbd.cpp.o: ../Examples/rgbd.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wang/workspace/SimpleORB-SLAM/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rgbd.dir/Examples/rgbd.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rgbd.dir/Examples/rgbd.cpp.o -c /home/wang/workspace/SimpleORB-SLAM/Examples/rgbd.cpp

CMakeFiles/rgbd.dir/Examples/rgbd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rgbd.dir/Examples/rgbd.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wang/workspace/SimpleORB-SLAM/Examples/rgbd.cpp > CMakeFiles/rgbd.dir/Examples/rgbd.cpp.i

CMakeFiles/rgbd.dir/Examples/rgbd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rgbd.dir/Examples/rgbd.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wang/workspace/SimpleORB-SLAM/Examples/rgbd.cpp -o CMakeFiles/rgbd.dir/Examples/rgbd.cpp.s

# Object files for target rgbd
rgbd_OBJECTS = \
"CMakeFiles/rgbd.dir/Examples/rgbd.cpp.o"

# External object files for target rgbd
rgbd_EXTERNAL_OBJECTS =

../Examples/rgbd: CMakeFiles/rgbd.dir/Examples/rgbd.cpp.o
../Examples/rgbd: CMakeFiles/rgbd.dir/build.make
../Examples/rgbd: ../lib/libSimple_SLAM.so
../Examples/rgbd: /usr/local/lib/libopencv_shape.so.3.2.0
../Examples/rgbd: /usr/local/lib/libopencv_stitching.so.3.2.0
../Examples/rgbd: /usr/local/lib/libopencv_objdetect.so.3.2.0
../Examples/rgbd: /usr/local/lib/libopencv_superres.so.3.2.0
../Examples/rgbd: /usr/local/lib/libopencv_videostab.so.3.2.0
../Examples/rgbd: /usr/local/lib/libopencv_calib3d.so.3.2.0
../Examples/rgbd: /usr/local/lib/libopencv_features2d.so.3.2.0
../Examples/rgbd: /usr/local/lib/libopencv_flann.so.3.2.0
../Examples/rgbd: /usr/local/lib/libopencv_highgui.so.3.2.0
../Examples/rgbd: /usr/local/lib/libopencv_ml.so.3.2.0
../Examples/rgbd: /usr/local/lib/libopencv_photo.so.3.2.0
../Examples/rgbd: /usr/local/lib/libopencv_video.so.3.2.0
../Examples/rgbd: /usr/local/lib/libopencv_videoio.so.3.2.0
../Examples/rgbd: /usr/local/lib/libopencv_imgcodecs.so.3.2.0
../Examples/rgbd: /usr/local/lib/libopencv_imgproc.so.3.2.0
../Examples/rgbd: /usr/local/lib/libopencv_core.so.3.2.0
../Examples/rgbd: /home/wang/workspace/ORB_SLAM2-master/Thirdparty/Pangolin/build/src/libpangolin.so
../Examples/rgbd: /usr/lib/x86_64-linux-gnu/libGLU.so
../Examples/rgbd: /usr/lib/x86_64-linux-gnu/libGL.so
../Examples/rgbd: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/rgbd: /usr/lib/x86_64-linux-gnu/libSM.so
../Examples/rgbd: /usr/lib/x86_64-linux-gnu/libICE.so
../Examples/rgbd: /usr/lib/x86_64-linux-gnu/libX11.so
../Examples/rgbd: /usr/lib/x86_64-linux-gnu/libXext.so
../Examples/rgbd: /usr/lib/x86_64-linux-gnu/libpng.so
../Examples/rgbd: /usr/lib/x86_64-linux-gnu/libz.so
../Examples/rgbd: CMakeFiles/rgbd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wang/workspace/SimpleORB-SLAM/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../Examples/rgbd"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rgbd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rgbd.dir/build: ../Examples/rgbd

.PHONY : CMakeFiles/rgbd.dir/build

CMakeFiles/rgbd.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rgbd.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rgbd.dir/clean

CMakeFiles/rgbd.dir/depend:
	cd /home/wang/workspace/SimpleORB-SLAM/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wang/workspace/SimpleORB-SLAM /home/wang/workspace/SimpleORB-SLAM /home/wang/workspace/SimpleORB-SLAM/cmake-build-debug /home/wang/workspace/SimpleORB-SLAM/cmake-build-debug /home/wang/workspace/SimpleORB-SLAM/cmake-build-debug/CMakeFiles/rgbd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rgbd.dir/depend

