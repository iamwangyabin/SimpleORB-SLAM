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
include CMakeFiles/Simple_SLAM.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Simple_SLAM.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Simple_SLAM.dir/flags.make

CMakeFiles/Simple_SLAM.dir/src/System.cpp.o: CMakeFiles/Simple_SLAM.dir/flags.make
CMakeFiles/Simple_SLAM.dir/src/System.cpp.o: ../src/System.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wang/workspace/SimpleORB-SLAM/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Simple_SLAM.dir/src/System.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Simple_SLAM.dir/src/System.cpp.o -c /home/wang/workspace/SimpleORB-SLAM/src/System.cpp

CMakeFiles/Simple_SLAM.dir/src/System.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Simple_SLAM.dir/src/System.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wang/workspace/SimpleORB-SLAM/src/System.cpp > CMakeFiles/Simple_SLAM.dir/src/System.cpp.i

CMakeFiles/Simple_SLAM.dir/src/System.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Simple_SLAM.dir/src/System.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wang/workspace/SimpleORB-SLAM/src/System.cpp -o CMakeFiles/Simple_SLAM.dir/src/System.cpp.s

CMakeFiles/Simple_SLAM.dir/src/ORBextractor.cpp.o: CMakeFiles/Simple_SLAM.dir/flags.make
CMakeFiles/Simple_SLAM.dir/src/ORBextractor.cpp.o: ../src/ORBextractor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wang/workspace/SimpleORB-SLAM/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Simple_SLAM.dir/src/ORBextractor.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Simple_SLAM.dir/src/ORBextractor.cpp.o -c /home/wang/workspace/SimpleORB-SLAM/src/ORBextractor.cpp

CMakeFiles/Simple_SLAM.dir/src/ORBextractor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Simple_SLAM.dir/src/ORBextractor.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wang/workspace/SimpleORB-SLAM/src/ORBextractor.cpp > CMakeFiles/Simple_SLAM.dir/src/ORBextractor.cpp.i

CMakeFiles/Simple_SLAM.dir/src/ORBextractor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Simple_SLAM.dir/src/ORBextractor.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wang/workspace/SimpleORB-SLAM/src/ORBextractor.cpp -o CMakeFiles/Simple_SLAM.dir/src/ORBextractor.cpp.s

CMakeFiles/Simple_SLAM.dir/src/Tracking.cpp.o: CMakeFiles/Simple_SLAM.dir/flags.make
CMakeFiles/Simple_SLAM.dir/src/Tracking.cpp.o: ../src/Tracking.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wang/workspace/SimpleORB-SLAM/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Simple_SLAM.dir/src/Tracking.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Simple_SLAM.dir/src/Tracking.cpp.o -c /home/wang/workspace/SimpleORB-SLAM/src/Tracking.cpp

CMakeFiles/Simple_SLAM.dir/src/Tracking.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Simple_SLAM.dir/src/Tracking.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wang/workspace/SimpleORB-SLAM/src/Tracking.cpp > CMakeFiles/Simple_SLAM.dir/src/Tracking.cpp.i

CMakeFiles/Simple_SLAM.dir/src/Tracking.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Simple_SLAM.dir/src/Tracking.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wang/workspace/SimpleORB-SLAM/src/Tracking.cpp -o CMakeFiles/Simple_SLAM.dir/src/Tracking.cpp.s

# Object files for target Simple_SLAM
Simple_SLAM_OBJECTS = \
"CMakeFiles/Simple_SLAM.dir/src/System.cpp.o" \
"CMakeFiles/Simple_SLAM.dir/src/ORBextractor.cpp.o" \
"CMakeFiles/Simple_SLAM.dir/src/Tracking.cpp.o"

# External object files for target Simple_SLAM
Simple_SLAM_EXTERNAL_OBJECTS =

../lib/libSimple_SLAM.so: CMakeFiles/Simple_SLAM.dir/src/System.cpp.o
../lib/libSimple_SLAM.so: CMakeFiles/Simple_SLAM.dir/src/ORBextractor.cpp.o
../lib/libSimple_SLAM.so: CMakeFiles/Simple_SLAM.dir/src/Tracking.cpp.o
../lib/libSimple_SLAM.so: CMakeFiles/Simple_SLAM.dir/build.make
../lib/libSimple_SLAM.so: /usr/local/lib/libopencv_shape.so.3.2.0
../lib/libSimple_SLAM.so: /usr/local/lib/libopencv_stitching.so.3.2.0
../lib/libSimple_SLAM.so: /usr/local/lib/libopencv_superres.so.3.2.0
../lib/libSimple_SLAM.so: /usr/local/lib/libopencv_videostab.so.3.2.0
../lib/libSimple_SLAM.so: /home/wang/workspace/ORB_SLAM2-master/Thirdparty/Pangolin/build/src/libpangolin.so
../lib/libSimple_SLAM.so: /usr/local/lib/libopencv_objdetect.so.3.2.0
../lib/libSimple_SLAM.so: /usr/local/lib/libopencv_calib3d.so.3.2.0
../lib/libSimple_SLAM.so: /usr/local/lib/libopencv_features2d.so.3.2.0
../lib/libSimple_SLAM.so: /usr/local/lib/libopencv_flann.so.3.2.0
../lib/libSimple_SLAM.so: /usr/local/lib/libopencv_highgui.so.3.2.0
../lib/libSimple_SLAM.so: /usr/local/lib/libopencv_ml.so.3.2.0
../lib/libSimple_SLAM.so: /usr/local/lib/libopencv_photo.so.3.2.0
../lib/libSimple_SLAM.so: /usr/local/lib/libopencv_video.so.3.2.0
../lib/libSimple_SLAM.so: /usr/local/lib/libopencv_videoio.so.3.2.0
../lib/libSimple_SLAM.so: /usr/local/lib/libopencv_imgcodecs.so.3.2.0
../lib/libSimple_SLAM.so: /usr/local/lib/libopencv_imgproc.so.3.2.0
../lib/libSimple_SLAM.so: /usr/local/lib/libopencv_core.so.3.2.0
../lib/libSimple_SLAM.so: /usr/lib/x86_64-linux-gnu/libGLU.so
../lib/libSimple_SLAM.so: /usr/lib/x86_64-linux-gnu/libGL.so
../lib/libSimple_SLAM.so: /usr/lib/x86_64-linux-gnu/libGLEW.so
../lib/libSimple_SLAM.so: /usr/lib/x86_64-linux-gnu/libSM.so
../lib/libSimple_SLAM.so: /usr/lib/x86_64-linux-gnu/libICE.so
../lib/libSimple_SLAM.so: /usr/lib/x86_64-linux-gnu/libX11.so
../lib/libSimple_SLAM.so: /usr/lib/x86_64-linux-gnu/libXext.so
../lib/libSimple_SLAM.so: /usr/lib/x86_64-linux-gnu/libpng.so
../lib/libSimple_SLAM.so: /usr/lib/x86_64-linux-gnu/libz.so
../lib/libSimple_SLAM.so: CMakeFiles/Simple_SLAM.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wang/workspace/SimpleORB-SLAM/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library ../lib/libSimple_SLAM.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Simple_SLAM.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Simple_SLAM.dir/build: ../lib/libSimple_SLAM.so

.PHONY : CMakeFiles/Simple_SLAM.dir/build

CMakeFiles/Simple_SLAM.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Simple_SLAM.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Simple_SLAM.dir/clean

CMakeFiles/Simple_SLAM.dir/depend:
	cd /home/wang/workspace/SimpleORB-SLAM/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wang/workspace/SimpleORB-SLAM /home/wang/workspace/SimpleORB-SLAM /home/wang/workspace/SimpleORB-SLAM/cmake-build-debug /home/wang/workspace/SimpleORB-SLAM/cmake-build-debug /home/wang/workspace/SimpleORB-SLAM/cmake-build-debug/CMakeFiles/Simple_SLAM.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Simple_SLAM.dir/depend

