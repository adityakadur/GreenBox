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

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/odroid/SDL/openNI

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/odroid/SDL/openNI

# Include any dependencies generated for this target.
include CMakeFiles/openni2_pcl.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/openni2_pcl.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/openni2_pcl.dir/flags.make

CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.o: CMakeFiles/openni2_pcl.dir/flags.make
CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.o: openni2_pcl.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/odroid/SDL/openNI/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.o -c /home/odroid/SDL/openNI/openni2_pcl.cpp


CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/odroid/SDL/openNI/openni2_pcl.cpp > CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.i

CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/odroid/SDL/openNI/openni2_pcl.cpp -o CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.s

CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.o.requires:
.PHONY : CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.o.requires

CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.o.provides: CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.o.requires
	$(MAKE) -f CMakeFiles/openni2_pcl.dir/build.make CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.o.provides.build
.PHONY : CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.o.provides

CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.o.provides.build: CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.o

# Object files for target openni2_pcl
openni2_pcl_OBJECTS = \
"CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.o"

# External object files for target openni2_pcl
openni2_pcl_EXTERNAL_OBJECTS =

openni2_pcl: CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.o
openni2_pcl: CMakeFiles/openni2_pcl.dir/build.make
openni2_pcl: /home/odroid/OpenNI-Linux-Arm-2.2/Redist/libOpenNI2.so
openni2_pcl: /usr/local/lib/libopencv_videostab.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_video.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_ts.a
openni2_pcl: /usr/local/lib/libopencv_superres.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_stitching.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_photo.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_ocl.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_objdetect.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_nonfree.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_ml.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_legacy.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_imgproc.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_highgui.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_gpu.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_flann.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_features2d.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_core.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_contrib.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_calib3d.so.2.4.9
openni2_pcl: /usr/lib/arm-linux-gnueabihf/libboost_system.so
openni2_pcl: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
openni2_pcl: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
openni2_pcl: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
openni2_pcl: /usr/lib/arm-linux-gnueabihf/libboost_iostreams.so
openni2_pcl: /usr/lib/arm-linux-gnueabihf/libboost_serialization.so
openni2_pcl: /usr/lib/arm-linux-gnueabihf/libpthread.so
openni2_pcl: /usr/local/lib/libpcl_common.so
openni2_pcl: /usr/lib/arm-linux-gnueabihf/libflann_cpp_s.a
openni2_pcl: /usr/local/lib/libpcl_kdtree.so
openni2_pcl: /usr/local/lib/libpcl_octree.so
openni2_pcl: /usr/local/lib/libpcl_search.so
openni2_pcl: /usr/local/lib/libpcl_sample_consensus.so
openni2_pcl: /usr/local/lib/libpcl_filters.so
openni2_pcl: /usr/lib/libvtkCommon.so.5.8.0
openni2_pcl: /usr/lib/libvtkFiltering.so.5.8.0
openni2_pcl: /usr/lib/libvtkImaging.so.5.8.0
openni2_pcl: /usr/lib/libvtkGraphics.so.5.8.0
openni2_pcl: /usr/lib/libvtkGenericFiltering.so.5.8.0
openni2_pcl: /usr/lib/libvtkIO.so.5.8.0
openni2_pcl: /usr/lib/libvtkRendering.so.5.8.0
openni2_pcl: /usr/lib/libvtkVolumeRendering.so.5.8.0
openni2_pcl: /usr/lib/libvtkHybrid.so.5.8.0
openni2_pcl: /usr/lib/libvtkWidgets.so.5.8.0
openni2_pcl: /usr/lib/libvtkParallel.so.5.8.0
openni2_pcl: /usr/lib/libvtkInfovis.so.5.8.0
openni2_pcl: /usr/lib/libvtkGeovis.so.5.8.0
openni2_pcl: /usr/lib/libvtkViews.so.5.8.0
openni2_pcl: /usr/lib/libvtkCharts.so.5.8.0
openni2_pcl: /usr/local/lib/libpcl_io.so
openni2_pcl: /usr/local/lib/libpcl_features.so
openni2_pcl: /usr/local/lib/libpcl_visualization.so
openni2_pcl: /usr/local/lib/libpcl_ml.so
openni2_pcl: /usr/local/lib/libpcl_segmentation.so
openni2_pcl: /usr/local/lib/libpcl_people.so
openni2_pcl: /usr/local/lib/libpcl_keypoints.so
openni2_pcl: /usr/lib/arm-linux-gnueabihf/libqhull.so
openni2_pcl: /usr/local/lib/libpcl_surface.so
openni2_pcl: /usr/local/lib/libpcl_registration.so
openni2_pcl: /usr/local/lib/libpcl_recognition.so
openni2_pcl: /usr/local/lib/libpcl_outofcore.so
openni2_pcl: /usr/local/lib/libpcl_tracking.so
openni2_pcl: /usr/local/lib/libpcl_stereo.so
openni2_pcl: /usr/lib/arm-linux-gnueabihf/libboost_system.so
openni2_pcl: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
openni2_pcl: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
openni2_pcl: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
openni2_pcl: /usr/lib/arm-linux-gnueabihf/libboost_iostreams.so
openni2_pcl: /usr/lib/arm-linux-gnueabihf/libboost_serialization.so
openni2_pcl: /usr/lib/arm-linux-gnueabihf/libpthread.so
openni2_pcl: /usr/lib/arm-linux-gnueabihf/libqhull.so
openni2_pcl: /usr/lib/arm-linux-gnueabihf/libflann_cpp_s.a
openni2_pcl: /usr/lib/libvtkCommon.so.5.8.0
openni2_pcl: /usr/lib/libvtkFiltering.so.5.8.0
openni2_pcl: /usr/lib/libvtkImaging.so.5.8.0
openni2_pcl: /usr/lib/libvtkGraphics.so.5.8.0
openni2_pcl: /usr/lib/libvtkGenericFiltering.so.5.8.0
openni2_pcl: /usr/lib/libvtkIO.so.5.8.0
openni2_pcl: /usr/lib/libvtkRendering.so.5.8.0
openni2_pcl: /usr/lib/libvtkVolumeRendering.so.5.8.0
openni2_pcl: /usr/lib/libvtkHybrid.so.5.8.0
openni2_pcl: /usr/lib/libvtkWidgets.so.5.8.0
openni2_pcl: /usr/lib/libvtkParallel.so.5.8.0
openni2_pcl: /usr/lib/libvtkInfovis.so.5.8.0
openni2_pcl: /usr/lib/libvtkGeovis.so.5.8.0
openni2_pcl: /usr/lib/libvtkViews.so.5.8.0
openni2_pcl: /usr/lib/libvtkCharts.so.5.8.0
openni2_pcl: /usr/local/lib/libpcl_common.so
openni2_pcl: /usr/local/lib/libpcl_kdtree.so
openni2_pcl: /usr/local/lib/libpcl_octree.so
openni2_pcl: /usr/local/lib/libpcl_search.so
openni2_pcl: /usr/local/lib/libpcl_sample_consensus.so
openni2_pcl: /usr/local/lib/libpcl_filters.so
openni2_pcl: /usr/local/lib/libpcl_io.so
openni2_pcl: /usr/local/lib/libpcl_features.so
openni2_pcl: /usr/local/lib/libpcl_visualization.so
openni2_pcl: /usr/local/lib/libpcl_ml.so
openni2_pcl: /usr/local/lib/libpcl_segmentation.so
openni2_pcl: /usr/local/lib/libpcl_people.so
openni2_pcl: /usr/local/lib/libpcl_keypoints.so
openni2_pcl: /usr/local/lib/libpcl_surface.so
openni2_pcl: /usr/local/lib/libpcl_registration.so
openni2_pcl: /usr/local/lib/libpcl_recognition.so
openni2_pcl: /usr/local/lib/libpcl_outofcore.so
openni2_pcl: /usr/local/lib/libpcl_tracking.so
openni2_pcl: /usr/local/lib/libpcl_stereo.so
openni2_pcl: /usr/local/lib/libopencv_nonfree.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_ocl.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_gpu.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_photo.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_objdetect.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_legacy.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_video.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_ml.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_calib3d.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_features2d.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_highgui.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_imgproc.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_flann.so.2.4.9
openni2_pcl: /usr/local/lib/libopencv_core.so.2.4.9
openni2_pcl: /usr/local/lib/libtbb.so
openni2_pcl: /usr/lib/libvtkViews.so.5.8.0
openni2_pcl: /usr/lib/libvtkInfovis.so.5.8.0
openni2_pcl: /usr/lib/libvtkWidgets.so.5.8.0
openni2_pcl: /usr/lib/libvtkVolumeRendering.so.5.8.0
openni2_pcl: /usr/lib/libvtkHybrid.so.5.8.0
openni2_pcl: /usr/lib/libvtkParallel.so.5.8.0
openni2_pcl: /usr/lib/libvtkRendering.so.5.8.0
openni2_pcl: /usr/lib/libvtkImaging.so.5.8.0
openni2_pcl: /usr/lib/libvtkGraphics.so.5.8.0
openni2_pcl: /usr/lib/libvtkIO.so.5.8.0
openni2_pcl: /usr/lib/libvtkFiltering.so.5.8.0
openni2_pcl: /usr/lib/libvtkCommon.so.5.8.0
openni2_pcl: /usr/lib/libvtksys.so.5.8.0
openni2_pcl: CMakeFiles/openni2_pcl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable openni2_pcl"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/openni2_pcl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/openni2_pcl.dir/build: openni2_pcl
.PHONY : CMakeFiles/openni2_pcl.dir/build

CMakeFiles/openni2_pcl.dir/requires: CMakeFiles/openni2_pcl.dir/openni2_pcl.cpp.o.requires
.PHONY : CMakeFiles/openni2_pcl.dir/requires

CMakeFiles/openni2_pcl.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/openni2_pcl.dir/cmake_clean.cmake
.PHONY : CMakeFiles/openni2_pcl.dir/clean

CMakeFiles/openni2_pcl.dir/depend:
	cd /home/odroid/SDL/openNI && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/odroid/SDL/openNI /home/odroid/SDL/openNI /home/odroid/SDL/openNI /home/odroid/SDL/openNI /home/odroid/SDL/openNI/CMakeFiles/openni2_pcl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/openni2_pcl.dir/depend

