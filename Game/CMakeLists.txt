project (openni2_example)
cmake_minimum_required(VERSION 2.8)

# CONFIGURE OPENCV
find_package (OpenCV REQUIRED)

# CONFIGURE OPENNI2
find_library(OPENNI2_LIBRARY
	NAMES OpenNI2
	PATHS "~/OpenNI-Linux-Arm-2.2/Redist"
)
find_path(OPENNI2_INCLUDE_DIR OpenNI.h
	PATHS "~/OpenNI-Linux-Arm-2.2/Include"
)
link_directories(${OPENNI2_LIBRARY})
include_directories(${OPENNI2_INCLUDE_DIR})

# CONFIGURE PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# CREATE EXECUTABLE
add_executable (greenBox openni2_pcl.cpp)
target_link_libraries(greenBox ${OPENNI2_LIBRARY} ${OpenCV_LIBS} ${PCL_LIBRARIES})
