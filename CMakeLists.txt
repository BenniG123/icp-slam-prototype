cmake_minimum_required(VERSION 3.1)
set(OpenCV_DIR "/usr/local/opencv/release")
set(CMAKE_BUILD_TYPE Release)
project( SLAM CXX)

find_package( OpenCV REQUIRED )
set(CMAKE_CXX_STANDARD 11)

set(SOURCE
  ${SOURCE}
  icp.cpp
  quaternion.cpp
  pointcloud.cpp
  map.cpp
  SLAM.cpp
)

set(HEADERS
   ${HEADERS}
   SLAM.hpp
   icp.hpp
   quaternion.hpp
   pointcloud.hpp
   map.hpp
)

# add_library (quaternion SHARED quaternion.cpp)

add_executable( SLAM.exe ${SOURCE} ${HEADERS})
target_link_libraries( SLAM.exe ${OpenCV_LIBS})