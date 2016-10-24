cmake_minimum_required(VERSION 2.8)
set(OpenCV_DIR "/usr/local/opencv-3.0.0/release")
project( SLAM )

find_package( OpenCV REQUIRED )

find_package( Boost REQUIRED )

include_directories( ${Boost_INCLUDE_DIR} )
link_directories( ${Boost_LIBRARY_DIRS} )
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 


add_executable( SLAM.exe SLAM.cpp )
target_link_libraries( SLAM.exe ${OpenCV_LIBS} ${Boost_LIBRARIES} )