cmake_minimum_required(VERSION 3.7)
project(Play_With_Kitti)

set(CMAKE_CXX_STANDARD 11)
set(SOURCE_FILES main.cpp )
aux_source_directory(. SOURCE_FILES)
set(OpenCV_DIR "/Users/qiaoting/Documents/opencv/build/lib")

find_package( OpenCV REQUIRED COMPONENTS core imgproc highgui contrib)
include_directories(${OpenCV_INCLUDE_DIRS})

aux_source_directory(. DIR_SRCS)
add_executable(Play_With_Kitti ${SOURCE_FILES} Disparity_Map.cpp Disparity_Map.hpp)

target_link_libraries(Play_With_Kitti ${OpenCV_LIBS})