#Define cmake version
cmake_minimum_required(VERSION 3.7)
#Define the project
project(Play_With_Kitti)
#Define the complier
set(CMAKE_CXX_STANDARD 11)
#Add all the file to compiler path
aux_source_directory(. SOURCE_FILES)

#Define the path of oepncv
set(OpenCV_DIR "/Users/qiaoting/Documents/opencv/build/lib")
#Define the linker files
find_package( OpenCV REQUIRED COMPONENTS core imgproc highgui contrib)
#Define the include of OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

#Compile the files
add_executable(Play_With_Kitti ${SOURCE_FILES} feature_matching.cpp feature_matching.h)
#Define the lib path
target_link_libraries(Play_With_Kitti ${OpenCV_LIBS})