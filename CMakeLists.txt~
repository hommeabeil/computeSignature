cmake_minimum_required(VERSION 2.8.3)
project(histogramme_convert)

find_package( PCL REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable(convert
	src/convert/convert_file.cpp
	src/convert/mainProgram.cpp)
target_link_libraries(convert ${PCL_LIBRARIES})



