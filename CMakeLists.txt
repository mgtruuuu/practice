cmake_minimum_required(VERSION 3.2)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++14)

project(Test)

find_package(PCL REQUIRED)
find_package(libLAS REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS} ${OpenCV_LIBS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (Test main.cpp)
target_link_libraries (Test ${PCL_LIBRARIES} ${libLAS_LIBRARIES} ${OpenCV_LIBS})

