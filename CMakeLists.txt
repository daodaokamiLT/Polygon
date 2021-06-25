project("polygon")
cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}
    "/usr/include/eigen3")

# add_library(polygon SHARED core/polygon.cpp)

add_executable(test_createpolygon test_createpolygon.cpp)
target_link_libraries(test_createpolygon ${OpenCV_LIBS})

add_executable(testQuad testQuad.cpp)