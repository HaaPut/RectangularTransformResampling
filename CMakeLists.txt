cmake_minimum_required(VERSION 3.15)
project(RectangularTransformResampling)

find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

set(CMAKE_CXX_STANDARD 14)

include_directories(include)

set(SOURCES main.cpp)

add_executable(SphericalMapping main.cpp)

target_link_libraries(SphericalMapping ${ITK_LIBRARIES})
