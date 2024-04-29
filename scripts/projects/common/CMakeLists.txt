
cmake_minimum_required(VERSION 3.5)
project(gams_app)

message("CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_BUILD_TYPE debug)

find_package(madara REQUIRED)
find_package(gams REQUIRED)
find_package(Eigen3 REQUIRED)

# Set the path to your project root directory
set(PROJECT_ROOT ${CMAKE_SOURCE_DIR})

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
    ${PROJECT_ROOT}/src/*.c
    ${PROJECT_ROOT}/src/*.cpp 
)

file(GLOB_RECURSE HEADER_FILES CONFIGURE_DEPENDS
    ${PROJECT_ROOT}/src/*.h
    ${PROJECT_ROOT}/src/*.inl
)

include_directories(${PROJECT_ROOT})

add_executable(controller ${SRC_FILES} ${HEADER_FILES})
target_link_libraries(controller gams)
target_link_libraries(controller madara)

