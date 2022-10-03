cmake_minimum_required(VERSION 3.5)
project(camel-canine LANGUAGES CXX)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(raisim CONFIG REQUIRED)

add_subdirectory(canine_controller/convexMPC)
add_subdirectory(canine_visualizer)
add_subdirectory(canine_util)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${qpOASES_SOURCE_DIR}/include
        ${convexMPC_SOURCE_DIR}/include
        ${canine_visualizer_SOURCE_DIR}/include
        ${canine_util_SOURCE_DIR}/include
)

add_executable(camel-canine src/CanineMain.cpp )
target_link_libraries(camel-canine
        PRIVATE
        canine_controller::convexMPC
        canine_visualizer
        canine_util
)