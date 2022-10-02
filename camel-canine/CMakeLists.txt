cmake_minimum_required(VERSION 3.5)
project(camel-canine LANGUAGES CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(canine_controller/convexMPC)
add_subdirectory(canine_visualizer)

include_directories(
        include
        ${convexMPC_SOURCE_DIR}/include
        ${canine_visualizer_SOURCE_DIR}/include
)

add_executable(camel-canine src/CanineMain.cpp )
target_link_libraries(camel-canine
        PRIVATE
        canine_controller::convexMPC
        canine_visualizer
)