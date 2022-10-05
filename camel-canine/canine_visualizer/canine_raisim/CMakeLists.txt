cmake_minimum_required(VERSION 3.5)
project(canine_raisim LANGUAGES CXX)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${canine_util_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} STATIC
        src/RobotVisualization.cpp
)

add_library(canine_visualizer::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
        raisim::raisim
        pthread
)

