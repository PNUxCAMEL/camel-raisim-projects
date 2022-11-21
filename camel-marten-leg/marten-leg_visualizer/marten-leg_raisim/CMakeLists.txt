cmake_minimum_required(VERSION 3.5)
project(marten-leg_raisim LANGUAGES CXX)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${marten-leg_util_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} STATIC
        src/RobotVisualization.cpp
        )

add_library(marten-leg_visualizer::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
        raisim::raisim
        pthread
        )
