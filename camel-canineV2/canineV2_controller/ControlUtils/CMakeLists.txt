cmake_minimum_required(VERSION 3.5)
project(ControlUtils LANGUAGES CXX)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${canineV2_util_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} STATIC
        src/Gait.cpp
        src/SwingLeg.cpp
        )

add_library(canineV2_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})