cmake_minimum_required(VERSION 3.5)
project(PDcontroller LANGUAGES CXX)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${canine_util_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} STATIC
        src/JointPDController.cpp
        src/BezierTrajectoryGenerator.cpp
        src/CubicTrajectoryGenerator.cpp
        src/QuinticTrajectoryGenerator.cpp
        src/SincurveTrajectoryGenerator.cpp
)

add_library(canine_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
        raisim::raisim
        canine_util
)